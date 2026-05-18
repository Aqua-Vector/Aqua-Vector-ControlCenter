#include "LidarNode.h"
#include <iostream>
#include <cmath>
#include <algorithm> 
#include <chrono>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace sl;

// 생성자, 소멸자는 기존과 동일하여 생략 (그대로 사용하시면 됩니다)
LidarNode::LidarNode(const char* port, sl_u32 baudrate) : is_connected(false) {
    drv = *createLidarDriver();
    if (!drv) {
        std::cerr << "[Lidar Error] Insufficient memory." << std::endl;
        return;
    }

    IChannel* _channel = *createSerialPortChannel(port, baudrate);
    if (SL_IS_OK((drv)->connect(_channel))) {
        drv->setMotorSpeed();
        drv->startScan(0, 1);
        is_connected = true;
        std::cout << "[Lidar Info] Connected and scanning started." << std::endl;
    } else {
        std::cerr << "[Lidar Error] Connection failed to " << port << std::endl;
    }
}

LidarNode::~LidarNode() {
    if (drv) {
        drv->stop();
        drv->setMotorSpeed(0);
        delete drv;
        drv = NULL;
    }
}

// 1. 전방 좌우 40도로 수정
void LidarNode::preprocessData(sl_lidar_response_measurement_node_hq_t* raw_nodes, size_t count, std::vector<sl_lidar_response_measurement_node_hq_t>& valid_nodes) {
    for (size_t i = 0; i < count; ++i) {
        int quality = raw_nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        float angle_deg = (raw_nodes[i].angle_z_q14 * 90.f) / 16384.f;
        
        // Quality가 0 초과이고, 전방 좌우 40도 (0~40도, 320~360도) 이내인 데이터만 추출
        if (quality > 0 && (angle_deg <= 40.0f || angle_deg >= 320.0f)) {
            valid_nodes.push_back(raw_nodes[i]);
        }
    }
}

// 2. 초기화 명령어(캘리브레이션) 로직: 지정된 시간 동안 배경의 정적 맵을 그림
void LidarNode::startCalibration(int duration_sec) {
    std::cout << "[Lidar] Background Calibration Started (" << duration_sec << " sec)..." << std::endl;
    background_points.clear();
    
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(duration_sec)) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);
        if (SL_IS_OK(drv->grabScanDataHq(nodes, count))) {
            drv->ascendScanData(nodes, count);
            std::vector<sl_lidar_response_measurement_node_hq_t> valid_nodes;
            std::vector<Point2D> points;
            
            preprocessData(nodes, count, valid_nodes);
            transformCoordinates(valid_nodes, points);
            
            // 데이터 폭발을 막기 위해 5cm 간격으로 배경점(Background Point) 다운샘플링 저장
            for (const auto& p : points) {
                bool too_close = false;
                for (const auto& bg : background_points) {
                    if (std::hypot(p.x - bg.x, p.y - bg.y) < 50.0f) { // 50mm 이내면 중복 저장 안함
                        too_close = true;
                        break;
                    }
                }
                if (!too_close) {
                    background_points.push_back(p);
                }
            }
        }
    }
    std::cout << "[Lidar] Calibration Done. Static points mapped: " << background_points.size() << std::endl;
}

bool LidarNode::getDynamicCarPosition(Point2D& car_pos) {
    if (!is_connected) return false;

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    if (SL_IS_OK(drv->grabScanDataHq(nodes, count))) {
        drv->ascendScanData(nodes, count); 
        
        std::vector<sl_lidar_response_measurement_node_hq_t> valid_nodes;
        std::vector<Point2D> points, filtered_points;

        preprocessData(nodes, count, valid_nodes);
        transformCoordinates(valid_nodes, points);
        filterBackground(points, filtered_points); // 정적 배경 제거
        
        std::vector<std::vector<Point2D>> clusters = doClustering(filtered_points);

        // 가장 가까운 혹은 가장 뚜렷한 첫 번째 클러스터를 어뢰로 간주
        if (!clusters.empty() && clusters[0].size() > 2) {
            float sum_x = 0, sum_y = 0;
            for (const auto& p : clusters[0]) {
                sum_x += p.x;
                sum_y += p.y;
            }
            car_pos.x = sum_x / clusters[0].size();
            car_pos.y = sum_y / clusters[0].size();
            return true;
        }
    }
    return false;
}

// 3. 배경(정적 객체) 걸러내기
void LidarNode::filterBackground(const std::vector<Point2D>& input, std::vector<Point2D>& output) {
    if (background_points.empty()) {
        output = input; // 초기화 전이면 전부 동적 객체로 취급
        return;
    }
    for (const auto& p : input) {
        bool is_static = false;
        for (const auto& bg : background_points) {
            // 초기화 때 찍힌 배경좌표와 300mm(30cm) 이내라면 무시 (의자, 벽 등)
            if (std::hypot(p.x - bg.x, p.y - bg.y) < 300.0f) {
                is_static = true;
                break;
            }
        }
        if (!is_static) {
            output.push_back(p);
        }
    }
}

// transformCoordinates는 기존과 동일
void LidarNode::transformCoordinates(const std::vector<sl_lidar_response_measurement_node_hq_t>& valid_nodes, std::vector<Point2D>& points) {
    for (const auto& node : valid_nodes) {
        float angle_deg = (node.angle_z_q14 * 90.f) / 16384.f;
        float dist_mm = node.dist_mm_q2 / 4.0f;
        float angle_rad = angle_deg * (M_PI / 180.0f);
        points.push_back({
        static_cast<float>(dist_mm * sin(angle_rad)), 
        static_cast<float>(dist_mm * cos(angle_rad))
        });
    }
}

// 4. 클러스터링 로직: 중심점이 아닌 모든 점의 묶음을 반환
std::vector<std::vector<Point2D>> LidarNode::doClustering(const std::vector<Point2D>& points) {
    std::vector<std::vector<Point2D>> clusters;
    if (points.empty()) return clusters;

    float threshold = 300.0f; // 30cm 이내 동일 객체
    std::vector<Point2D> current_cluster;
    current_cluster.push_back(points[0]);

    for (size_t i = 1; i < points.size(); ++i) {
        float dx = points[i].x - points[i-1].x;
        float dy = points[i].y - points[i-1].y;
        float dist = std::sqrt(dx*dx + dy*dy);

        if (dist < threshold) {
            current_cluster.push_back(points[i]);
        } else {
            clusters.push_back(current_cluster);
            current_cluster.clear();
            current_cluster.push_back(points[i]);
        }
    }
    if (!current_cluster.empty()) {
        clusters.push_back(current_cluster);
    }
    return clusters;
}

// 5. 최종 반환 함수 (5cm 그리드로 변환 및 다중 벡터 패키징)
std::vector<std::vector<PointGrid>> LidarNode::getGridObstacles() {
    std::vector<std::vector<PointGrid>> grid_clusters;
    if (!is_connected) return grid_clusters;

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    sl_result op_result = drv->grabScanDataHq(nodes, count);

    if (SL_IS_OK(op_result)) {
        drv->ascendScanData(nodes, count); 
        
        std::vector<sl_lidar_response_measurement_node_hq_t> valid_nodes;
        std::vector<Point2D> points;
        std::vector<Point2D> filtered_points;

        preprocessData(nodes, count, valid_nodes);
        transformCoordinates(valid_nodes, points);
        
        // 배경지우개 동작 (캘리브레이션 안했으면 스킵됨)
        filterBackground(points, filtered_points);
        
        std::vector<std::vector<Point2D>> clusters = doClustering(filtered_points);

        for (const auto& cluster : clusters) {
            std::vector<PointGrid> grid_cluster;
            for (const auto& p : cluster) {
                PointGrid pg = {
                    static_cast<int>(std::round(p.x / 50.0f)),
                    static_cast<int>(std::round(p.y / 50.0f))
                };
                
                // 연속된 중복 좌표 제거 (예: [2,3]이 10개 찍히는 것 방지)
                if (grid_cluster.empty() || grid_cluster.back().x != pg.x || grid_cluster.back().y != pg.y) {
                    grid_cluster.push_back(pg);
                }
            }
            grid_clusters.push_back(grid_cluster);
        }
    }
    return grid_clusters;
}

// getObstacles()는 반환형이 달라져서 현재는 비워두거나, 기존 코드 유지하셔도 됩니다.