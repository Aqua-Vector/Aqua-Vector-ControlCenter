#include "LidarNode.h"
#include <iostream>
#include <cmath>
#include <algorithm> 
#include <chrono>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace sl;

// 생성자: Lidar 장치 연결 및 스캔 파라미터 초기 설정
LidarNode::LidarNode(const char* port, sl_u32 baudrate) : is_connected(false) {
    fprintf(stderr, "  [Lidar] Creating driver...\n");
    fflush(stderr);
    
    drv = *createLidarDriver();
    if (!drv) {
        fprintf(stderr, "  [Lidar Error] Insufficient memory\n");
        fflush(stderr);
        return;
    }
    
    fprintf(stderr, "  [Lidar] Opening port %s at %u baud...\n", port, baudrate);
    fflush(stderr);

    IChannel* _channel = *createSerialPortChannel(port, baudrate);
    if (SL_IS_OK((drv)->connect(_channel))) {
        fprintf(stderr, "  [Lidar] Connected, starting motor...\n");
        fflush(stderr);
        
        drv->setMotorSpeed(); 
        drv->startScan(0, 1); 
        is_connected = true;
        
        fprintf(stderr, "  [Lidar] Scanning started\n");
        fflush(stderr);
    } else {
        fprintf(stderr, "  [Lidar Error] Connection failed\n");
        fflush(stderr);
    }
}

// 소멸자: 장치 정지 및 메모리 자원 해제
LidarNode::~LidarNode() {
    if (drv) {
        drv->stop();
        drv->setMotorSpeed(0);
        delete drv;
        drv = NULL;
    }
}

/**
 * 1. 데이터 전처리: 유효 데이터 선별 및 FOV(전방 좌우 40도) 제한
 * @param raw_nodes: Lidar로부터 수신한 원시 데이터
 * @param count: 데이터 개수
 * @param valid_nodes: 필터링을 거친 유효 데이터 저장 벡터
 */
void LidarNode::preprocessData(sl_lidar_response_measurement_node_hq_t* raw_nodes, size_t count, std::vector<sl_lidar_response_measurement_node_hq_t>& valid_nodes) {
    for (size_t i = 0; i < count; ++i) {
        int quality = raw_nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        float angle_deg = (raw_nodes[i].angle_z_q14 * 90.f) / 16384.f;
        
        if (quality > 0 && (angle_deg <= 40.0f || angle_deg >= 320.0f)) {
            valid_nodes.push_back(raw_nodes[i]);
        }
    }
}

/**
 * 2. 배경 캘리브레이션: 정적 환경(벽, 기둥 등)의 맵을 생성하여 추후 동적 객체와 분리
 * @param duration_sec: 맵 생성 시간 (초)
 */
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
            
            for (const auto& p : points) {
                bool too_close = false;
                for (const auto& bg : background_points) {
                    if (std::hypot(p.x - bg.x, p.y - bg.y) < 50.0f) { 
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

/**
 * 3. 좌표 변환: 극좌표계(Lidar 원시 데이터)를 연산 가능한 직교좌표계(mm 단위)로 변환
 */
void LidarNode::transformCoordinates(const std::vector<sl_lidar_response_measurement_node_hq_t>& valid_nodes, std::vector<Point2D>& points) {
    for (const auto& node : valid_nodes) {
        float angle_deg = (node.angle_z_q14 * 90.f) / 16384.f;
        float dist_mm = node.dist_mm_q2 / 4.0f; // SDK 사양에 따른 mm 단위 변환
        float angle_rad = angle_deg * (M_PI / 180.0f);
        
        points.push_back({
            static_cast<float>(dist_mm * sin(angle_rad)), 
            static_cast<float>(dist_mm * cos(angle_rad))
        });
    }
}

/**
 * 4. 배경 제거 로직: 학습된 캘리브레이션 데이터와 비교하여 정적인 점들을 제외
 */
void LidarNode::filterBackground(const std::vector<Point2D>& input, std::vector<Point2D>& output) {
    if (background_points.empty()) {
        output = input; 
        return;
    }
    for (const auto& p : input) {
        bool is_static = false;
        for (const auto& bg : background_points) {
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

/**
 * 5. 클러스터링 로직: 인접한 점들 사이의 거리를 계산하여 동일 객체(어뢰 등)로 그룹화
 */
std::vector<std::vector<Point2D>> LidarNode::doClustering(const std::vector<Point2D>& points) {
    std::vector<std::vector<Point2D>> clusters;
    if (points.empty()) return clusters;

    float threshold = 300.0f; 
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

/**
 * 6. 최종 반환 함수: 고해상도 좌표를 5cm 단위 그리드로 변환 및 데이터 경량화
 */
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
        filterBackground(points, filtered_points); 
        
        std::vector<std::vector<Point2D>> clusters = doClustering(filtered_points);

        for (const auto& cluster : clusters) {
            std::vector<PointGrid> grid_cluster;
            for (const auto& p : cluster) {
                PointGrid pg = {
                    static_cast<int>(std::round(p.x / 50.0f)),
                    static_cast<int>(std::round(p.y / 50.0f))
                };
                
                if (grid_cluster.empty() || grid_cluster.back().x != pg.x || grid_cluster.back().y != pg.y) {
                    grid_cluster.push_back(pg);
                }
            }
            grid_clusters.push_back(grid_cluster);
        }
    }
    return grid_clusters;
}

/**
 * 동적 객체(어뢰/이동체) 위치 추적 메인 로직
 * 상기 정의된 1~5번의 처리 과정을 통합하여 실시간 좌표 산출
 */
bool LidarNode::getDynamicCarPosition(Point2D& car_pos) {
    if (!is_connected) return false;

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    if (SL_IS_OK(drv->grabScanDataHq(nodes, count))) {
        drv->ascendScanData(nodes, count); 
        
        std::vector<sl_lidar_response_measurement_node_hq_t> valid_nodes;
        std::vector<Point2D> points, filtered_points;

        preprocessData(nodes, count, valid_nodes); // 1. 전역 필터링
        transformCoordinates(valid_nodes, points); // 3. 좌표 변환
        filterBackground(points, filtered_points); // 4. 배경 제거
        
        std::vector<std::vector<Point2D>> clusters = doClustering(filtered_points); // 5. 클러스터링

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