#include "LidarNode.h"
#include <iostream>
#include <cmath>
#include <algorithm> 
#include <chrono>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace sl;

// ═══════════════════════════════════════════════
// 생성자: 라이다 드라이버 초기화 및 스캔 시작
// ═══════════════════════════════════════════════
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

// ═══════════════════════════════════════════════
// 소멸자: 모터 정지 및 리소스 해제
// ═══════════════════════════════════════════════
LidarNode::~LidarNode() {
    if (drv) {
        drv->stop();           
        drv->setMotorSpeed(0); 
        delete drv;
        drv = NULL;
    }
}

// ═══════════════════════════════════════════════
// 1단계: 데이터 전처리 (품질 필터링 + FOV 제한)
// ═══════════════════════════════════════════════
void LidarNode::preprocessData(
    sl_lidar_response_measurement_node_hq_t* raw_nodes, 
    size_t count, 
    std::vector<sl_lidar_response_measurement_node_hq_t>& valid_nodes) 
{
    for (size_t i = 0; i < count; ++i) {
        int quality = raw_nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        float angle_deg = (raw_nodes[i].angle_z_q14 * 90.f) / 16384.f;
        
        if (quality > 0 && (angle_deg <= 40.0f || angle_deg >= 320.0f)) {
            valid_nodes.push_back(raw_nodes[i]);
        }
    }
}

// ═══════════════════════════════════════════════
// 2단계: 배경 캘리브레이션 (정적 환경 학습)
// ═══════════════════════════════════════════════
void LidarNode::startCalibration(int duration_sec) {
    std::cout << "[Lidar] Background Calibration Started (" 
              << duration_sec << " sec)..." << std::endl;
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
    std::cout << "[Lidar] Calibration Done. Static points mapped: " 
              << background_points.size() << std::endl;
}

// ═══════════════════════════════════════════════
// 3단계: 좌표 변환 (극좌표 → 직교좌표)
// ═══════════════════════════════════════════════
void LidarNode::transformCoordinates(
    const std::vector<sl_lidar_response_measurement_node_hq_t>& valid_nodes, 
    std::vector<Point2D>& points) 
{
    for (const auto& node : valid_nodes) {
        float angle_deg = (node.angle_z_q14 * 90.f) / 16384.f;
        float dist_mm = node.dist_mm_q2 / 4.0f;  
        float angle_rad = angle_deg * (M_PI / 180.0f);
        
        points.push_back({
            static_cast<float>(dist_mm * sin(angle_rad)),  // X
            static_cast<float>(dist_mm * cos(angle_rad))   // Y
        });
    }
}

// ═══════════════════════════════════════════════
// 4단계: 배경 제거 (정적 맵 기반 필터링)
// ═══════════════════════════════════════════════
void LidarNode::filterBackground(
    const std::vector<Point2D>& input, 
    std::vector<Point2D>& output) 
{
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

// ═══════════════════════════════════════════════
// 5단계: 클러스터링 (인접 점 그룹화)
// ═══════════════════════════════════════════════
// ═══════════════════════════════════════════════
// 개선된 5단계: 클러스터링 (인접 점 그룹화 및 노이즈 필터링)
// ═══════════════════════════════════════════════
std::vector<std::vector<Point2D>> LidarNode::doClustering(const std::vector<Point2D>& points) {
    std::vector<std::vector<Point2D>> clusters;
    if (points.empty()) return clusters;

    // 🎯 [파라미터 튜닝] 단위가 mm라면 300~400 사이에서 물체 거리에 따라 조절 필요
    float threshold = 350.0f;  
    
    // 🎯 작은 물체 유실을 방지하기 위한 최소 점 개수 하한선 (기존에 상위 단에 있었다면 여기로 통합 권장)
    size_t min_cluster_size = 3; 
    size_t max_cluster_size = 100;

    std::vector<Point2D> current_cluster;
    current_cluster.push_back(points[0]);

    for (size_t i = 1; i < points.size(); ++i) {
        // 🔥 [개선] 직전 점(i-1) 뿐만 아니라, 현재 클러스터의 '마지막 원소'와 비교
        // 이렇게 하면 중간에 점이 하나 누락되어도 cluster의 형태를 유지하며 묶을 수 있음
        float dx = points[i].x - current_cluster.back().x;
        float dy = points[i].y - current_cluster.back().y;
        float dist = std::sqrt(dx*dx + dy*dy);

        if (dist < threshold) {
            current_cluster.push_back(points[i]);
        } else {
            // 조건을 만족하는 클러스터만 최종 리스트에 추가
            if (current_cluster.size() >= min_cluster_size && current_cluster.size() <= max_cluster_size) {
                clusters.push_back(current_cluster);
            }
            
            current_cluster.clear();
            current_cluster.push_back(points[i]);
        }
    }
    
    // 마지막 잔여 클러스터 처리
    if (current_cluster.size() >= min_cluster_size && current_cluster.size() <= max_cluster_size) {
        clusters.push_back(current_cluster);
    }
    
    return clusters;
}

// ═══════════════════════════════════════════════════════════════════════
// [신규 핵심 기능] 통합 데이터 처리 파이프라인 (GUI용과 제어용 분기)
// ═══════════════════════════════════════════════════════════════════════
bool LidarNode::processLidarFrame(Point2D& car_pos, std::vector<std::vector<PointGrid>>& grid_clusters) {
    if (!is_connected) return false;

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    
    // 데이터 획득은 무조건 한 루프에 딱 한 번만 수행
    if (SL_IS_OK(drv->grabScanDataHq(nodes, count))) {
        drv->ascendScanData(nodes, count);
        
        std::vector<sl_lidar_response_measurement_node_hq_t> valid_nodes;
        std::vector<Point2D> points; 

        // 1. 공통 전처리 및 직교 좌표 변환
        preprocessData(nodes, count, valid_nodes);      
        transformCoordinates(valid_nodes, points);      

        // ─────────────────────────────────────────────────────────
        // 갈래 1: GUI 전송 로직 (배경 제거 안 한 '전체 스캔 데이터' 사용)
        // ─────────────────────────────────────────────────────────
        std::vector<std::vector<Point2D>> gui_raw_clusters = doClustering(points); 
        
        grid_clusters.clear();
        for (const auto& cluster : gui_raw_clusters) {
            std::vector<PointGrid> grid_cluster;
            for (const auto& p : cluster) {
                PointGrid pg;
                pg.x = p.x / 1000.0f; // mm -> m
                pg.y = p.y / 1000.0f; // mm -> m
                pg.is_obstacle = true;
                
                if (grid_cluster.empty() || 
                    grid_cluster.back().x != pg.x || 
                    grid_cluster.back().y != pg.y) {
                    grid_cluster.push_back(pg);
                }
            }
            grid_clusters.push_back(grid_cluster);
        }

        // ─────────────────────────────────────────────────────────
        // 갈래 2: 제어 로직 (배경을 칼같이 제거한 '동적 어뢰'만 추적)
        // ─────────────────────────────────────────────────────────
        std::vector<Point2D> filtered_points;
        filterBackground(points, filtered_points); 
        
        std::vector<std::vector<Point2D>> control_clusters = doClustering(filtered_points);

        // 첫 번째 동적 객체의 중심점 계산
        if (!control_clusters.empty() && control_clusters[0].size() > 2) {
            float sum_x = 0, sum_y = 0;
            for (const auto& p : control_clusters[0]) {
                sum_x += p.x;
                sum_y += p.y;
            }
            car_pos.x = sum_x / control_clusters[0].size();
            car_pos.y = sum_y / control_clusters[0].size();
            return true; // 어뢰(동적 객체) 발견 성공
        }
    }
    return false; // 센서 에러 발생 혹은 동적 객체를 놓침
}