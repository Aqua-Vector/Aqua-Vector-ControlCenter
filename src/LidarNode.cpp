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
    
    // SLAMTEC SDK 드라이버 생성
    drv = *createLidarDriver();
    if (!drv) {
        fprintf(stderr, "  [Lidar Error] Insufficient memory\n");
        fflush(stderr);
        return;
    }
    
    fprintf(stderr, "  [Lidar] Opening port %s at %u baud...\n", port, baudrate);
    fflush(stderr);

    // 시리얼 포트 연결
    IChannel* _channel = *createSerialPortChannel(port, baudrate);
    if (SL_IS_OK((drv)->connect(_channel))) {
        fprintf(stderr, "  [Lidar] Connected, starting motor...\n");
        fflush(stderr);
        
        drv->setMotorSpeed();  // 모터 회전 시작 (기본 속도)
        drv->startScan(0, 1);  // 스캔 시작 (일반 모드, 강제 시작)
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
        drv->stop();           // 스캔 중지
        drv->setMotorSpeed(0); // 모터 정지
        delete drv;
        drv = NULL;
    }
}

// ═══════════════════════════════════════════════
// 1단계: 데이터 전처리
// - 품질 필터링: quality > 0인 데이터만 선택
// - FOV 제한: 전방 ±40도만 사용 (0~40도, 320~360도)
// ═══════════════════════════════════════════════
void LidarNode::preprocessData(
    sl_lidar_response_measurement_node_hq_t* raw_nodes, 
    size_t count, 
    std::vector<sl_lidar_response_measurement_node_hq_t>& valid_nodes) 
{
    for (size_t i = 0; i < count; ++i) {
        // 품질 추출 (상위 6비트)
        int quality = raw_nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        
        // 각도 변환 (Q14 고정소수점 → degree)
        float angle_deg = (raw_nodes[i].angle_z_q14 * 90.f) / 16384.f;
        
        // 품질 및 FOV 필터링
        if (quality > 0 && (angle_deg <= 40.0f || angle_deg >= 320.0f)) {
            valid_nodes.push_back(raw_nodes[i]);
        }
    }
}

// ═══════════════════════════════════════════════
// 2단계: 배경 캘리브레이션
// 시스템 부팅 시 정적 환경(벽, 기둥)을 학습
// @param duration_sec: 캘리브레이션 시간 (초)
// ═══════════════════════════════════════════════
void LidarNode::startCalibration(int duration_sec) {
    std::cout << "[Lidar] Background Calibration Started (" 
              << duration_sec << " sec)..." << std::endl;
    background_points.clear();
    
    auto start_time = std::chrono::steady_clock::now();
    
    // 지정 시간 동안 반복 스캔
    while (std::chrono::steady_clock::now() - start_time < 
           std::chrono::seconds(duration_sec)) {
        
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);
        
        if (SL_IS_OK(drv->grabScanDataHq(nodes, count))) {
            drv->ascendScanData(nodes, count);  // 각도 순 정렬
            
            std::vector<sl_lidar_response_measurement_node_hq_t> valid_nodes;
            std::vector<Point2D> points;
            
            preprocessData(nodes, count, valid_nodes);
            transformCoordinates(valid_nodes, points);
            
            // 중복 제거 (50mm 이내 점은 하나로 병합)
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
// SDK 원시 데이터(각도, 거리) → X,Y 좌표 (mm)
// ═══════════════════════════════════════════════
void LidarNode::transformCoordinates(
    const std::vector<sl_lidar_response_measurement_node_hq_t>& valid_nodes, 
    std::vector<Point2D>& points) 
{
    for (const auto& node : valid_nodes) {
        // 각도 및 거리 변환
        float angle_deg = (node.angle_z_q14 * 90.f) / 16384.f;
        float dist_mm = node.dist_mm_q2 / 4.0f;  // Q2 고정소수점 → mm
        float angle_rad = angle_deg * (M_PI / 180.0f);
        
        // 극좌표 → 직교좌표 (라이다 중심 기준)
        points.push_back({
            static_cast<float>(dist_mm * sin(angle_rad)),  // X
            static_cast<float>(dist_mm * cos(angle_rad))   // Y
        });
    }
}

// ═══════════════════════════════════════════════
// 4단계: 배경 제거
// 캘리브레이션 데이터와 비교하여 정적 물체 필터링
// @param threshold: 300mm - 이 거리 이내면 정적 물체로 판단
// ═══════════════════════════════════════════════
void LidarNode::filterBackground(
    const std::vector<Point2D>& input, 
    std::vector<Point2D>& output) 
{
    // 캘리브레이션 안 했으면 전체 반환
    if (background_points.empty()) {
        output = input; 
        return;
    }
    
    for (const auto& p : input) {
        bool is_static = false;
        
        // 배경 포인트와 거리 비교
        for (const auto& bg : background_points) {
            if (std::hypot(p.x - bg.x, p.y - bg.y) < 300.0f) {
                is_static = true;
                break;
            }
        }
        
        // 동적 객체만 추가
        if (!is_static) {
            output.push_back(p);
        }
    }
}

// ═══════════════════════════════════════════════
// 5단계: 클러스터링
// 인접한 점들을 그룹화하여 개별 객체 식별
// @param threshold: 300mm - 이 거리 이내면 같은 객체
// ═══════════════════════════════════════════════
std::vector<std::vector<Point2D>> LidarNode::doClustering(
    const std::vector<Point2D>& points) 
{
    std::vector<std::vector<Point2D>> clusters;
    if (points.empty()) return clusters;

    float threshold = 300.0f;  // 클러스터 병합 임계값
    std::vector<Point2D> current_cluster;
    current_cluster.push_back(points[0]);

    // 순차적으로 거리 비교
    for (size_t i = 1; i < points.size(); ++i) {
        float dx = points[i].x - points[i-1].x;
        float dy = points[i].y - points[i-1].y;
        float dist = std::sqrt(dx*dx + dy*dy);

        if (dist < threshold) {
            // 같은 클러스터에 추가
            current_cluster.push_back(points[i]);
        } else {
            // 새 클러스터 시작
            clusters.push_back(current_cluster);
            current_cluster.clear();
            current_cluster.push_back(points[i]);
        }
    }
    
    // 마지막 클러스터 추가
    if (!current_cluster.empty()) {
        clusters.push_back(current_cluster);
    }
    
    return clusters;
}

// ═══════════════════════════════════════════════
// 6단계: 그리드 장애물 반환 (GUI 전송용)
// 고해상도 Point2D → 50mm 그리드 PointGrid 변환
// 네트워크 전송량 감소 및 GUI 렌더링 최적화
// ═══════════════════════════════════════════════
std::vector<std::vector<PointGrid>> LidarNode::getGridObstacles() {
    std::vector<std::vector<PointGrid>> grid_clusters;
    if (!is_connected) return grid_clusters;

    // 라이다 데이터 획득
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    sl_result op_result = drv->grabScanDataHq(nodes, count);

    if (SL_IS_OK(op_result)) {
        drv->ascendScanData(nodes, count);  // 각도 순 정렬
        
        // 처리 파이프라인 실행
        std::vector<sl_lidar_response_measurement_node_hq_t> valid_nodes;
        std::vector<Point2D> points;
        std::vector<Point2D> filtered_points;

        preprocessData(nodes, count, valid_nodes);         // 1. 전처리
        transformCoordinates(valid_nodes, points);         // 2. 좌표 변환
        filterBackground(points, filtered_points);         // 3. 배경 제거
        
        std::vector<std::vector<Point2D>> clusters = doClustering(filtered_points); // 4. 클러스터링

        // Point2D → PointGrid 변환 (50mm 단위로 양자화)
        for (const auto& cluster : clusters) {
            std::vector<PointGrid> grid_cluster;
            for (const auto& p : cluster) {
                PointGrid pg;
                pg.x = p.x / 1000.0f;   // mm → m
                pg.y = p.y / 1000.0f;   // mm → m
                pg.is_obstacle = true;
                
                // 중복 제거 (연속된 동일 그리드)
                if (grid_cluster.empty() || 
                    grid_cluster.back().x != pg.x || 
                    grid_cluster.back().y != pg.y) {
                    grid_cluster.push_back(pg);
                }
            }
            grid_clusters.push_back(grid_cluster);
        }
    }
    return grid_clusters;
}

// ═══════════════════════════════════════════════
// 동적 객체(어뢰) 위치 추적
// 전체 파이프라인 실행 후 첫 번째 클러스터의 중심 반환
// @param car_pos: 어뢰 위치 출력 (mm)
// @return: true=객체 발견, false=없음
// ═══════════════════════════════════════════════
bool LidarNode::getDynamicCarPosition(Point2D& car_pos) {
    if (!is_connected) return false;

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    
    if (SL_IS_OK(drv->grabScanDataHq(nodes, count))) {
        drv->ascendScanData(nodes, count);
        
        std::vector<sl_lidar_response_measurement_node_hq_t> valid_nodes;
        std::vector<Point2D> points, filtered_points;

        preprocessData(nodes, count, valid_nodes);      // 1. 전처리
        transformCoordinates(valid_nodes, points);      // 2. 좌표 변환
        filterBackground(points, filtered_points);      // 3. 배경 제거
        
        std::vector<std::vector<Point2D>> clusters = doClustering(filtered_points); // 4. 클러스터링

        // 첫 번째 클러스터의 중심점 계산
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