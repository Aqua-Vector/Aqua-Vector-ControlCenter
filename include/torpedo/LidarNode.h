#ifndef LIDAR_NODE_H
#define LIDAR_NODE_H

#include <vector>
#include <chrono>
#include "DataTypes.h"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

// 라이다 센서 관리 클래스
// SLAMTEC RPLidar 제어 및 데이터 처리
class LidarNode {
private:
    sl::ILidarDriver* drv;                     // 라이다 드라이버 포인터
    bool is_connected;                         // 연결 상태 플래그
    std::vector<Point2D> background_points;    // 배경 캘리브레이션 데이터

    // ─────────────────────────────────────────
    // 내부 데이터 처리 파이프라인 (Private 멤버 함수)
    // ─────────────────────────────────────────
    
    // 1단계: 원시 데이터 전처리 (품질 필터링 + FOV 제한)
    void preprocessData(
        sl_lidar_response_measurement_node_hq_t* raw_nodes, 
        size_t count, 
        std::vector<sl_lidar_response_measurement_node_hq_t>& valid_nodes
    );
    
    // 2단계: 좌표 변환 (극좌표 → 직교좌표)
    void transformCoordinates(
        const std::vector<sl_lidar_response_measurement_node_hq_t>& valid_nodes, 
        std::vector<Point2D>& points
    );
    
    // 3단계: 배경 제거 (정적 물체 필터링)
    void filterBackground(
        const std::vector<Point2D>& input, 
        std::vector<Point2D>& output
    );
    
    // 4단계: 클러스터링 (인접 점 그룹화 → 객체 식별)
    std::vector<std::vector<Point2D>> doClustering(
        const std::vector<Point2D>& points
    );

public:
    // 생성자: 시리얼 포트 연결 및 스캔 시작
    LidarNode(const char* port, sl_u32 baudrate);
    
    // 소멸자: 모터 정지 및 리소스 해제
    ~LidarNode();
    
    // 배경 캘리브레이션 (시스템 부팅 시 1회 실행)
    // @param duration_sec: 캘리브레이션 시간 (초)
    void startCalibration(int duration_sec);
    
    // 💡 [개선 완료] 통합 데이터 처리 파이프라인 함수
    // 단 한 번의 스캔(Grab)으로 GUI 격자 맵과 제어용 어뢰 위치를 동시에 연산 및 분기 출력
    // @param car_pos: [Output] 배경이 제거된 어뢰(동적 객체)의 중심 위치 (mm)
    // @param grid_clusters: [Output] GUI 전송용 전체 환경 격자 지도 데이터 (m)
    // @return: true = 어뢰 추적 성공, false = 라이다 데이터 획득 실패 혹은 어뢰 놓침
    bool processLidarFrame(Point2D& car_pos, std::vector<std::vector<PointGrid>>& grid_clusters);
    
    // 배경 포인트 반환 (디버깅/시각화용)
    std::vector<Point2D> getBackgroundPoints() { return background_points; }
};

#endif