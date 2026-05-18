#ifndef LIDAR_NODE_H
#define LIDAR_NODE_H

#include <vector>
#include <chrono>
#include "DataTypes.h"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

class LidarNode {
private:
    sl::ILidarDriver* drv;
    bool is_connected;
    std::vector<Point2D> background_points;

    // 내부 처리 함수들
    void preprocessData(sl_lidar_response_measurement_node_hq_t* raw_nodes, size_t count, std::vector<sl_lidar_response_measurement_node_hq_t>& valid_nodes);
    void transformCoordinates(const std::vector<sl_lidar_response_measurement_node_hq_t>& valid_nodes, std::vector<Point2D>& points);
    void filterBackground(const std::vector<Point2D>& input, std::vector<Point2D>& output);
    std::vector<std::vector<Point2D>> doClustering(const std::vector<Point2D>& points);

public:
    LidarNode(const char* port, sl_u32 baudrate);
    ~LidarNode();
    
    void startCalibration(int duration_sec);
    std::vector<std::vector<PointGrid>> getGridObstacles();
    bool getDynamicCarPosition(Point2D& pos);
};

#endif