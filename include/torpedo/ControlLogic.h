#ifndef CONTROL_LOGIC_H
#define CONTROL_LOGIC_H

#include "DataTypes.h"
#include <vector>

class ControlLogic {
private:
    std::vector<Point2D> static_map;
public:
    ControlLogic();
    void setStaticMap(const std::vector<Point2D>& map);
    int calculateSteering(const TorpedoPose& current, const Point2D& target);
};
#endif