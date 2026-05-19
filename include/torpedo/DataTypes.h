#ifndef DATATYPES_H
#define DATATYPES_H

struct Point2D {
    float x; // mm 단위
    float y; // mm 단위
};

struct PointGrid {
    int x; // 50mm 단위
    int y; 
};

// 어뢰의 현재 상태를 담는 구조체
struct TorpedoPose {
    Point2D position;
    float speed;
    float heading;
    float acc_x;
    float acc_y;
};

#endif