//
// Created by rbdstudent on 15/06/2021.
//

#include "../include/Point.h"


Point::Point() {
    this->x = 0;
    this->y = 0;
    this->z = 0;
    this->lidarId = -1;
}

Point::Point(const Point &point) {
    this->x = point.x;
    this->y = point.y;
    this->z = point.z;
    this->lidarId = point.lidarId;
}

Point::Point(double x, double y, double z, int lidarId) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->lidarId = lidarId;
}
