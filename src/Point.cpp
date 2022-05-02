//
// Created by rbdstudent on 15/06/2021.
//

#include "../include/Point.h"


Point::Point() {
    this->x = 0;
    this->y = 0;
    this->z = 0;
    this->lidarId = -1;
    this->lidarOriginalPosition = 0;
    this->isAlgoFloor = false;
    this->isGroundTruthFloor = false;
}

Point::Point(const Point &point) {
    this->x = point.x;
    this->y = point.y;
    this->z = point.z;
    this->lidarId = point.lidarId;
    this->lidarOriginalPosition = point.lidarOriginalPosition;

    this->isAlgoFloor = point.isAlgoFloor;
    this->isGroundTruthFloor = point.isGroundTruthFloor;
}


Point::Point(double x, double y, double z, int lidarOriginalPosition, int lidarId, bool isAlgoFloor,
             bool isGroundTruthFloor) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->lidarId = lidarId;
    this->lidarOriginalPosition = lidarOriginalPosition;
    this->isAlgoFloor = isAlgoFloor;
    this->isGroundTruthFloor = isGroundTruthFloor;
}