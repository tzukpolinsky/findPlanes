//
// Created by tzuk on 02/01/2022.
//

#ifndef ORB_SLAM2_NAVIGATION_H
#define ORB_SLAM2_NAVIGATION_H

#include "include/Point.h"
#include <vector>
#include "include/Auxiliary.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
class Navigation {
public:

    std::vector<Point> getFloor(std::vector<Point> &points, unsigned long sizeOfJump,bool isDebug = false,std::string &pangolinPostfix = (std::string &) "");

    std::vector<bool>
    objectDetection(std::vector<Point> &points, std::vector<Point> &track, Point &currentPosition);

};


#endif //ORB_SLAM2_NAVIGATION_H
