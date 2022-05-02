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
#include <thread>

class Navigation {
public:

    std::vector<Point> getFloorFromLidar(std::vector<Point> &points, unsigned long sizeOfJump, bool isDebug = false,
                                         std::string pangolinPostfix = "");

    void getFloorByCovariance(std::vector<Point> &points, unsigned long sizeOfJump,std::pair<long, std::vector<Point>> &scoreAndFloor,
                                                             bool isDebug = false, std::string pangolinPostfix = "");

    std::vector<Point>
    alignByFloor(std::vector<Point> points, std::vector<Point> floor, int heightDirection);

    std::vector<Point>
    getFloorFromOrbSlam(std::vector<Point> &points, unsigned long sizeOfJump, bool isDebug = false,
                        std::string pangolinPostfix = "");

    std::vector<bool>
    objectDetection(std::vector<Point> &points, std::vector<Point> &track, Point &currentPosition);

    void alignByAngle(std::vector<Point> &points, double roll, double pitch,std::vector<Point> &rotatedPoints);

    std::vector<Point>
    getFloorAndAlign(std::vector<Point> &points, unsigned long sizeOfJump, bool isDebug, std::string pangolinPostfix);

    std::vector<Point>
    getFloorAndBruteForceAlignUsingThreads(std::vector<Point> &points, unsigned long sizeOfJump, bool isDebug,
                                           std::string pangolinPostfix);

    std::vector<Point> getFloorAndBruteForceAlign(std::vector<Point> &points, unsigned long sizeOfJump, bool isDebug,
                                                  std::string pangolinPostfix);

    int bestPitch = std::numeric_limits<int>::min();
    int bestRoll = std::numeric_limits<int>::min();

    void alignByAngleThread(std::vector<Point> &points, int roll, int pitch, int sizeOfJump,
                            std::pair<long, std::vector<Point>> &result, std::vector<Point> &rotatedPoints);
};


#endif //ORB_SLAM2_NAVIGATION_H
