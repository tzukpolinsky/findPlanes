//
// Created by rbdstudent on 17/06/2021.
//

#ifndef ORB_SLAM2_AUXILIARY_H
#define ORB_SLAM2_AUXILIARY_H

#include "Point.h"
#include "Line.h"
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Eigen>
//#ifdef NOTRPI
#include <matplotlibcpp.h>
#include <limits>
#include <pangolin/pangolin.h>

#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>
#include <opencv2/core/mat.hpp>

//#endif
class Auxiliary {
public:
    static Point GetCenterOfMass(const std::vector<Point> &points);

    static std::string GetDataSetsDirPath();

    static double angleToRadians(int angle);

    static double det(const Point &point1, const Point &point2);

    static double distanceBetweenPointAndSegment(const Point &point, Line segment);

    static long myGcd(long a, long b);

    static double radiansToAngle(double radian);

    static double calculateDistanceXY(const Point &point1, const Point &point2);

    static double calculateDistance3D(const Point &point1, const Point &point2);

    static double calculateVariance(const std::vector<double> &distances);

    static std::vector<double> getXValues(const std::vector<Point> &points);

    static std::vector<double> getYValues(const std::vector<Point> &points);


    static std::vector<double> getZValues(const std::vector<Point> &points);
    static void displayLidarOfImage(cv::Mat &img);

    static void showGraph(std::vector<double> &x, std::vector<double> &y, const std::string &pointsDisplay = "");

    static double
    GetMinDistance(const std::vector<Point> &points, const std::function<double(Point, Point)> &DistanceFunc);

    static std::pair<double, double> GetMinMax(std::vector<double> &points);

    static void DrawMapPointsPangolin(const std::vector<Point> &cloud, const std::vector<Point> &redPoints,
                                      const std::string &windowName, const Point &lineFromCenter = Point());

    static void SetupPangolin(const std::string &window_name);

    static cv::Mat getCovarianceMat(std::vector<double> &x, std::vector<double> &y);
};


#endif //ORB_SLAM2_AUXILIARY_H
