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

//#endif
class Auxiliary {
public:
    static Point GetCenterOfMass(const std::vector<Point> &points);

    static std::string GetDataSetsDirPath();


    static double det(const Point &point1, const Point &point2);

    static double distanceBetweenPointAndSegment(const Point &point, Line segment);

    static double angleToRadians(int angle);

    static long myGcd(long a, long b);

    static std::tuple<int, int, int> getRationalInverse(double input);

    static double getAngleFromSlope(double slope);

    static double radiansToAngle(double radian);

    static double calculateDistanceXY(const Point &point1, const Point &point2);

    static double calculateDistance3D(const Point &point1, const Point &point2);

    static double getDistanceToClosestSegment(const Point &point, const std::vector<Line> &segments);

    static double getAngleBySlopes(Line line1, Line line2);

    static double calculateVariance(const std::vector<double> &distances);

    static double calculateMeanOfDistanceDifferences(std::vector<double> distances);

    static std::pair<int, bool> getRotationToTargetInFront(const Point &point1, const Point &point2);

    static std::pair<int, bool>
    getRotationToTargetInFront(const Point &previous, const Point &current, const Point &destination, bool isMinusUp);

    static std::vector<double> getXValues(const std::vector<Point> &points);

    static std::vector<double> getYValues(const std::vector<Point> &points);

//#ifdef NOTRPI
    static void showCloudPoint(const std::vector<Point> &redPoints, const std::vector<Point> &cloud);
//#endif

    double calculateMedianError(const std::vector<double> &errorVector, int numberOfSamples);

    static std::vector<double> Get3dAnglesBetween2Points(const Point &point1, const Point &point2);

    static double GetPitchFrom2Points(const Point &point1, const Point &point2);

    static double calculateDistanceXZ(const Point &point1, const Point &point2);

    static void showCloudPoint3D(const std::vector<Point> &redPoints, const std::vector<Point> &cloud);

    static std::vector<double> getZValues(const std::vector<Point> &points);

    static void exportToXYZFile(const std::vector<Point> &points, std::string fileName = "/tmp/result.xyz");

    static void showGraph(std::vector<double> &x, std::vector<double> &y, const std::string &pointsDisplay = "");

    static double
    GetMinDistance(const std::vector<Point> &points, const std::function<double(Point, Point)> &DistanceFunc);

    static std::pair<double, double> GetMinMax(std::vector<double> &points);

    static void DrawMapPointsPangolin(const std::vector<Point> &cloud, const std::vector<Point> &redPoints,
                                      const std::string &windowName, const Point &lineFromCenter = Point());

    static void SetupPangolin(const std::string &window_name);
};


#endif //ORB_SLAM2_AUXILIARY_H
