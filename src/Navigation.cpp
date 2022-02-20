//
// Created by tzuk on 02/01/2022.
//

#include "../include/Navigation.h"


std::vector<bool>
Navigation::objectDetection(std::vector<Point> &points, std::vector<Point> &track, Point &currentPosition) {
    std::string pangolinWindowName = "main";
    Auxiliary::SetupPangolin(pangolinWindowName);
    std::vector<bool> trackOptions;
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    for (int i = 0; i < track.size() - 1; ++i) {
        Point current = track[i];
        Point next = track[i + 1];
        std::vector<Point> pointsInTrack;
        double trackLength = Auxiliary::calculateDistance3D(current, next);
        for (const auto &point: points) {
            if (current.z < point.z || Auxiliary::calculateDistance3D(current, point) > trackLength) {
                continue;
            }
            pointsInTrack.emplace_back(point);
        }
        if (pointsInTrack.empty()) {
            std::cout << "no points in track" << std::endl;
            continue;
        }
        Auxiliary::DrawMapPointsPangolin(points, pointsInTrack, pangolinWindowName, track[1]);
        std::vector<Point> pointsInFieldOfView;

        double trackAngle = atan2(next.y - current.y, next.x - current.x);
        for (const auto &point: pointsInTrack) {
            double pointAngle = atan2(point.y - current.y, point.x - current.x);
            double angle = Auxiliary::radiansToAngle(pointAngle - trackAngle);
            if (angle < 15 && angle > -15) {
                pointsInFieldOfView.emplace_back(point);
            }
        }
        int sizeOfJump = 2;
        if (pointsInFieldOfView.size() < sizeOfJump) {
            std::cout << "no points in 30 angle around the track" << std::endl;
            continue;
        }
        std::cout << pointsInFieldOfView.size() << std::endl;
        std::sort(pointsInFieldOfView.begin(), pointsInFieldOfView.end(),
                  [](const Point &point1, const Point &point2) -> bool {
                      return std::abs(point1.y) > std::abs(point2.y);
                  });
        std::vector<double> z = Auxiliary::getZValues(pointsInFieldOfView);
        std::vector<double> y = Auxiliary::getYValues(pointsInFieldOfView);
        std::vector<double> x = Auxiliary::getXValues(pointsInFieldOfView);
        std::cout << pointsInFieldOfView.size() << std::endl;
        std::vector<double> pointsSizes;
        std::vector<double> variances;
        std::vector<std::pair<double, std::vector<Point>>> weightedPoints;
        while (y.size() > sizeOfJump) {
            double zVariance = Auxiliary::calculateVariance(z);
            double xVariance = Auxiliary::calculateVariance(x);
            double yVariance = Auxiliary::calculateVariance(y);
            variances.emplace_back(((xVariance + zVariance) / yVariance) * y.size());
            pointsSizes.emplace_back(z.size());
            weightedPoints.emplace_back(((xVariance + zVariance) / yVariance) * y.size(),
                                        std::vector<Point>(pointsInFieldOfView.begin(),
                                                           pointsInFieldOfView.begin() + y.size()));
            z.resize(z.size() - sizeOfJump);
            x.resize(x.size() - sizeOfJump);
            y.resize(y.size() - sizeOfJump);
        }
        std::sort(weightedPoints.begin(), weightedPoints.end(),
                  [](const std::pair<double, std::vector<Point>> &weightedPoint1,
                     const std::pair<double, std::vector<Point>> &weightedPoint2) -> bool {
                      return weightedPoint1.first > weightedPoint2.first;
                  });
        std::cout << weightedPoints.front().second.size() << std::endl;

        Auxiliary::DrawMapPointsPangolin(points, weightedPoints.front().second, pangolinWindowName, track[1]);
        Auxiliary::showGraph(pointsSizes, variances, "ro");

    }
    return {};
}
std::vector<Point>
Navigation::getFloorFromOrbSlam(std::vector<Point> &points, unsigned long sizeOfJump, bool isDebug, std::string pangolinPostfix) {
    std::sort(points.begin(), points.end(), [](const Point &point1, const Point &point2) -> bool {
        return point1.z > point2.z;
    });

    //Auxiliary::exportToXYZFile(points,
    //                         "/home/tzuk/Documents/AutonomousDroneResults/varianceFilter/cloud.xyz");
    std::vector<double> z = Auxiliary::getZValues(points);
    std::vector<double> y = Auxiliary::getYValues(points);
    std::vector<double> x = Auxiliary::getXValues(points);
    std::vector<double> pointsSizes;
    std::vector<double> variances;
    std::vector<std::pair<double, std::vector<Point>>> weightedPoints;
    while (z.size() > sizeOfJump) {
        double zVariance = Auxiliary::calculateVariance(z);
        double xVariance = Auxiliary::calculateVariance(x);
        double yVariance = Auxiliary::calculateVariance(y);
        variances.emplace_back(((xVariance + yVariance) / zVariance) * z.size());
        pointsSizes.emplace_back(z.size());
        weightedPoints.emplace_back(((xVariance + yVariance) / zVariance) * z.size(),
                                    std::vector<Point>(points.begin(), points.begin() + z.size()));
        z.resize(z.size() - sizeOfJump);
        x.resize(x.size() - sizeOfJump);
        y.resize(y.size() - sizeOfJump);
    }
    std::sort(weightedPoints.begin(), weightedPoints.end(),
              [](const std::pair<double, std::vector<Point>> &weightedPoint1,
                 const std::pair<double, std::vector<Point>> &weightedPoint2) -> bool {
                  return weightedPoint1.first > weightedPoint2.first;
              });
    /*Auxiliary::exportToXYZFile(weightedPoints.front().second,
                               "/home/tzuk/Documents/AutonomousDroneResults/varianceFilter/floor.xyz");
    Auxiliary::showGraph(pointsSizes, variances, "ro");*/
    if (isDebug) {
        Auxiliary::showGraph(pointsSizes, variances, "ro");
        Auxiliary::SetupPangolin("floor" + pangolinPostfix);
        Auxiliary::DrawMapPointsPangolin(points, weightedPoints.front().second, "floor" + pangolinPostfix);
    }
    return weightedPoints.front().second;
}
std::vector<Point>
Navigation::getFloorFromLidar(std::vector<Point> &points, unsigned long sizeOfJump, bool isDebug, std::string pangolinPostfix) {
    std::sort(points.begin(), points.end(), [](const Point &point1, const Point &point2) -> bool {
        return point1.z < point2.z;
    });

    //Auxiliary::exportToXYZFile(points,
    //                         "/home/tzuk/Documents/AutonomousDroneResults/varianceFilter/cloud.xyz");
    std::vector<double> z = Auxiliary::getZValues(points);
    std::vector<double> y = Auxiliary::getYValues(points);
    std::vector<double> x = Auxiliary::getXValues(points);
    std::vector<double> pointsSizes;
    std::vector<double> variances;
    std::vector<std::pair<double, std::vector<Point>>> weightedPoints;
    while (z.size() > sizeOfJump) {
        double zVariance = Auxiliary::calculateVariance(z);
        double xVariance = Auxiliary::calculateVariance(x);
        double yVariance = Auxiliary::calculateVariance(y);
        variances.emplace_back(((xVariance + yVariance) / zVariance) * z.size());
        pointsSizes.emplace_back(z.size());
        weightedPoints.emplace_back(((xVariance + yVariance) / zVariance) * z.size(),
                                    std::vector<Point>(points.begin(), points.begin() + z.size()));
        z.resize(z.size() - sizeOfJump);
        x.resize(x.size() - sizeOfJump);
        y.resize(y.size() - sizeOfJump);
    }
    std::sort(weightedPoints.begin(), weightedPoints.end(),
              [](const std::pair<double, std::vector<Point>> &weightedPoint1,
                 const std::pair<double, std::vector<Point>> &weightedPoint2) -> bool {
                  return weightedPoint1.first > weightedPoint2.first;
              });
    /*Auxiliary::exportToXYZFile(weightedPoints.front().second,
                               "/home/tzuk/Documents/AutonomousDroneResults/varianceFilter/floor.xyz");
    Auxiliary::showGraph(pointsSizes, variances, "ro");*/
    if (isDebug) {
        Auxiliary::showGraph(pointsSizes, variances, "ro");
        Auxiliary::SetupPangolin("floor" + pangolinPostfix);
        Auxiliary::DrawMapPointsPangolin(points, weightedPoints.front().second, "floor" + pangolinPostfix);
    }
    return weightedPoints.front().second;
}

//ccl logic

/*double minDistance = Auxiliary::GetMinDistance(pointsInFieldOfView, Auxiliary::calculateDistanceXZ);
auto x = Auxiliary::getXValues(pointsInFieldOfView);
auto[xMax, xMin] = Auxiliary::GetMinMax(x);
auto z = Auxiliary::getZValues(pointsInFieldOfView);
auto[zMax, zMin] = Auxiliary::GetMinMax(z);
int rows = std::ceil(std::abs((xMax - xMin) / minDistance));
int cols = std::ceil(std::abs((zMax - zMin) / minDistance));
std::cout << "rows: " << rows << " cols:" << cols << std::endl;
cv::Mat pointsMat(rows, cols, CV_8U);
std::vector<std::unordered_map<int, std::unordered_map<int, std::vector<Point>>>> results;
std::unordered_map<int, std::unordered_map<int, std::vector<Point>>> pointsForDisplay;
for (const auto &point: pointsInFieldOfView) {
    int row = std::floor((point.x - xMin) / minDistance);
    int col = std::floor((point.z - zMin) / minDistance);
    pointsMat.at<int>(row, col) = 1;
    if (!pointsForDisplay.count(row)) {
        pointsForDisplay.insert({row, std::unordered_map<int, std::vector<Point>>{}});
    } else {
        if (!pointsForDisplay.count(col)) {
            pointsForDisplay.at(row).insert({col, std::vector<Point>{}});
        }
        pointsForDisplay.at(row).at(col).emplace_back(point);
    }
}
results.emplace_back(pointsForDisplay);
int numberOfLabels = cv::connectedComponentsWithStats(pointsMat, labels, stats, centroids, 8, CV_32S);
std::cout << numberOfLabels << std::endl;
std::vector<Point> labeledPoints;
for (int row = 0; row < labels.rows; row++) {
    for (int col = 0; col < labels.cols; col++) {
        if (labels.at<int>(row, col)) {
            labeledPoints.insert(labeledPoints.end(), pointsForDisplay.at(row).at(col).begin(),
                                 pointsForDisplay.at(row).at(col).end());
        }
    }
}
Auxiliary::DrawMapPointsPangolin(points, labeledPoints, pangolinWindowName, track[1]);*/