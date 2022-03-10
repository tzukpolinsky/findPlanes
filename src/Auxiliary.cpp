//
// Created by rbdstudent on 17/06/2021.
//

#include <fstream>
#include <opencv2/core/mat.hpp>
#include "../include/Auxiliary.h"

Point Auxiliary::GetCenterOfMass(const std::vector<Point> &points) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    for (const Point &point: points) {
        x += point.x;
        y += point.y;
        z += point.z;
    }
    int size = points.size();
    return {x / size, y / size, z / size, -1};
}

double Auxiliary::det(const Point &point1, const Point &point2) {
    return point1.x * point2.y - point1.y * point2.x;
}

double Auxiliary::angleToRadians(int angle) {
    return (angle * M_PI) / 180;
}

double Auxiliary::radiansToAngle(double radian) {
    return radian * (180 / M_PI);
}

std::vector<double> Auxiliary::getXValues(const std::vector<Point> &points) {
    std::vector<double> xValues;
    for (const Point &point: points) {
        xValues.push_back(point.x);
    }
    return xValues;
}

std::vector<double> Auxiliary::getYValues(const std::vector<Point> &points) {
    std::vector<double> yValues;
    for (const Point &point: points) {
        yValues.push_back(point.y);
    }
    return yValues;
}

std::vector<double> Auxiliary::getZValues(const std::vector<Point> &points) {
    std::vector<double> yValues;
    for (const Point &point: points) {
        yValues.push_back(point.z);
    }
    return yValues;
}

//#ifdef NOTRPI
void Auxiliary::showCloudPoint(const std::vector<Point> &redPoints, const std::vector<Point> &cloud) {
    matplotlibcpp::clf();
    matplotlibcpp::scatter(getXValues(cloud), getYValues(cloud), 2.0);
    matplotlibcpp::plot(getXValues(redPoints), getYValues(redPoints), "ro");
    matplotlibcpp::show();
}

void Auxiliary::exportToXYZFile(const std::vector<Point> &points, std::string fileName) {
    std::ofstream pointData;
    pointData.open(fileName);
    for (const auto &point: points) {
        pointData << point.x << " " << point.y << " " << point.z << std::endl;
    }
    pointData.close();
}

void Auxiliary::SetupPangolin(const std::string &window_name) {
    // create a window and bind its context to the main thread
    pangolin::CreateWindowAndBind(window_name, 640, 480);

    // enable depth
    glEnable(GL_DEPTH_TEST);

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();
}

void Auxiliary::DrawMapPointsPangolin(const std::vector<Point> &cloud, const std::vector<Point> &redPoints,
                                      const std::string &windowName,
                                      const Point &lineFromCenter) {
    pangolin::BindToContext(windowName);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
            pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisZ)
    );

    pangolin::Renderable tree;
    tree.Add(std::make_shared<pangolin::Axis>());

    // Create Interactive View in window
    pangolin::SceneHandler handler(tree, s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
            .SetHandler(&handler);

    d_cam.SetDrawFunction([&](pangolin::View &view) {
        view.Activate(s_cam);
        tree.Render();
    });

    while (!pangolin::ShouldQuit()) {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        for (const auto &point: cloud) {
            glPointSize(1);
            glBegin(GL_POINTS);
            switch (point.lidarId) {
                case 0:
                    glColor3f(1.0, 0.0, 1.0);
                    break;
                case 1:

                    glColor3f(0.0, 0.0, 1.0);
                    break;
                case 2:

                    glColor3f(0.0, 1.0, 0.0);
                    break;
                case 3:

                    glColor3f(1.0, 1.0, 1.0);
                    break;
                case 4:

                    glColor3f(0.0, 1.0, 1.0);
                    break;
            }
            glVertex3f(point.x, point.y, point.z);


            glEnd();
        }
        glPointSize(10);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);
        for (const auto &point: redPoints) {
            glVertex3f(point.x, point.y, point.z);
        }
        glEnd();
        //pangolin::process::Mouse()
        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
    pangolin::GetBoundWindow()->RemoveCurrent();
    pangolin::DestroyWindow(windowName);
    return;
}

void Auxiliary::showGraph(std::vector<double> &x, std::vector<double> &y, const std::string &pointsDisplay) {
    matplotlibcpp::plot(x, y, pointsDisplay);
    matplotlibcpp::show();
}

void Auxiliary::showCloudPoint3D(const std::vector<Point> &redPoints, const std::vector<Point> &cloud) {
    //matplotlibcpp::axis("3d");
    matplotlibcpp::scatter(getXValues(cloud), getYValues(cloud), getZValues(cloud), 0.1);
    matplotlibcpp::show();
}

//#endif
double Auxiliary::distanceBetweenPointAndSegment(const Point &point, Line segment) {
    auto point1 = segment.getPoint1();
    auto point2 = segment.getPoint2();
    Point segmentDifference(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z);
    double dot = (point.x - point1.x) * segmentDifference.x + (point.y - point1.y) * segmentDifference.y;
    double segmentLength = pow(segmentDifference.x, 2) + pow(segmentDifference.y, 2);
    double param = -1;
    Point distancePoint(0, 0, 0);
    if (segmentLength != 0) {
        param = dot / segmentLength;
    }
    if (param < 0) {
        distancePoint.x = point1.x;
        distancePoint.y = point1.y;
    } else if (param > 1) {
        distancePoint.x = point2.x;
        distancePoint.y = point2.y;
    } else {
        distancePoint.x = point1.x + param * segmentDifference.x;
        distancePoint.y = point1.y + param * segmentDifference.y;
    }
    return calculateDistanceXY(Point(point.x - distancePoint.x, point.y - distancePoint.y, 0), Point(0, 0, 0));
}

double Auxiliary::getDistanceToClosestSegment(const Point &point, const std::vector<Line> &segments) {
    double minDistance = 10000;
    for (const auto &segment: segments) {
        double distance = distanceBetweenPointAndSegment(point, segment);
        minDistance = minDistance > distance ? distance : minDistance;
    }
    return minDistance;
}

double Auxiliary::getAngleFromSlope(double slope) {
    return radiansToAngle(atan(slope));
}

double Auxiliary::calculateDistance3D(const Point &point1, const Point &point2) {
    return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2) + pow(point2.z - point1.z, 2));
}

double Auxiliary::calculateDistanceXY(const Point &point1, const Point &point2) {
    return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2));
}

double Auxiliary::calculateDistanceXZ(const Point &point1, const Point &point2) {
    return sqrt(pow(point2.x - point1.x, 2) + pow(point2.z - point1.z, 2));
}

std::pair<int, bool>
Auxiliary::getRotationToTargetInFront(const Point &previous, const Point &current, const Point &destination,
                                      bool isMinusUp) {
    double PreviousToCurrent = atan2(previous.y - current.y, previous.x - current.x);
    double CurrentToDistance = atan2(destination.y - current.y, destination.x - current.x);
    double angle = Auxiliary::radiansToAngle(CurrentToDistance - PreviousToCurrent);
    std::cout << "angle:" << angle << std::endl;
    bool clockwise;
    if (angle < 0) {
        if (angle >= -180) {
            angle *= -1;
            clockwise = isMinusUp;
        } else {
            angle += 360;
            clockwise = !isMinusUp;
        }
    } else {
        if (angle >= 180) {
            clockwise = !isMinusUp;
            angle = 360 - angle;
        } else {
            clockwise = isMinusUp;
        }
    }
    if (angle > 90) {
        angle = 180 - angle;
    }
    return std::pair<int, bool>{angle, clockwise};
}

std::pair<double, double> Auxiliary::GetMinMax(std::vector<double> &points) {
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::min();
    for (const auto &point: points) {
        if (min > point) {
            min = point;
        } else if (max < point) {
            max = point;
        }
    }
    return {min, max};
}

double
Auxiliary::GetMinDistance(const std::vector<Point> &points, const std::function<double(Point, Point)> &DistanceFunc) {
    double minDistance = std::numeric_limits<double>::max();
    for (const auto &point1: points) {
        for (const auto &point2: points) {
            double distance = DistanceFunc(point1, point2);
            if (distance != 0 && distance < minDistance) {
                minDistance = distance;
            }
        }
    }
    return minDistance;
}

std::pair<int, bool> Auxiliary::getRotationToTargetInFront(const Point &point1, const Point &point2) {
    Eigen::Vector3d vector1(point1.x, point1.y, 0);
    Eigen::Vector3d vector2(point2.x, point2.y, 0);
    Eigen::Vector3d unitVector1 = vector1 / vector1.norm();
    Eigen::Vector3d unitVector2 = vector2 / vector2.norm();
    int angle = int(radiansToAngle(acos(unitVector1.dot(unitVector2))));
    bool clockwise = unitVector1.cross(unitVector2).z() <= 0;
    if (angle > 90) {
        angle = 180 - angle;
        clockwise = !clockwise;
    }
    return std::pair<int, bool>{angle, clockwise};
}

long Auxiliary::myGcd(long a, long b) {
    if (a == 0)
        return b;
    else if (b == 0)
        return a;

    if (a < b)
        return myGcd(a, b % a);
    else
        return myGcd(b, a % b);
}

std::string Auxiliary::GetDataSetsDirPath() {
    char currentDirPath[256];
    getcwd(currentDirPath, 256);
    std::string settingPath = currentDirPath;
    settingPath += "/../datasets/";
    return settingPath;
}

double Auxiliary::calculateMeanOfDistanceDifferences(std::vector<double> distances) {
    double sumOfDistances = 0.0;
    for (int i = 0; i < distances.size() - 1; ++i) {
        sumOfDistances += std::abs(distances[i] - distances[i + 1]);
    }
    return sumOfDistances / (distances.size() - 1);
}

cv::Mat Auxiliary::getCovarianceMat3D(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z) {
    cv::Mat aux(x.size(), 2, CV_64F);
    double xMean = 0.0;
    double yMean = 0.0;
    double zMean = 0.0;
    for (int i = 0; i < x.size(); ++i) {
        xMean += x[i];
        yMean += y[i];
        yMean += z[i];
    }
    zMean /= x.size();
    yMean /= x.size();
    xMean /= x.size();
    for (int i = 0; i < x.size(); ++i) {
        aux.at<double>(i, 0) = x[i] - xMean;
        aux.at<double>(i, 1) = y[i] - yMean;
        aux.at<double>(i, 2) = z[i] - zMean;
    }
    cv::Mat cov = aux.t() * aux;
    return cov;
}

cv::Mat Auxiliary::getCovarianceMat(std::vector<double> &x, std::vector<double> &y) {
    cv::Mat aux(x.size(), 2, CV_64F);
    double xMean = 0.0;
    double yMean = 0.0;
    for (int i = 0; i < x.size(); ++i) {
        xMean += x[i];
        yMean += y[i];
    }
    yMean /= x.size();
    xMean /= x.size();
    for (int i = 0; i < x.size(); ++i) {
        aux.at<double>(i, 0) = x[i] - xMean;
        aux.at<double>(i, 1) = y[i] - yMean;
    }
    cv::Mat cov = aux.t() * aux;
    return cov;
}

cv::Mat Auxiliary::getPointsMatrix(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z) {
    cv::Mat mat(x.size(), 3, CV_64F);
    for (int i = 0; i < x.size(); ++i) {
        mat.at<double>(i, 0) = x[i];
        mat.at<double>(i, 1) = y[i];
        mat.at<double>(i, 2) = z[i];
    }
    return mat;
}


double Auxiliary::calculateMean(const std::vector<double> &distances) {
    double sumOfDistances = 0.0;
    for (auto distance: distances) {
        sumOfDistances += distance;
    }
    double mean = sumOfDistances / distances.size();
    return mean;
}

double Auxiliary::calculateVariance(const std::vector<double> &distances) {
    double sumOfDistances = 0.0;
    for (auto distance: distances) {
        sumOfDistances += distance;
    }
    double mean = sumOfDistances / distances.size();
    double variance = 0.0;
    for (auto distance: distances) {
        variance += pow(distance - mean, 2);
    }
    return variance / distances.size();
}

double Auxiliary::getAngleBySlopes(Line line1, Line line2) {
    Eigen::Vector3d vector1(1, line1.getSlope(), 0);
    Eigen::Vector3d vector2(1, line2.getSlope(), 0);
    Eigen::Vector3d unitVector1 = vector1 / vector1.norm();
    Eigen::Vector3d unitVector2 = vector2 / vector2.norm();
    return radiansToAngle(acos(unitVector1.dot(unitVector2)));
}

double Auxiliary::GetPitchFrom2Points(const Point &point1, const Point &point2) {
    return radiansToAngle(acos((point1.x * point2.x + point1.y * point2.y) /
                               (sqrt(pow(point1.x, 2) + pow(point1.y, 2)) *
                                sqrt(pow(point2.x, 2) + pow(point2.y, 2)))));
}

std::vector<double> Auxiliary::Get3dAnglesBetween2Points(const Point &point1, const Point &point2) {
    double x = acos((point1.y * point2.y + point1.z * point2.z) /
                    (sqrt(pow(point1.y, 2) + pow(point1.z, 2)) * sqrt(pow(point2.y, 2) + pow(point2.z, 2))));
    double y = acos((point1.x * point2.x + point1.z * point2.z) /
                    (sqrt(pow(point1.x, 2) + pow(point1.z, 2)) * sqrt(pow(point2.x, 2) + pow(point2.z, 2))));
    double z = acos((point1.x * point2.x + point1.y * point2.y) /
                    (sqrt(pow(point1.x, 2) + pow(point1.y, 2)) * sqrt(pow(point2.x, 2) + pow(point2.y, 2))));
    return {x, y, z};
}

std::vector<Point> Auxiliary::getPointsVector(cv::Mat points) {
    std::vector<Point> vec{};

    for (int i = 0; i < points.rows; i++)
        vec.emplace_back(points.at<double>(i, 0), points.at<double>(i, 1), points.at<double>(i, 2));

    return vec;
}