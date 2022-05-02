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

void
Navigation::getFloorByCovariance(std::vector<Point> &points, unsigned long sizeOfJump,
                                 std::pair<long, std::vector<Point>> &scoreAndFloor, bool isDebug,
                                 std::string pangolinPostfix) {

    /*std::sort(points.begin(), points.end(), [](const Point &point1, const Point &point2) -> bool {
        return point1.z < point2.z;
    });*/
    std::vector<double> z = Auxiliary::getZValues(points);
    std::vector<double> y = Auxiliary::getYValues(points);
    std::vector<double> x = Auxiliary::getXValues(points);
    std::vector<double> pointsSizes;
    std::vector<double> variances;
    std::vector<std::pair<double, unsigned long>> weightedPoints;
    double bestScore = std::numeric_limits<double>::min();
    unsigned long bestSizeZ = 0;
    while (z.size() > sizeOfJump) {
        auto cov = Auxiliary::getCovarianceMat(x, y);
        auto trace = cov.at<double>(0, 0) + cov.at<double>(1, 1);
        auto det = cv::determinant(cov);
        double zVariance = Auxiliary::calculateVariance(z);
        if (isDebug) {
            variances.emplace_back(((det / trace) / (zVariance)));
            pointsSizes.emplace_back(z.size());
        }
        double score = (det / trace) / (zVariance);
        //weightedPoints.emplace_back(score, z.size());
        if (bestScore < score){
            bestScore = score;
            bestSizeZ = z.size();
        }
        z.resize(z.size() - sizeOfJump);
        x.resize(x.size() - sizeOfJump);
        y.resize(y.size() - sizeOfJump);
    }
    /*std::sort(weightedPoints.begin(), weightedPoints.end(),
              [](const std::pair<double, long> &weightedPoint1,
                 const std::pair<double, long> &weightedPoint2) -> bool {
                  return weightedPoint1.first > weightedPoint2.first;
              });*/

    if (!isDebug) {
        scoreAndFloor.first = bestScore;
        scoreAndFloor.second = std::vector<Point>(points.begin(), points.begin() + bestSizeZ);
        return;
    }
    /*auto bestPoints = std::pair<double, std::vector<Point>>{weightedPoints.front().first,
                                                            std::vector<Point>(points.begin(), points.begin() +
                                                                                               weightedPoints.front().second)};*/
    scoreAndFloor.first = bestScore;
    scoreAndFloor.second = std::vector<Point>(points.begin(), points.begin() + weightedPoints.front().second);
    if (isDebug) {
        Auxiliary::showGraph(pointsSizes, variances, "ro");
        Auxiliary::SetupPangolin("floor" + pangolinPostfix);
            Auxiliary::DrawMapPointsPangolin(points, scoreAndFloor.second , "floor" + pangolinPostfix);

    }

    return;
    //return bestPoints;
}

std::vector<Point>
Navigation::alignByFloor(std::vector<Point> points, std::vector<Point> floor, int heightDirection) {
    auto floorMean = Auxiliary::getMean(floor);
    for (auto &point: points) {
        point.x -= floorMean.x;
        point.y -= floorMean.y;
        point.z -= floorMean.z;
    }
    for (auto &point: floor) {
        point.x -= floorMean.x;
        point.y -= floorMean.y;
        point.z -= floorMean.z;
    }
    cv::Mat floorMat(points.size(), 3, CV_64F);
    cv::Mat pointsMat(points.size(), 3, CV_64F);

    Auxiliary::getPointsMatrix(floor, floorMat);
    Auxiliary::getPointsMatrix(points, pointsMat);
    cv::PCA floorPca(floorMat, cv::Mat(), CV_PCA_DATA_AS_ROW, 0);
    cv::Mat changeOfBasis = floorPca.eigenvectors.inv();
    changeOfBasis.at<double>(0, 2) *= heightDirection;
    changeOfBasis.at<double>(1, 2) *= heightDirection;
    changeOfBasis.at<double>(2, 2) *= heightDirection;
    cv::Mat alignedPointsMatrix = changeOfBasis * pointsMat.t();
    std::vector<Point> result;
    Auxiliary::getPointsVector(alignedPointsMatrix.t(),points, result);
    return result;

}

void
Navigation::alignByAngle(std::vector<Point> &points, double roll, double pitch, std::vector<Point> &rotatedPoints) {
    cv::Mat pointsMat(points.size(), 3, CV_64F);
    Auxiliary::getPointsMatrix(points, pointsMat);

    cv::Mat rotationMat = Auxiliary::build3DRotationMatrix(roll, pitch);
    cv::Mat alignedPointsMatrix = rotationMat * pointsMat.t();
    Auxiliary::getPointsVector(alignedPointsMatrix.t(),points, rotatedPoints);
}

void Navigation::alignByAngleThread(std::vector<Point> &points, int roll, int pitch, int sizeOfJump,
                                    std::pair<long, std::vector<Point>> &result, std::vector<Point> &rotatedPoints) {
    alignByAngle(points, Auxiliary::angleToRadians(pitch),
                 Auxiliary::angleToRadians(roll), rotatedPoints);
    getFloorByCovariance(rotatedPoints, sizeOfJump, result, false);
    //result.first = resultCurrent.first;
    //result.second = resultCurrent.second;
    result.first *= result.second.size();
}

std::vector<Point>
Navigation::getFloorAndBruteForceAlignUsingThreads(std::vector<Point> &points, unsigned long sizeOfJump, bool isDebug,
                                                   std::string pangolinPostfix) {
    long maxScore = std::numeric_limits<long>::min();
    std::vector<Point> bestFloor;
    std::vector<Point> bestCloud;
    auto numberOfThreads = std::thread::hardware_concurrency();
    std::vector<std::thread> threads(numberOfThreads);
    std::vector<std::pair<long, std::vector<Point>>> results(numberOfThreads);
    std::vector<std::vector<Point>> clouds(numberOfThreads);
    int pitch;
    int roll;
    int startRoll = bestRoll == std::numeric_limits<int>::min() ? 0 : bestRoll - 2;
    int endRoll = bestRoll == std::numeric_limits<int>::min() ? 20 : bestRoll + 2;
    int startPitch = bestRoll == std::numeric_limits<int>::min() ? 0 : bestPitch - 2;
    int endPitch = bestRoll == std::numeric_limits<int>::min() ? 20 : bestPitch + 2;
    int currentThread = 0;
    std::sort(points.begin(), points.end(), [](const Point &point1, const Point &point2) -> bool {
        return point1.z < point2.z;
    });
    for (int rollAngle = startRoll; rollAngle < endRoll; rollAngle += 2) {
        for (int pitchAngle = startPitch; pitchAngle <= endPitch; pitchAngle += 2) {
            if (currentThread == numberOfThreads) {
                for (int i = 0; i < numberOfThreads; ++i) {
                    threads[i].join();
                    if (results[i].first > maxScore) {
                        maxScore = results[i].first;
                        bestFloor = results[i].second;
                        pitch = i - 10 + i * 2;
                        roll = i * 2;
                        bestCloud = clouds[i];
                    }
                }
                currentThread = 0;
            }
            //results.push_back(std::make_pair<long,std::vector<Point>>(0, std::vector<Point>{points.size()}));
            //clouds.emplace_back(std::vector<Point>(points.size()));
            std::thread t(&Navigation::alignByAngleThread, this, std::ref(points), rollAngle, pitchAngle,
                          sizeOfJump, std::ref(results[currentThread]), std::ref(clouds[currentThread]));
            threads[currentThread++] = std::move(t);
        }
    }

    for (int i = 0; i < numberOfThreads; ++i) {
        if (threads[i].joinable()) {
            threads[i].join();
            if (results[i].first > maxScore) {
                maxScore = results[i].first;
                bestFloor = results[i].second;
                pitch = i - 10 + i * 2;
                roll = i * 2;
                bestCloud = clouds[i];
            }
        }

    }
    bestRoll = roll;
    bestPitch = pitch;
    if (isDebug) {
        Auxiliary::SetupPangolin("floor" + pangolinPostfix);
        Auxiliary::DrawMapPointsPangolin(bestCloud, bestFloor, "floor" + pangolinPostfix);
    }
    return bestFloor;
}

std::vector<Point>
Navigation::getFloorAndBruteForceAlign(std::vector<Point> &points, unsigned long sizeOfJump, bool isDebug,
                                       std::string pangolinPostfix) {
    long maxScore = std::numeric_limits<long>::min();
    std::vector<Point> bestFloor;
    std::vector<Point> bestCloud;
    int pitch;
    int roll;
    if (bestRoll == std::numeric_limits<int>::min()) {
        for (int rollAngle = 0; rollAngle < 20; rollAngle += 2) {
            for (int pitchAngle = -10; pitchAngle <= 10; pitchAngle += 2) {
                std::pair<long, std::vector<Point>> scoreAndFloor;
                std::vector<Point> rotatedPoints;
                alignByAngle(points, Auxiliary::angleToRadians(pitchAngle),
                             Auxiliary::angleToRadians(rollAngle), rotatedPoints);
                getFloorByCovariance(rotatedPoints, sizeOfJump, scoreAndFloor, false, pangolinPostfix);
                scoreAndFloor.first *= scoreAndFloor.second.size();
                if (scoreAndFloor.first > maxScore) {
                    maxScore = scoreAndFloor.first;
                    pitch = pitchAngle;
                    roll = rollAngle;
                    bestFloor = scoreAndFloor.second;
                    bestCloud = rotatedPoints;
                    std::cout << "new best floor for pitch: " << rollAngle << " and roll: " << pitchAngle << std::endl;
                }
            }
        }
    } else {
        for (int rollAngle = bestRoll - 2; rollAngle < bestRoll + 2; rollAngle += 2) {
            for (int pitchAngle = bestPitch - 2; pitchAngle < bestPitch + 2; pitchAngle += 2) {
                std::pair<long, std::vector<Point>> scoreAndFloor;
                std::vector<Point> rotatedPoints;
                alignByAngle(points, Auxiliary::angleToRadians(pitchAngle),
                             Auxiliary::angleToRadians(rollAngle), rotatedPoints);
                getFloorByCovariance(rotatedPoints, sizeOfJump, scoreAndFloor, false, pangolinPostfix);
                scoreAndFloor.first *= scoreAndFloor.second.size();
                if (scoreAndFloor.first > maxScore) {
                    maxScore = scoreAndFloor.first;
                    bestFloor = scoreAndFloor.second;
                    pitch = pitchAngle;
                    roll = rollAngle;
                    bestCloud = rotatedPoints;
                    std::cout << "new best floor for pitch: " << rollAngle << " and roll: " << pitchAngle << std::endl;
                }
            }
        }
    }
    bestRoll = roll;
    bestPitch = pitch;
    if (isDebug) {
        Auxiliary::SetupPangolin("floor" + pangolinPostfix);
        Auxiliary::DrawMapPointsPangolin(bestCloud, bestFloor, "floor" + pangolinPostfix);
    }
    return bestFloor;
}

std::vector<Point> Navigation::getFloorAndAlign(std::vector<Point> &points, unsigned long sizeOfJump, bool isDebug,
                                                std::string pangolinPostfix) {
    std::pair<long, std::vector<Point>> scoreAndFloor;
    std::vector<Point> floor;
    getFloorByCovariance(points, sizeOfJump, scoreAndFloor, true, pangolinPostfix);
    for (int i = 0; i < 10; ++i) {
        std::pair<long, std::vector<Point>> currentScoreAndFloorNonFlip;
        std::pair<long, std::vector<Point>> currentScoreAndFloorFlip;

        auto NonFlipAlignedPoints = alignByFloor(points, scoreAndFloor.second, 1);
        getFloorByCovariance(NonFlipAlignedPoints, sizeOfJump, currentScoreAndFloorNonFlip, true,
                             pangolinPostfix);;
        auto FlipAlignedPoints = alignByFloor(points, scoreAndFloor.second, -1);
        getFloorByCovariance(FlipAlignedPoints, sizeOfJump, currentScoreAndFloorFlip, true, pangolinPostfix);;
        if (currentScoreAndFloorNonFlip.second.size() < currentScoreAndFloorFlip.second.size()) {
            std::cout << "non fliped floor was taken" << std::endl;
        }
        floor = currentScoreAndFloorNonFlip.second.size() < currentScoreAndFloorFlip.second.size()
                ? currentScoreAndFloorNonFlip.second
                : currentScoreAndFloorFlip.second;
    }
    return floor;
}

std::vector<Point>
Navigation::getFloorFromOrbSlam(std::vector<Point> &points, unsigned long sizeOfJump, bool isDebug,
                                std::string pangolinPostfix) {
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
Navigation::getFloorFromLidar(std::vector<Point> &points, unsigned long sizeOfJump, bool isDebug,
                              std::string pangolinPostfix) {
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
    if (isDebug) {
        Auxiliary::showGraph(pointsSizes, variances, "ro");
        Auxiliary::SetupPangolin("floor" + pangolinPostfix);
        Auxiliary::DrawMapPointsPangolin(points, weightedPoints.front().second, "floor" + pangolinPostfix);
        //Auxiliary::showCloudPoint(weightedPoints.front().second, points);
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