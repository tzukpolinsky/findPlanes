//
// Created by tzuk on 20/02/2022.
//

#include <iostream>
#include <cnpy.h>
#include <filesystem>
#include "Navigation.h"

std::vector<Point> getPointsFromPYZFile(const std::string &fileName) {
    std::map<std::string, cnpy::NpyArray> npPointsMap = cnpy::npz_load(fileName);
    auto numPyPoints = npPointsMap["points"].as_vec<double>();
    auto lidarIds = npPointsMap["lidar_id"].as_vec<double>();
    std::vector<Point> points;
    int lidarId = 0;
    for (int i = 0; i < numPyPoints.size(); i += 3) {
        points.emplace_back(Point(numPyPoints[i], numPyPoints[i + 1], numPyPoints[i + 2], lidarIds[lidarId++]));
    }

    return points;
}

int loopThroughCarsDataBase(std::string &dataBasePath) {
    Navigation navigation;
    int i = 0;
    for (auto &dir: std::filesystem::directory_iterator(dataBasePath)) {
        //if (i++ > 5) {
        if (std::filesystem::is_directory(dir.path().string() + "/lidar")) {
            for (auto &database: std::filesystem::directory_iterator(dir.path().string() + "/lidar")) {
                for (auto &npzFile: std::filesystem::directory_iterator(database)) {
                    std::string fileName = npzFile.path().filename().string();
                    //if (fileName.find("center") != std::string::npos) {
                    auto points = getPointsFromPYZFile(npzFile.path().string());
                    std::cout << "amount of points: " << points.size() << " for file: " << fileName
                              << std::endl;
                    auto start = std::chrono::high_resolution_clock::now();
                    navigation.getFloorByCovariance(points, points.size() / 100, true, fileName);
                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                    std::cout << "for file: " << fileName << "time : " << duration.count() / 1000 << std::endl;
                    //            }
                }
            }
        }
        //}

    }
    return 1;
}

int main() {
    std::string dataBasePath = "/home/tzuk/Documents/carsDatasets/camera_lidar_semantic";
    return loopThroughCarsDataBase(dataBasePath);
}