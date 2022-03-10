//
// Created by rbdstudent on 27/06/2021.
//

#include "include/Navigation.h"
#include<fstream>
#include <chrono>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <cnpy.h>
#include <filesystem>

cv::Mat points3d_to_mat(const std::vector<cv::Point3d> &points3d) {
    std::size_t nPoints = points3d.size();
    cv::Mat mat((int) nPoints, 3, CV_64FC1);
    for (std::size_t i = 0; i < nPoints; i++) {
        mat.at<double>(i, 0) = points3d[i].x;
        mat.at<double>(i, 1) = points3d[i].y;
        mat.at<double>(i, 2) = points3d[i].z;
    }

    return mat.t();
}

std::pair<cv::Mat, cv::Mat> calculateAlignMatrices(std::vector<cv::Point3d> points) {
    cv::Mat mu_align1;
    cv::Mat R_align;
    cv::reduce(points, mu_align1, 01, CV_REDUCE_AVG);

    cv::Point3d mu_align_pnt(mu_align1.at<double>(0), mu_align1.at<double>(1), mu_align1.at<double>(2));
    cv::Mat mu_align(mu_align_pnt);

    std::cout << "Centering points" << std::endl;
    for (auto &p: points) {
        p = p - mu_align_pnt;
    }

    cv::Mat A = points3d_to_mat(points);
    cv::Mat w, u, vt;
    // cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    cv::SVDecomp(A, w, u, vt);
    R_align = u.t();


    return {R_align, mu_align};
}

std::vector<cv::Point3d> convertToCvPoints(const std::vector<Point> &points) {
    std::vector<cv::Point3d> cvPoints;
    for (const auto &point: points) {
        cvPoints.emplace_back(cv::Point3d(point.x, point.y, point.z));
    }
    return cvPoints;
}

std::vector<Point> convertCVToPoints(const std::vector<cv::Point3d> &cvpoints) {
    std::vector<Point> points;
    for (const auto &point: cvpoints) {
        points.emplace_back(Point(point.x, point.y, point.z));
    }
    return points;
}

cv::Mat convertPointToCVMat(const Point &point) {
    cv::Mat pnt3d(1, 3, CV_64FC1);
    pnt3d.at<double>(0, 0) = point.x;
    pnt3d.at<double>(0, 1) = point.y;
    pnt3d.at<double>(0, 2) = point.z;
    return pnt3d.t();
}

std::pair<cv::Mat, cv::Mat> align_map(std::vector<Point> &points) {
    auto[R_align, mu_align] = calculateAlignMatrices(convertToCvPoints(points));

    for (auto &point: points) {
        auto pnt3d = convertPointToCVMat(point);
        cv::Mat align_pos;
        align_pos = R_align * (pnt3d - mu_align);

        point.x = align_pos.at<double>(0, 0);
        point.y = align_pos.at<double>(1, 0);
        point.z = align_pos.at<double>(2, 0);
    }
    return {R_align, mu_align};
}

std::vector<Point> getPointsFromFile(const std::string &fileName) {
    std::unordered_map<int, std::vector<Point>> framesMap;
    std::ifstream myFile(fileName);
    std::string line;
    std::vector<Point> allPoints;
    while (std::getline(myFile, line)) {
        std::stringstream lineStream(line);
        Point point;
        lineStream >> point.x;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.z;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.y;

        allPoints.emplace_back(point);
    }
    return allPoints;

}

std::vector<Point> getPointsFromPYZFile(const std::string &fileName) {
    std::map<std::string, cnpy::NpyArray> npPointsMap = cnpy::npz_load(fileName);
    auto numPyPoints = npPointsMap["points"].as_vec<double>();
    std::vector<Point> points;
    for (int i = 0; i < numPyPoints.size(); i += 3) {
        points.emplace_back(Point(numPyPoints[i], numPyPoints[i + 1], numPyPoints[i + 2]));
    }

    return points;
}

std::vector<Point> getPointsFromXYZFile(const std::string &fileName) {
    std::ifstream myFile(fileName);
    std::string line;
    std::vector<Point> allPoints;
    while (std::getline(myFile, line)) {
        std::stringstream lineStream(line);
        Point point;
        lineStream >> point.x;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.z;
        if (lineStream.peek() == ',') lineStream.ignore();
        lineStream >> point.y;
        allPoints.emplace_back(point);
    }
    return allPoints;

}

int loopThroughCarsDataBase(std::string &dataBasePath) {
    Navigation navigation;
    for (auto &dir: std::filesystem::directory_iterator(dataBasePath)) {
        if (std::filesystem::is_directory(dir.path().string() + "/lidar")) {
            for (auto &database: std::filesystem::directory_iterator(dir.path().string() + "/lidar")) {
                for (auto &npzFile: std::filesystem::directory_iterator(database)) {
                    std::string fileName = npzFile.path().filename().string();
                    auto points = getPointsFromPYZFile(npzFile.path().string());
                    std::cout << "amount of points: " << points.size() << " for file: " << fileName
                              << std::endl;
                    auto start = std::chrono::high_resolution_clock::now();
                    navigation.getFloorFromLidar(points, points.size() / 100, true, fileName);
                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                    std::cout << "for file: " << fileName << "time : " << duration.count() << std::endl;
                }
            }
        }

    }
    return 1;
}

int lidar(std::string &datasetFilePath) {
    auto points = getPointsFromPYZFile(datasetFilePath);
    std::cout << "amount of points: " << points.size() << std::endl;
    auto start = std::chrono::high_resolution_clock::now();

    Navigation navigation;
    navigation.getFloorFromLidar(points, points.size() / 100, true);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << duration.count() << std::endl;
    return 1;
}

int orbSlam(std::string &datasetFilePath) {
    auto points = getPointsFromFile(datasetFilePath);
    auto[R, T] = align_map(points);
    std::cout << "amount of points: " << points.size() << std::endl;
    auto start = std::chrono::high_resolution_clock::now();

    Navigation navigation;
    navigation.getFloorFromOrbSlam(points, points.size() / 100, true);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << duration.count() << std::endl;
    return 1;
}

int main() {
    std::string datasetFilePath =
            Auxiliary::GetDataSetsDirPath() + "buildings/pointData1.csv";
    orbSlam(datasetFilePath);

    /*std::string datasetFilePath ="/home/tzuk/Documents/carsDatasets/camera_lidar_semantic/20181108_103155/lidar/cam_front_center/20181108103155_lidar_frontcenter_000000028.npz";
    lidar(datasetFilePath);*/

}