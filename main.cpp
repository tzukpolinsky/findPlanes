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

std::pair<cv::Mat, cv::Mat> calculate_align_matrices(std::vector<cv::Point3d> points) {
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
    auto[R_align, mu_align] = calculate_align_matrices(convertToCvPoints(points));

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

int main() {
    std::string datasetFilePath = Auxiliary::GetDataSetsDirPath() + "buildings/pointData1.csv";
    std::cout << "dataset file path" << datasetFilePath << std::endl;
    auto points = getPointsFromFile(datasetFilePath);
    auto[R, T] = align_map(points);
    auto start = std::chrono::high_resolution_clock::now();

    Navigation navigation;
    //std::vector<Point> track{Point(0, 0, -0.05), Point(0.3, 0.1, -0.1)};
    //Point currentPosition(0, 0, 0);
    //navigation.objectDetection(points, track, currentPosition);

    navigation.getFloor(points, points.size() / 100);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << duration.count() << std::endl;
}