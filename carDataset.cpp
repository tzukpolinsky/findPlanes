//
// Created by tzuk on 20/02/2022.
//

#include <iostream>
#include <cnpy.h>
#include <filesystem>
#include "Navigation.h"

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
        points.emplace_back(Point(point.x, point.y, point.z, 0));
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

std::vector<double> getColsIndicesFromPYZFile(const std::string &fileName) {
    std::map<std::string, cnpy::NpyArray> npPointsMap = cnpy::npz_load(fileName);
    return npPointsMap["col"].as_vec<double>();
}

std::vector<double> getRowsIndicesFromPYZFile(const std::string &fileName) {
    std::map<std::string, cnpy::NpyArray> npPointsMap = cnpy::npz_load(fileName);
    return npPointsMap["row"].as_vec<double>();
}

std::vector<Point> getPointsFromPYZFile(const std::string &fileName) {
    std::map<std::string, cnpy::NpyArray> npPointsMap = cnpy::npz_load(fileName);
    auto numPyPoints = npPointsMap["points"].as_vec<double>();
    auto lidarIds = npPointsMap["lidar_id"].as_vec<double>();
    std::vector<Point> points;
    int lidarId = 0;
    for (int i = 0; i < numPyPoints.size(); i += 3) {
        points.emplace_back(
                Point(numPyPoints[i], numPyPoints[i + 1], numPyPoints[i + 2], std::floor(i / 3), lidarIds[lidarId++]));
    }

    return points;
}

int loopThroughCarsDataBase(std::string &dataBasePath) {
    Navigation navigation;
    int i = 0;
    bool isDebug = true;
    for (auto &dir: std::filesystem::directory_iterator(dataBasePath)) {
        //if (i++ > 5) {
        if (std::filesystem::is_directory(dir.path().string() + "/lidar")) {
            for (auto &database: std::filesystem::directory_iterator(dir.path().string() + "/lidar")) {
                for (auto &npzFile: std::filesystem::directory_iterator(database)) {
                    std::string fileName = npzFile.path().filename().string();
                    if (fileName.find("side") != std::string::npos) {
                        auto points = getPointsFromPYZFile(npzFile.path().string());
                        //align_map(points);
                        std::cout << "amount of points: " << points.size() << " for file: " << fileName
                                  << std::endl;
                        auto start = std::chrono::high_resolution_clock::now();
                        auto floor = navigation.getFloorByCovariance(points, points.size() / 100, !isDebug, fileName);
                        if (isDebug) {
                            auto rows = getRowsIndicesFromPYZFile(npzFile.path().string());
                            auto cols = getColsIndicesFromPYZFile(npzFile.path().string());
                            std::string imagePath = dir.path().string() + "/camera";
                            std::string cameraLocation = database.path().filename().string().substr(4);
                            for (auto &cameraDir: std::filesystem::directory_iterator(imagePath)) {
                                std::string currentCameraLocation = cameraDir.path().filename().string().substr(4);
                                if (cameraLocation == currentCameraLocation) {
                                    imagePath += "/" + cameraDir.path().filename().string();
                                    break;
                                }
                            }
                            imagePath +=
                                    "/" + fileName.substr(0, 15) + "camera" +
                                    fileName.substr(20, fileName.size() - 24) +
                                    ".png";
                            cv::Mat image = cv::imread(imagePath);
                            Auxiliary::displayLidarOnImage(image, floor, rows, cols);
                        }
                        auto stop = std::chrono::high_resolution_clock::now();
                        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                        std::cout << "for file: " << fileName << "time : " << duration.count() / 1000 << std::endl;
                    }
                }
            }
        }
        //}

    }
    return 1;
}

int main() {
    std::string dataBasePath = "/media/daniel/wd_elements/carsDatasets/camera_lidar_semantic";
    return loopThroughCarsDataBase(dataBasePath);
}