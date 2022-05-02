//
// Created by tzuk on 20/02/2022.
//

#include <iostream>
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
    auto [R_align, mu_align] = calculateAlignMatrices(convertToCvPoints(points));

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


int loopThroughCarsDataBase(std::string &dataBasePath) {
    bool isDebug = false;
    char time_buf[21];
    time_t time_tNow;
    std::time(&time_tNow);
    std::strftime(time_buf, 21, "%Y-%m-%d_%H:%S:%MZ", gmtime(&time_tNow));
    std::string currentTime(time_buf);
    std::ofstream data;
    std::string dataPath = "/home/tzuk/Documents/carsDatasets/data/" + currentTime;
    std::filesystem::create_directory(dataPath);

    for (auto &dir: std::filesystem::directory_iterator(dataBasePath)) {
        //if (i++ > 5) {

        if (std::filesystem::is_directory(dir.path().string() + "/lidar")) {

            for (auto &database: std::filesystem::directory_iterator(dir.path().string() + "/lidar")) {
                Navigation navigation;
                dataPath = "/home/tzuk/Documents/carsDatasets/data/" + currentTime;
                dataPath += "/"+dir.path().string().substr(dir.path().string().size() -15 );
                std::filesystem::create_directory(dataPath);
                dataPath += "/"+database.path().string().substr(dir.path().string().size()+7);
                std::filesystem::create_directory(dataPath);
                data.open(dataPath + "/data.txt");
                std::vector<std::filesystem::path> files_in_directory;
                std::copy(std::filesystem::directory_iterator(database), std::filesystem::directory_iterator(),
                          std::back_inserter(files_in_directory));
                std::sort(files_in_directory.begin(), files_in_directory.end());
                for (auto &npzFile: files_in_directory) {
                    std::string fileName = npzFile.filename().string();
                    //if (fileName.find("center") != std::string::npos) {
                    auto points = Auxiliary::getPointsFromPYZFile(npzFile.string());
                    std::cout << "amount of points: " << points.size() << " for file: " << fileName
                              << std::endl;
                    auto start = std::chrono::high_resolution_clock::now();
                    auto floor = navigation.getFloorAndBruteForceAlignUsingThreads(points, points.size() / 100,
                                                                                   isDebug, fileName);
                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                    start = std::chrono::high_resolution_clock::now();
                    std::pair<long, std::vector<Point>> result;
                    std::vector<Point> rotatedPoints;
                    navigation.alignByAngleThread(points, navigation.bestRoll, navigation.bestPitch,
                                                  points.size() / 100,
                                                  result, rotatedPoints);
                    stop = std::chrono::high_resolution_clock::now();
                    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                    std::cout << "for file: " << fileName << " time in micro seconds: " << duration.count()
                              << std::endl;
                    data << "for file: " << fileName << " time in micro seconds: " << duration.count()
                         << " amount of points: " << points.size() <<
                         " points in floor: " << result.second.size() << std::endl;
                    Auxiliary::displayLidarOnImage(result.second, npzFile.string(), dir.path().string(),
                                                   database.path().string(), fileName, isDebug, dataPath);
                    int amountOfTrueTrue = 0;
                    int amountOfFalseFalse = 0;
                    for (auto &point: result.second) {
                        point.isAlgoFloor = true;
                        amountOfFalseFalse += !point.isGroundTruthFloor;
                        amountOfTrueTrue += point.isAlgoFloor && point.isGroundTruthFloor;
                    }
                    data << "true true: " << amountOfTrueTrue << std::endl;
                    data << "false false: " << points.size() - result.second.size() << std::endl;
                    data << "wrong result: " << amountOfFalseFalse
                         << std::endl;
                    //}
                }
                data.close();
            }
        }
        //}

    }
    data.close();
    return 1;
}

int main() {
    std::string dataBasePath = "/home/tzuk/Documents/carsDatasets/camera_lidar_semantic";
    return loopThroughCarsDataBase(dataBasePath);
}