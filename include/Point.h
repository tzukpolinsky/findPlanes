#ifndef TELLO_POINT_H
#define TELLO_POINT_H

#include <string>
#include <sstream>
#include <cmath>

class Point {
public:
    Point();

    Point(const Point &point);

    /*
     * a wrapper over the orb_slam2 representation of point, every point carry with it all the data we need
     * in order to calculate easy navigation to it
     * all the q* are the quaternion value of the frame that contains the point, the label is the label we
     * give to the point when we cluster it with other points
     * and the frame id is the orb_slam2 frame id in ORBSLAM2::Frame
     *
     * x is right left
     * y is up down
     * z is depth
     * q* is quaternion
     *
     * */
    Point(double x, double y, double z, int lidarOriginalPosition=0,int lidarId = -1,bool isAlgoFloor = false,bool isGroundTruthFloor = false);

    bool operator==(const Point &ref) const {

        return this->x == ref.x && this->y == ref.y && this->z == ref.z;
    }

    bool approximationEquality(const Point &ref,double approximation) const {
        return std::ceil(this->x * approximation) / approximation == std::ceil(ref.x * approximation) / approximation
               && std::ceil(this->y * approximation) / approximation == std::ceil(ref.y * approximation) / approximation
               && std::ceil(this->z * approximation) / approximation == std::ceil(ref.z * approximation) / approximation;
    }
    [[nodiscard]] std::string to_string() const {
        std::ostringstream ss;
        ss << this->x << "," << this->y << "," << this->z << "," << this->lidarId;
        return ss.str();
    }

    Point &operator=(const Point &point) = default;

    double x;
    double y;
    double z;
    int lidarId;
    int lidarOriginalPosition;
    bool isAlgoFloor;
    bool isGroundTruthFloor;

};


#endif //TELLO_POINT_H
