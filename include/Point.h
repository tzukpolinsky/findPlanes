#ifndef TELLO_POINT_H
#define TELLO_POINT_H

#include <string>
#include <sstream>

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
    Point(double x, double y, double z, int lidarId = -1);

    bool operator==(const Point &ref) const {
        return this->x == ref.x && this->y == ref.y && this->z == ref.z;
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
};


#endif //TELLO_POINT_H
