#ifndef LTM_GEOMETRY_H
#define LTM_GEOMETRY_H

#include <string>
#include <vector>
#include <algorithm>
#include <geometry_msgs/Point.h>

namespace ltm {

    bool hull_compare_points(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);

    // 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
    // Returns a positive value, if OAB makes a counter-clockwise turn,
    // negative for clockwise turn, and zero if the points are collinear.
    double
    hull_cross_product(const geometry_msgs::Point &O, const geometry_msgs::Point &A, const geometry_msgs::Point &B);

    // Returns a list of points on the convex hull in counter-clockwise order.
    // Note: the last point in the returned list is the same as the first one.
    void convex_hull_2d(const std::vector<geometry_msgs::Point> &points, std::vector<geometry_msgs::Point> &hull);

    // incomplete!
    void boost_hull(const std::vector<geometry_msgs::Point> &points, std::vector<geometry_msgs::Point> &hull);

    std::string point_vector_to_str(const std::vector<geometry_msgs::Point> &array);

    geometry_msgs::Point polygon_centroid_2d(const std::vector<geometry_msgs::Point> &points);

    bool equals_point(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);

}

#endif //LTM_GEOMETRY_H
