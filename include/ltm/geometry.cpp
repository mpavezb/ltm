#include "geometry.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/algorithms/assign.hpp>

namespace ltm {

    // ==================== Convex Hull ================================================================================
    // Implementation of Andrew's monotone chain 2D convex hull algorithm.
    // Asymptotic complexity: O(n log n).
    // Source: https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain
    bool hull_compare_points(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) {
        return p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y);
    }

    // 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
    // Returns a positive value, if OAB makes a counter-clockwise turn,
    // negative for clockwise turn, and zero if the points are collinear.
    double
    hull_cross_product(const geometry_msgs::Point &O, const geometry_msgs::Point &A, const geometry_msgs::Point &B) {
        return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
    }

    bool equals_point(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
        return (p1.x == p2.x && p1.y == p2.y);
    }

    // Returns a list of points on the convex hull in counter-clockwise order.
    // Note: the last point in the returned list is the same as the first one.
    void convex_hull_2d(const std::vector<geometry_msgs::Point> &points, std::vector<geometry_msgs::Point> &hull) {
        size_t n = points.size();
        size_t k = 0;
        geometry_msgs::Point p1, p2, p3;
        if (n == 1) {
            hull = points;
            return;
        }
        if (n == 2) {
            p1 = points[0];
            p2 = points[1];
            if (equals_point(p1, p2)) {
                hull.clear();
                hull.push_back(p1);
                return;
            }
        }
        if (n == 3) {
            p1 = points[0];
            p2 = points[1];
            p3 = points[2];
            bool keep_p2 = false;
            bool keep_p3 = false;
            if (equals_point(p1, p2)) {
                if (!equals_point(p2, p3)) {
                    keep_p3 = true;
                }
            } else {
                keep_p2 = true;
                if (!equals_point(p1, p3) && !equals_point(p2, p3)) {
                    keep_p3 = true;
                }
            }
            hull.clear();
            hull.push_back(p1);
            if (keep_p2) hull.push_back(p2);
            if (keep_p3) hull.push_back(p3);
            return;
        }

        std::vector<geometry_msgs::Point> P = points;
        std::vector<geometry_msgs::Point> H(2 * n);

        // Sort points lexicographically
        std::sort(P.begin(), P.end(), hull_compare_points);

        // Build lower hull
        for (size_t i = 0; i < n; ++i) {
            while (k >= 2 && hull_cross_product(H[k - 2], H[k - 1], P[i]) <= 0) k--;
            H[k++] = P[i];
        }

        // Build upper hull
        for (size_t i = n - 1, t = k + 1; i > 0; --i) {
            while (k >= t && hull_cross_product(H[k - 2], H[k - 1], P[i - 1]) <= 0) k--;
            H[k++] = P[i - 1];
        }

        H.resize(k - 1);
        hull = H;
    }
    // ==================== Convex Hull ================================================================================


    void boost_hull(const std::vector<geometry_msgs::Point> &points, std::vector<geometry_msgs::Point> &hull) {
        typedef boost::geometry::model::d2::point_xy<double> point_type;
        typedef boost::geometry::model::polygon<point_type> polygon_type;

        // create boost polygon
        std::vector<point_type> boost_points;
        std::vector<geometry_msgs::Point>::const_iterator it;
        for (it = points.begin(); it != points.end(); ++it) {
            boost_points.push_back(point_type(it->x, it->y));
        }
        polygon_type polygon;
        boost::geometry::assign_points(polygon, boost_points);

        // compute hull
        polygon_type boost_hull;
        boost::geometry::convex_hull(polygon, boost_hull);

        // TODO: transform to geometry_msgs::Point vector
    }

    geometry_msgs::Point polygon_centroid_2d(const std::vector<geometry_msgs::Point> &points) {

        typedef boost::geometry::model::d2::point_xy<double> point_type;
        typedef boost::geometry::model::polygon<point_type> polygon_type;

        // create boost polygon
        std::vector<point_type> boost_points;
        std::vector<geometry_msgs::Point>::const_iterator it;
        for (it = points.begin(); it != points.end(); ++it) {
            boost_points.push_back(point_type(it->x, it->y));
        }
        polygon_type polygon;
        boost::geometry::assign_points(polygon, boost_points);

        // compute centroid
        point_type p;
        boost::geometry::centroid(polygon, p);

        // return
        geometry_msgs::Point result;
        result.x = p.x();
        result.y = p.y();
        return result;
    }

    std::string point_vector_to_str(const std::vector<geometry_msgs::Point>& array) {
        std::vector<geometry_msgs::Point>::const_iterator it;
        std::stringstream ss;
        ss << "[";
        for (it = array.begin(); it != array.end(); ++it) {
            ss << "(" << it->x << ", " << it->y << ")" << ", ";
        }
        ss.seekp(-2, ss.cur);
        ss << "]";
        return ss.str();
    }

}
