#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace manipulator::collision {

using Vec3 = Eigen::Vector3d;

struct AABB {
    Vec3 min;
    Vec3 max;
    
    AABB() : min(Vec3::Zero()), max(Vec3::Zero()) {}
    AABB(const Vec3& min_pt, const Vec3& max_pt) : min(min_pt), max(max_pt) {}
};

struct Cylinder {
    Vec3 center;
    double radius;
    double height;
    Vec3 axis;
    
    Cylinder() : center(Vec3::Zero()), radius(0.0), height(0.0), axis(Vec3(0, 0, 1)) {}
    Cylinder(const Vec3& c, double r, double h, const Vec3& a = Vec3(0, 0, 1))
        : center(c), radius(r), height(h), axis(a.normalized()) {}
};

inline double Clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(x, hi));
}

inline double PointAABBDistance(const Vec3& p, const AABB& box) {
    double dx = std::max({box.min.x() - p.x(), 0.0, p.x() - box.max.x()});
    double dy = std::max({box.min.y() - p.y(), 0.0, p.y() - box.max.y()});
    double dz = std::max({box.min.z() - p.z(), 0.0, p.z() - box.max.z()});
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

inline double SegmentAABBDistance(const Vec3& p1, const Vec3& p2, const AABB& box, int samples = 10) {
    double min_dist = 1e9;
    
    for (int i = 0; i <= samples; ++i) {
        double t = static_cast<double>(i) / samples;
        Vec3 p = (1 - t) * p1 + t * p2;
        min_dist = std::min(min_dist, PointAABBDistance(p, box));
    }
    return min_dist;
}

inline double PointCylinderDistance(const Vec3& p, const Cylinder& cyl) {
    Vec3 rel = p - cyl.center;
    double axial_dist = rel.dot(cyl.axis);
    double half_height = cyl.height / 2.0;
    
    axial_dist = Clamp(axial_dist, -half_height, half_height);
    
    Vec3 axial_point = cyl.center + axial_dist * cyl.axis;
    Vec3 radial_vec = p - axial_point;
    double radial_dist = radial_vec.norm();
    
    double dist_to_surface = radial_dist - cyl.radius;
    
    if (dist_to_surface < 0) {
        return 0.0;
    }
    
    return dist_to_surface;
}

inline double SegmentCylinderDistance(const Vec3& p1, const Vec3& p2, const Cylinder& cyl, int samples = 10) {
    double min_dist = 1e9;
    
    for (int i = 0; i <= samples; ++i) {
        double t = static_cast<double>(i) / samples;
        Vec3 p = (1 - t) * p1 + t * p2;
        min_dist = std::min(min_dist, PointCylinderDistance(p, cyl));
    }
    return min_dist;
}

} // namespace manipulator::collision
