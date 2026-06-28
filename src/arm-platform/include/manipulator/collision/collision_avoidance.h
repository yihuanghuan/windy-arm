#pragma once

#include <memory>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <manipulator/collision/aabb.h>

namespace manipulator::collision {

struct CollisionResult {
    bool has_collision_risk;
    double min_distance;
    int colliding_link_index;
    int colliding_obstacle_index;
};

class CollisionAvoidance {
public:
    using Ptr = std::shared_ptr<CollisionAvoidance>;
    
    CollisionAvoidance();
    ~CollisionAvoidance() = default;
    
    bool LoadModel(const std::string& urdf_path);
    
    void SetPropellers(const std::vector<AABB>& propellers);
    void AddPropeller(const AABB& propeller);
    void AddCylinder(const Cylinder& cylinder);
    void ClearObstacles();
    
    void SetSafetyDistance(double distance);
    double GetSafetyDistance() const;
    
    CollisionResult CheckCollision(const Eigen::VectorXd& q);
    
    Eigen::VectorXd AdjustTarget(const Eigen::VectorXd& current_q,
                                 const Eigen::VectorXd& target_q,
                                 double step_size = 0.01);
    
    const std::vector<AABB>& GetPropellers() const { return propellers_; }
    const std::vector<Cylinder>& GetCylinders() const { return cylinders_; }
    
private:
    std::vector<std::pair<Vec3, Vec3>> GetLinkSegments(pinocchio::Data& data);
    
    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    std::vector<AABB> propellers_;
    std::vector<Cylinder> cylinders_;
    double safety_distance_ = 0.1;
    bool model_loaded_ = false;
};

} // namespace manipulator::collision
