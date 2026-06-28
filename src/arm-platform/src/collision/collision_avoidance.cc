#include <manipulator/collision/collision_avoidance.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <iostream>
#include <cmath>

namespace manipulator::collision {

CollisionAvoidance::CollisionAvoidance() = default;

bool CollisionAvoidance::LoadModel(const std::string& urdf_path) {
	try {
		pinocchio::urdf::buildModel(urdf_path, model_);
		data_ = std::make_unique<pinocchio::Data>(model_);
		model_loaded_ = true;
		return true;
	} catch (const std::exception& e) {
		std::cerr << "Failed to load URDF model: " << e.what() << std::endl;
		model_loaded_ = false;
		return false;
	}
}

void CollisionAvoidance::SetPropellers(const std::vector<AABB>& propellers) {
	propellers_ = propellers;
}

void CollisionAvoidance::AddPropeller(const AABB& propeller) {
	propellers_.push_back(propeller);
}

void CollisionAvoidance::AddCylinder(const Cylinder& cylinder) {
	cylinders_.push_back(cylinder);
}

void CollisionAvoidance::ClearObstacles() {
	propellers_.clear();
	cylinders_.clear();
}

void CollisionAvoidance::SetSafetyDistance(double distance) {
	safety_distance_ = distance;
}

double CollisionAvoidance::GetSafetyDistance() const {
	return safety_distance_;
}

std::vector<std::pair<Vec3, Vec3>> CollisionAvoidance::GetLinkSegments(pinocchio::Data& data) {
	std::vector<std::pair<Vec3, Vec3>> segments;
	
	for (int i = 1; i < model_.njoints; ++i) {
		int parent = model_.parents[i];
		if (parent == 0) continue;
		
		Vec3 p1 = data.oMi[parent].translation();
		Vec3 p2 = data.oMi[i].translation();
		
		segments.emplace_back(p1, p2);
	}
	
	return segments;
}

CollisionResult CollisionAvoidance::CheckCollision(const Eigen::VectorXd& q) {
	CollisionResult result;
	result.has_collision_risk = false;
	result.min_distance = 1e9;
	result.colliding_link_index = -1;
	result.colliding_obstacle_index = -1;
	
	if (!model_loaded_ || (propellers_.empty() && cylinders_.empty())) {
		return result;
	}
	
	if (q.size() == 0) {
		return result;
	}
	
	int nq = std::min(static_cast<int>(q.size()), model_.nq);
	Eigen::VectorXd q_internal = Eigen::VectorXd::Zero(model_.nq);
	for (int i = 0; i < nq; ++i) {
		q_internal[i] = q[i];
	}
	
	pinocchio::forwardKinematics(model_, *data_, q_internal);
	pinocchio::updateGlobalPlacements(model_, *data_);
	
	auto segments = GetLinkSegments(*data_);
	
	for (size_t link_idx = 0; link_idx < segments.size(); ++link_idx) {
		const auto& segment = segments[link_idx];
		
		for (size_t prop_idx = 0; prop_idx < propellers_.size(); ++prop_idx) {
			const auto& prop = propellers_[prop_idx];
			double dist = SegmentAABBDistance(segment.first, segment.second, prop);
			
			if (dist < result.min_distance) {
				result.min_distance = dist;
				result.colliding_link_index = static_cast<int>(link_idx);
				result.colliding_obstacle_index = static_cast<int>(prop_idx);
			}
			
			if (dist < safety_distance_) {
				result.has_collision_risk = true;
			}
		}
		
		for (size_t cyl_idx = 0; cyl_idx < cylinders_.size(); ++cyl_idx) {
			const auto& cyl = cylinders_[cyl_idx];
			double dist = SegmentCylinderDistance(segment.first, segment.second, cyl);
			
			if (dist < result.min_distance) {
				result.min_distance = dist;
				result.colliding_link_index = static_cast<int>(link_idx);
				result.colliding_obstacle_index = static_cast<int>(propellers_.size() + cyl_idx);
			}
			
			if (dist < safety_distance_) {
				result.has_collision_risk = true;
			}
		}
	}
	
	return result;
}

Eigen::VectorXd CollisionAvoidance::AdjustTarget(const Eigen::VectorXd& current_q,
												 const Eigen::VectorXd& target_q,
												 double step_size) {
	CollisionResult result = CheckCollision(target_q);
	
	if (!result.has_collision_risk) {
		return target_q;
	}
	
	Eigen::VectorXd adjusted_q = current_q;
	Eigen::VectorXd direction = target_q - current_q;
	double total_distance = direction.norm();
	
	if (total_distance < 1e-6) {
		return current_q;
	}
	
	direction.normalize();
	
	int max_iterations = static_cast<int>(total_distance / step_size) + 1;
	
	for (int i = 1; i <= max_iterations; ++i) {
		double t = static_cast<double>(i) / max_iterations;
		Eigen::VectorXd test_q = current_q + t * (target_q - current_q);
		
		CollisionResult test_result = CheckCollision(test_q);
		
		if (test_result.has_collision_risk) {
			double prev_t = static_cast<double>(i - 1) / max_iterations;
			return current_q + prev_t * (target_q - current_q);
		}
		
		adjusted_q = test_q;
	}
	
	return adjusted_q;
}

} // namespace manipulator::collision