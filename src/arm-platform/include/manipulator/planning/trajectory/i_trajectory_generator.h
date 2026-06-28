#pragma once
#include <vector>
#include <memory>
#include <manipulator/common_types.h>

namespace manipulator::planning {

class ITrajectoryGenerator {
 public:
  using UniPtr = std::unique_ptr<ITrajectoryGenerator>;

  virtual ~ITrajectoryGenerator() = default;

  virtual void Generate(
      JointTrajectory& traj,
      const std::vector<double>& start,
      const std::vector<double>& goal,
      double v_max = 0,
      double a_max = 0,
      double j_max = 0) = 0;
};

} // namespace manipulator::planning
