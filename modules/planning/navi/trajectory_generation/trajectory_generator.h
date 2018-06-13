#ifndef MODULES_PLANNING_NAVI_TRAJECTORY_GENERATION_TRAJECTORY_GENERATOR_H_
#define MODULES_PLANNING_NAVI_TRAJECTORY_GENERATION_TRAJECTORY_GENERATOR_H_

#include <memory>

namespace apollo {
namespace planning {

class TrajectoryGenerator {
 public:
  explicit TrajectoryEvaluator() = default;

  virtual ~TrajectoryEvaluator() = default;
  
  void Init();
  
  void Generate();
 private:
  double EuclideanDistance(common::);

  double GetTheta();

  std::unique_ptr<ADCTrajectory> adc_trajectory_;
};

}  // namespace planning
}  // namespace apollo

#endif
//MODULES_PLANNING_NAVI_TRAJECTORY_GENERATION_TRAJECTORY_GENERATOR_H_