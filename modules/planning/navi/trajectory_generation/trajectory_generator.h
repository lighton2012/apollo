#ifndef MODULES_PLANNING_NAVI_TRAJECTORY_GENERATION_TRAJECTORY_GENERATOR_H_
#define MODULES_PLANNING_NAVI_TRAJECTORY_GENERATION_TRAJECTORY_GENERATOR_H_

#include <memory>
#include <list>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/drivers/proto/mobileye.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/common/time/time.h"


namespace apollo {
namespace planning {

typedef std::pair<double, double> XYPair;

class TrajectoryGenerator {
 public:
  TrajectoryEvaluator(){
    mobileye_.clear();
  }

  virtual ~TrajectoryEvaluator() = default;
  
  bool Generate(std::shared_ptr<ADCTrajectory> adc_trajectory,
                double final_path_length,double speed,double start_timestamp);
 private:
  double EuclideanDistance(XYPair point1,XYPair point2);

  double GetTheta(XYPair point,XYPair point_base);

  bool UpdateXYListFromLocalPath();
  
  void UpdateChassisInfo(apollo::canbus::Chassis chassis);
  
  void UpdateLocalizationInfo(apollo::localization::LocalizationEstimate localization);

  apollo::canbus::Chassis chassis_;
  
  apollo::localization::LocalizationEstimate localization_

  apollo::drivers::Mobileye mobileye_
  
  std::vector<XYPair> path_x_path_y_list_;
};

}  // namespace planning
}  // namespace apollo

#endif
//MODULES_PLANNING_NAVI_TRAJECTORY_GENERATION_TRAJECTORY_GENERATOR_H_


