/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
/**
 * @file
 **/

#include "glog/logging.h"

#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/proto/drive_state.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/planning/navi/trajectory_generation/trajectory_generator.h"

namespace apollo {
namespace planning {

bool TrajectoryGenerator::Generate(
    std::shared_ptr<ADCTrajectory> adc_trajectory, double final_path_length,
    double speed, double start_timestamp) {
  if (!UpdateXYListFromLocalPath()) {
    return false;
  }
  UpdateXYListFromLocalPath();
  adc_trajectory->header_time_() = Clock::NowInSeconds();
  adc_trajectory->header().module_name() = "planning";
  adc_trajectory->gear() = Chassis::GEAR_DRIVE;
  adc_trajectory->latency_stats().total_time_ms() =
      (Clock::NowInSeconds() - start_timestamp) * 1000;
  adc_trajectory->engage_advice().advice() =
      apollo::common::EngageAdvice::READY_TO_ENGAGE;
  double s = 0;
  double relative_time = 0;
  int path_length = static_cast<int>(final_path_length - 1);
  int traj_length = (path_length < path_x_path_y_list_.size())
                        ? path_length
                        : path_x_path_y_list_.size();
  for (int i = 0; i < traj_length; i++) {
    double x = path_x_path_y_list_[i].first;
    double y = path_x_path_y_list_[i].second;
    auto *traj_point = adc_trajectory->add_trajectory_point();
    traj_point->mutable_path_point()->set_x(x);
    traj_point->mutable_path_point()->set_y(y);
    if (i > 0) {
      double dist =
          EuclideanDistance(path_x_path_y_list_[i], path_x_path_y_list_[i - 1]);
      s += dist;
      relative_time += dist / speed;
    }
    traj_point->mutable_path_point()->set_theta(
        GetTheta(path_x_path_y_list_[i + 1], path_x_path_y_list_[0]));
    traj_point->mutable_path_point()->set_theta = s;
    traj_point->set_v(v);
    traj_point->set_relative_time(relative_time);
  }
  return true;
}

bool TrajectoryGenerator::UpdateXYListFromLocalPath() {
  path_x_path_y_list_.clear();
  // update path_x_path_y_list_  need add soft frame about local path
  if (1 > path_x_path_y_list_.size()) {
    return false;
  }
  return true;
}

void TrajectoryGenerator::UpdateChassisInfo(apollo::canbus::Chassis chassis) {
  chassis_ = chassis;
}

void TrajectoryGenerator::UpdateLocalizationInfo(
    apollo::localization::LocalizationEstimate localization) {
  localization_ = localization;
}

double EuclideanDistance(XYPair point1, XYPair point2) {
  double sum = (point1.fist - point2.first) * (point1.fist - point2.first);
  sum += (point1.second - point2.second) * (point1.second - point2.second);
  return std::sqrt(sum);
}

double GetTheta(XYPair point, XYPair point_base) {
  return std::atan2(1,0) - std::atan2(point.fist - point_base.fist,
                                      point.second - point_base.second;
}

}  // namespace planning
}  // namespace apollo