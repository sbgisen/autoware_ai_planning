/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PURE_PURSUIT_PURE_PURSUIT_H
#define PURE_PURSUIT_PURE_PURSUIT_H

// ROS includes
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// C++ includes
#include <vector>

// User defined includes
#include <autoware_msgs/Lane.h>
#include <libwaypoint_follower/libwaypoint_follower.h>

namespace waypoint_follower
{
class PurePursuit
{
public:
  PurePursuit() = default;
  ~PurePursuit() = default;

  // for setting data
  void setLookaheadDistance(const double& ld)
  {
    lookahead_distance_ = ld;
  }
  void setMinimumLookaheadDistance(const double& minld)
  {
    minimum_lookahead_distance_ = minld;
  }
  void setCurrentVelocity(const double& cur_vel)
  {
    current_linear_velocity_ = cur_vel;
  }
  void setCurrentWaypoints(const std::vector<autoware_msgs::Waypoint>& wps)
  {
    current_waypoints_ = wps;
  }
  void setCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    current_pose_ = msg->pose;
  }
  void setLinearInterpolationParameter(const bool& param)
  {
    is_linear_interpolation_ = param;
  }

  void setUseBackward(bool use_back)
  {
    use_back_ = use_back;
  }

  // for debug on ROS
  geometry_msgs::Point getPoseOfNextWaypoint() const
  {
    if (target_waypoint_index_ < 0 || target_waypoint_index_ >= static_cast<int>(current_waypoints_.size()))
    {
      return current_pose_.position;
    }
    return current_waypoints_.at(target_waypoint_index_).pose.pose.position;
  }
  geometry_msgs::Point getPoseOfNextTarget() const
  {
    return next_target_position_;
  }
  geometry_msgs::Pose getCurrentPose() const
  {
    return current_pose_;
  }
  std::vector<autoware_msgs::Waypoint> getCurrentWaypoints() const
  {
    return current_waypoints_;
  }
  double getLookaheadDistance() const
  {
    return lookahead_distance_;
  }
  double getMinimumLookaheadDistance() const
  {
    return minimum_lookahead_distance_;
  }
  // processing
  bool canGetCurvature(double& output_kappa, double& output_velocity);
  bool getCommandVelocity(double& output_velocity);

private:
  // constant
  static constexpr double RADIUS_MAX_ = 9e10;
  static constexpr double RADIUS_MIN_ = 0.3;
  static constexpr double RECOVERY_VEL = 0.2;

  // variables
  bool is_linear_interpolation_{ false };
  bool use_back_{ false };
  int target_waypoint_index_{ -1 };
  int current_waypoint_index_{ -1 };
  double lookahead_distance_{ 0.0 };
  double minimum_lookahead_distance_{ 6.0 };
  double current_linear_velocity_{ 0.0 };
  geometry_msgs::Pose current_pose_{};
  geometry_msgs::Point next_target_position_{};
  std::vector<autoware_msgs::Waypoint> current_waypoints_{};

  // functions
  double calcCurvature(const geometry_msgs::Point& target) const;
  bool interpolateNextTarget(int next_waypoint, geometry_msgs::Point* next_target) const;
  int getNextWaypoint(const autoware_msgs::Lane& current_path, geometry_msgs::Pose current_pose, int current_index,
                      double lookahead_distance);
};
}  // namespace waypoint_follower

#endif  // PURE_PURSUIT_PURE_PURSUIT_H
