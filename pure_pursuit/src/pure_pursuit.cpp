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

#include <pure_pursuit/pure_pursuit.h>
#include <cmath>
#include "libwaypoint_follower/libwaypoint_follower.h"
#include "ros/console.h"

namespace waypoint_follower
{
// Simple estimation of curvature given two points.
// 1. Convert the target point from map frame into the current pose frame,
//    so it has a local coorinates of (pt.x, pt.y, pt.z).
// 2. If we think it is a cirle with a curvature kappa passing the two points,
//    kappa = 2 * pt.y / (pt.x * pt.x + pt.y * pt.y). For detailed derivation, please
//    refer to "Integrated Mobile Robot Control" by Omead Amidi
//    (CMU-RI-TR-90-17, Equation 3.10 on Page 21)
double PurePursuit::calcCurvature(const geometry_msgs::Point& target) const
{
  double kappa;
  const geometry_msgs::Point pt = calcRelativeCoordinate(target, current_pose_);
  const double denominator = pt.x * pt.x + pt.y * pt.y;
  const double numerator = 2.0 * pt.y;

  if (denominator != 0.0)
  {
    kappa = numerator / denominator;
  }
  else
  {
    kappa = numerator > 0.0 ? (1.0 / RADIUS_MAX_) : -(1.0 / RADIUS_MAX_);
  }
  kappa = std::max(std::min(kappa, 1.0 / RADIUS_MIN_), -1.0 / RADIUS_MIN_);
  return kappa;
}

// Interpolate the path based on the current pose and the next waypoint.
bool PurePursuit::interpolateNextTarget(int next_waypoint, geometry_msgs::Point* next_target) const
{
  const int path_size = static_cast<int>(current_waypoints_.size());
  if (path_size == 0 || next_waypoint < 0 || next_waypoint >= path_size)
  {
    // current_waypoints_ is empty
    return false;
  }
  if (next_waypoint == path_size - 1)
  {
    *next_target = current_waypoints_.back().pose.pose.position;
    return true;
  }
  const double search_radius = lookahead_distance_;
  const geometry_msgs::Point end = current_waypoints_.at(next_waypoint).pose.pose.position;
  const geometry_msgs::Point start = current_waypoints_.at(next_waypoint - 1).pose.pose.position;

  // Project ego vehicle's current position at C onto the line at D in between two waypoints A and B.
  const tf::Vector3 p_A(start.x, start.y, 0.0);
  const tf::Vector3 p_B(end.x, end.y, 0.0);
  const tf::Vector3 p_C(current_pose_.position.x, current_pose_.position.y, 0.0);
  const tf::Vector3 AB = p_B - p_A;
  const tf::Vector3 AC = p_C - p_A;
  const tf::Vector3 p_D = p_A + AC.dot(AB) / AB.dot(AB) * AB;
  const double dist_CD = (p_D - p_C).length();

  bool found = false;
  tf::Vector3 final_goal;
  // Draw a circle centered at p_C with a radius of search_radius
  if (dist_CD > search_radius)
  {
    // no intersection in between the circle and AB
    found = false;
  }
  else if (dist_CD == search_radius)
  {
    // one intersection
    final_goal = p_D;
    found = true;
  }
  else
  {
    // two intersections
    // get intersection in front of vehicle
    double s = sqrt(pow(search_radius, 2) - pow(dist_CD, 2));
    tf::Vector3 p_E = p_D + s * AB.normalized();
    tf::Vector3 p_F = p_D - s * AB.normalized();

    // verify whether these two points lie on line segment AB
    if ((p_B - p_E).length2() < AB.length2())
    {
      final_goal = p_E;
      found = true;
    }
    else if ((p_B - p_F).length2() < AB.length2())
    {
      final_goal = p_F;
      found = true;
    }
  }

  if (found)
  {
    next_target->x = final_goal.x();
    next_target->y = final_goal.y();
    next_target->z = current_pose_.position.z;
  }

  return found;
}

int PurePursuit::getTargetIndex(const autoware_msgs::Lane& current_path, geometry_msgs::Pose current_pose,
                                int current_index, double lookahead_distance)
{
  const int path_size = static_cast<int>(current_path.waypoints.size());
  if (path_size == 0)
  {
    // Current_path is empty
    return -1;
  }
  else if (current_index < 0 || current_index > path_size - 1)
  {
    // Current_index is out of range
    return -1;
  }
  else if (current_index == path_size - 1)
  {
    // Current_index is the last waypoint
    return current_index;
  }
  // Set virtual target pose based on the current pose and the lookahead_distance
  geometry_msgs::Pose lookahead_pose = current_pose;
  double look_yaw = tf::getYaw(current_pose.orientation);
  double vel_sign = current_path.waypoints.at(current_index).twist.twist.linear.x < 0 ? -1.0 : 1.0;
  tf::Vector3 look_vector(lookahead_distance * cos(look_yaw), lookahead_distance * sin(look_yaw), 0);
  lookahead_pose.position.x = current_pose.position.x + vel_sign * look_vector.x();
  lookahead_pose.position.y = current_pose.position.y + vel_sign * look_vector.y();
  int target_index = updateCurrentIndex(current_path, lookahead_pose, current_index + 1);
  return target_index;
}

bool PurePursuit::canGetCurvature(double& output_kappa, double& output_velocity)
{
  autoware_msgs::Lane current_lane;
  current_lane.waypoints = current_waypoints_;

  // Calculate the size of the path
  const int path_size = static_cast<int>(current_waypoints_.size());

  // Get the updated indices
  current_waypoint_index_ = 0;  // updateCurrentIndex(current_lane, current_pose_, current_waypoint_index_);

  if (current_waypoint_index_ < 0 || current_waypoint_index_ > path_size - 1)
  {
    // Current waypoint index is out of range
    return false;
  }
  else if (current_waypoint_index_ == path_size - 1)
  {
    // Current waypoint index is the last waypoint
    output_kappa = 1.0 / RADIUS_MAX_;
    output_velocity = 0;
    return true;
  }

  target_waypoint_index_ = getTargetIndex(current_lane, current_pose_, current_waypoint_index_, lookahead_distance_);
  if (target_waypoint_index_ < 0 || target_waypoint_index_ >= path_size)
  {
    // Target waypoint index is out of range
    return false;
  }

  // Check target velocity
  output_velocity = getCurrentCommandVelocity(current_lane, current_waypoint_index_, current_pose_);
  next_target_position_ = current_waypoints_.at(target_waypoint_index_).pose.pose.position;
  // Verify if curvature can be calculated based on lookahead distance
  if (getPlaneDistance(next_target_position_, current_pose_.position) < minimum_lookahead_distance_)
  {
    // No valid points beyond lookahead distance -> Creating virtual target
    double additional_distance =
        minimum_lookahead_distance_ -
        std::max(minimum_lookahead_distance_, getPlaneDistance(next_target_position_, current_pose_.position));
    // Create a virtual target based on the current target
    // Get normalized direction vector
    geometry_msgs::Pose target_pose = current_waypoints_.at(target_waypoint_index_).pose.pose;
    geometry_msgs::Pose relative_target_pose = getRelativePose(current_pose_, target_pose);
    double vel_sign = relative_target_pose.position.x > 0 ? 1.0 : -1.0;
    double yaw = getYawFromPath(current_lane, target_waypoint_index_);
    double relative_target_yaw = tf::getYaw(relative_target_pose.orientation);
    if (fabs(relative_target_yaw) > M_PI * 0.5)
    {
      vel_sign *= -1.0;
    }
    tf::Vector3 direction = tf::Vector3(vel_sign * cos(yaw), vel_sign * sin(yaw), 0.0);
    next_target_position_.x += additional_distance * direction.x();
    next_target_position_.y += additional_distance * direction.y();
  }
  output_kappa = calcCurvature(next_target_position_);

  // Return true if the curvature can be calculated
  if (target_waypoint_index_ == 0 || target_waypoint_index_ == path_size - 1 ||
      target_waypoint_index_ == current_waypoint_index_)
  {
    return false;
  }
  return true;
}

double PurePursuit::getCurrentCommandVelocity(autoware_msgs::Lane current_waypoint, int current_index,
                                              geometry_msgs::Pose current_pose)
{
  int prev_index = std::max(0, current_index - 1);
  int next_index = std::min(static_cast<int>(current_waypoint.waypoints.size() - 1), current_index + 1);
  geometry_msgs::Pose current_waypoint_pose = current_waypoint.waypoints.at(current_index).pose.pose;
  geometry_msgs::Pose current_waypoint_pose_relative = getRelativePose(current_pose, current_waypoint_pose);
  double prev_waypoint_velocity = current_waypoint.waypoints.at(prev_index).twist.twist.linear.x;
  double current_waypoint_velocity = current_waypoint.waypoints.at(current_index).twist.twist.linear.x;
  double next_waypoint_velocity = current_waypoint.waypoints.at(next_index).twist.twist.linear.x;
  double current_waypoint_distance = getPlaneDistance(current_waypoint_pose.position, current_pose.position);
  if (current_waypoint_pose_relative.position.x * current_waypoint_velocity > 0 &&
      next_waypoint_velocity * current_waypoint_velocity > 0)
  {
    geometry_msgs::Pose next_waypoint_pose = current_waypoint.waypoints.at(next_index).pose.pose;
    double next_waypoint_distance = getPlaneDistance(next_waypoint_pose.position, current_pose.position);
    double target_velocity =
        current_waypoint_velocity * (next_waypoint_distance / (current_waypoint_distance + next_waypoint_distance)) +
        next_waypoint_velocity * (current_waypoint_distance / (current_waypoint_distance + next_waypoint_distance));
    return target_velocity;
  }
  else if (current_waypoint_pose_relative.position.x * current_waypoint_velocity < 0 &&
           prev_waypoint_velocity * current_waypoint_velocity > 0)
  {
    geometry_msgs::Pose prev_waypoint_pose = current_waypoint.waypoints.at(prev_index).pose.pose;
    geometry_msgs::Pose prev_waypoint_pose_relative = getRelativePose(current_pose, prev_waypoint_pose);
    double prev_waypoint_distance = getPlaneDistance(prev_waypoint_pose.position, current_pose.position);
    double target_velocity =
        prev_waypoint_velocity * (current_waypoint_distance / (prev_waypoint_distance + current_waypoint_distance)) +
        current_waypoint_velocity * (prev_waypoint_distance / (prev_waypoint_distance + current_waypoint_distance));
    return target_velocity;
  }
  return current_waypoint_velocity;
}
}  // namespace waypoint_follower
