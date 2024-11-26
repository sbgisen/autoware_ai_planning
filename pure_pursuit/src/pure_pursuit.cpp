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
  int target_index = -1;
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
  target_index = std::max(current_index + 1, path_size - 1);
  // Look ahead from current waypoint to find the next point within lookahead_distance_,
  // while ensuring we don't cross a switchback point.
  bool switchback_detected = false;
  for (int i = current_index + 1; i < path_size; i++)
  {
    double current_velocity = current_path.waypoints.at(i).twist.twist.linear.x;
    double next_velocity =
        (i + 1 < path_size) ? current_path.waypoints.at(i + 1).twist.twist.linear.x : current_velocity;

    if (current_velocity * next_velocity < 0)
    {
      // If the velocity changes its sign, the current waypoint is the next waypoint to avoid the case where the vehicle
      // is at the switchback point
      switchback_detected = true;
      target_index = i;
      return target_index;
    }
    if (getPlaneDistance(current_path.waypoints.at(i).pose.pose.position, current_pose.position) > lookahead_distance)
    {
      target_index = i;
      return target_index;
    }
  }
  if (!switchback_detected)
  {
    // Reached end of path; setting next waypoint to last index
    return target_index;
  }
  ROS_WARN("Failed to update target index. Unknown error.");
  return -1;
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
  double target_velocity = 0;
  bool target_velocity_is_valid = getCurrentCommandVelocity(target_velocity);
  if (!target_velocity_is_valid)
  {
    // Target velocity is wrong
    // Recover by setting the curvature to the maximum value and the velocity to a small value
    geometry_msgs::Pose target_pose = current_waypoints_.at(target_waypoint_index_).pose.pose;
    geometry_msgs::Pose relative_target_pose = getRelativePose(current_pose_, target_pose);
    if (recovery_rotate_direction_ == 0)
    {
      if (relative_target_pose.position.y > 0)
      {
        recovery_rotate_direction_ = 1;
      }
      else
      {
        recovery_rotate_direction_ = -1;
      }
    }
    output_kappa = recovery_rotate_direction_ / RADIUS_MIN_;
    output_velocity = RECOVERY_VEL;
    return true;
  }
  else
  {
    recovery_rotate_direction_ = 0;
  }
  output_velocity = target_velocity;
  next_target_position_ = current_waypoints_.at(target_waypoint_index_).pose.pose.position;
  // Verify if curvature can be calculated based on lookahead distance
  if (getPlaneDistance(next_target_position_, current_pose_.position) < minimum_lookahead_distance_)
  {
    // No valid points beyond lookahead distance -> Creating virtual target
    double additional_distance =
        minimum_lookahead_distance_ - getPlaneDistance(next_target_position_, current_pose_.position);
    // Create a virtual target based on the current target
    // Get normalized direction vector
    tf::Vector3 direction;
    double yaw = getYawFromPath(current_lane, target_waypoint_index_);
    double target_waypoint_velocity = current_waypoints_.at(target_waypoint_index_).twist.twist.linear.x;
    double vel_direction = target_waypoint_velocity > 0 ? 1.0 : -1.0;
    direction = tf::Vector3(vel_direction * cos(yaw), vel_direction * sin(yaw), 0.0);
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
  else if (!is_linear_interpolation_)
  {
    return true;
  }

  // Perform linear interpolation for the next target
  const bool interpolation = interpolateNextTarget(target_waypoint_index_, &next_target_position_);
  if (!interpolation)
  {
    // return false;
  }
  return true;
}

bool PurePursuit::getCurrentCommandVelocity(double& output_velocity, autoware_msgs::Lane current_waypoint,
                                            int current_index, int target_index, geometry_msgs::Pose current_pose)
{
  if (current_index < 0 || current_index >= static_cast<int>(current_waypoints_.size()))
  {
    // ROS_WARN("Current waypoint index is out of range");
    output_velocity = 0;
    return false;
  }
  double current_vel = current_waypoints_.at(current_waypoint_index_).twist.twist.linear.x;
  double prev_vel = current_vel;
  if (current_waypoint_index_ > 0)
  {
    prev_vel = current_waypoints_.at(current_waypoint_index_ - 1).twist.twist.linear.x;
  }
  if (current_vel == 0 && prev_vel == 0)
  {
    // Current velocity is zero
    output_velocity = 0;
    return true;
  }
  else if (target_waypoint_index_ == current_waypoint_index_)
  {
    // Current waypoint index is the same as target waypoint index
    output_velocity = current_vel;
    return true;
  }
  else if (target_waypoint_index_ < current_waypoint_index_)
  {
    // Target waypoint index is behind current waypoint index
    output_velocity = 0;
    return false;
  }
  for (int i = current_waypoint_index_; i < target_waypoint_index_; i++)
  {
    geometry_msgs::Pose relative_pose = getRelativePose(current_pose_, current_waypoints_.at(i).pose.pose);
    current_vel = current_waypoints_.at(i).twist.twist.linear.x;

    if (relative_pose.position.x * current_vel > 0)
    {
      output_velocity = current_vel;
      return true;
    }
    else if (relative_pose.position.x * prev_vel > 0)
    {
      output_velocity = prev_vel;
      return true;
    }
    prev_vel = current_vel;
  }
  output_velocity = 0;
  return false;
}
}  // namespace waypoint_follower
