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

#include "waypoint_planner/astar_avoid/astar_avoid.h"
#include "amathutils_lib/amathutils.hpp"
#include "libwaypoint_follower/libwaypoint_follower.h"
#include "ros/console.h"

AstarAvoid::AstarAvoid() : nh_(), private_nh_("~")
{
  private_nh_.param<int>("safety_waypoints_size", safety_waypoints_size_, 100);
  private_nh_.param<double>("update_rate", update_rate_, 10.0);

  private_nh_.param<bool>("enable_avoidance", enable_avoidance_, false);
  private_nh_.param<bool>("use_back", use_back_, true);
  private_nh_.param<double>("avoid_waypoints_velocity", avoid_waypoints_velocity_, 10.0);
  private_nh_.param<double>("avoid_start_velocity", avoid_start_velocity_, 5.0);
  private_nh_.param<double>("replan_interval", replan_interval_, 2.0);
  private_nh_.param<int>("search_waypoints_size", search_waypoints_size_, 50);
  private_nh_.param<int>("search_waypoints_delta", search_waypoints_delta_, 2);
  private_nh_.param<int>("closest_search_size", closest_search_size_, 30);
  private_nh_.param<int>("stopline_ahead_num", stopline_ahead_num_, 1);
  private_nh_.param<double>("decel_limit", decel_limit_, 0.3);

  safety_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("safety_waypoints", 1, true);
  debug_pub_ = nh_.advertise<nav_msgs::Path>("debug", 1, true);
  costmap_sub_ = nh_.subscribe("costmap", 1, &AstarAvoid::costmapCallback, this);
  current_pose_sub_ = nh_.subscribe("current_pose", 1, &AstarAvoid::currentPoseCallback, this);
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &AstarAvoid::currentVelocityCallback, this);
  base_waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &AstarAvoid::baseWaypointsCallback, this);
  closest_waypoint_sub_ = nh_.subscribe("closest_waypoint", 1, &AstarAvoid::closestWaypointCallback, this);
  obstacle_waypoint_sub_ = nh_.subscribe("obstacle_waypoint", 1, &AstarAvoid::obstacleWaypointCallback, this);

  rate_ = new ros::Rate(update_rate_);
}

void AstarAvoid::costmapCallback(const nav_msgs::OccupancyGrid& msg)
{
  costmap_ = msg;
  tf::poseMsgToTF(costmap_.info.origin, local2costmap_);
  costmap_initialized_ = true;
}

void AstarAvoid::currentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  current_pose_global_ = msg;

  if (!enable_avoidance_)
  {
    current_pose_initialized_ = true;
  }
  else
  {
    current_pose_local_.pose = transformPose(
        current_pose_global_.pose, getTransform(costmap_.header.frame_id, current_pose_global_.header.frame_id));
    current_pose_local_.header.frame_id = costmap_.header.frame_id;
    current_pose_local_.header.stamp = current_pose_global_.header.stamp;
    current_pose_initialized_ = true;
  }
}

void AstarAvoid::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
{
  current_velocity_ = msg;
  current_velocity_initialized_ = true;
}

void AstarAvoid::baseWaypointsCallback(const autoware_msgs::Lane& msg)
{
  base_waypoints_ = msg;
  base_waypoints_initialized_ = true;
}

void AstarAvoid::closestWaypointCallback(const std_msgs::Int32& msg)
{
  base_index_ = msg.data;
  base_index_initialized_ = true;
}

void AstarAvoid::obstacleWaypointCallback(const std_msgs::Int32& msg)
{
  obstacle_index_ = msg.data;
}

void AstarAvoid::run()
{
  // check topics
  while (ros::ok())
  {
    ros::spinOnce();
    if (checkInitialized())
    {
      break;
    }
    ros::Duration(1.0).sleep();
  }

  // main loop
  ros::WallTime start_plan_time = ros::WallTime::now();
  ros::WallTime start_avoid_time = ros::WallTime::now();

  // reset obstacle index
  obstacle_index_ = -1;

  // relaying mode at startup
  state_ = AstarAvoid::STATE::RELAYING;
  select_way_ = AstarAvoid::STATE::RELAYING;

  // Kick off a timer to publish final waypoints
  timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &AstarAvoid::publishWaypoints, this);

  while (ros::ok())
  {
    ros::spinOnce();

    // relay mode
    if (!enable_avoidance_)
    {
      rate_->sleep();
      continue;
    }

    // avoidance mode
    bool found_obstacle = (obstacle_index_ >= 0);
    bool avoid_velocity = (fabs(current_velocity_.twist.linear.x) < avoid_start_velocity_ / 3.6);

    // update state
    if (state_ == AstarAvoid::STATE::RELAYING)
    {
      select_way_ = AstarAvoid::STATE::RELAYING;
      if (found_obstacle)
      {
        ROS_INFO("RELAYING -> STOPPING, Decelerate for stopping");
        state_ = AstarAvoid::STATE::STOPPING;
      }
    }
    else if (state_ == AstarAvoid::STATE::STOPPING)
    {
      bool replan = ((ros::WallTime::now() - start_plan_time).toSec() > replan_interval_);

      if (!found_obstacle)
      {
        if (select_way_ == AstarAvoid::STATE::AVOIDING)
        {
          ROS_INFO("STOPPING -> AVOIDING, Obstacle disappers");
          state_ = AstarAvoid::STATE::AVOIDING;
        }
        else
        {
          ROS_INFO("STOPPING -> RELAYING, Obstacle disappers");
          state_ = AstarAvoid::STATE::RELAYING;
        }
      }
      else if (replan && avoid_velocity)
      {
        ROS_INFO("STOPPING -> PLANNING, Start A* planning");
        state_ = AstarAvoid::STATE::PLANNING;
        select_way_ = AstarAvoid::STATE::RELAYING;
      }
    }
    else if (state_ == AstarAvoid::STATE::PLANNING)
    {
      start_plan_time = ros::WallTime::now();
      if (planAvoidWaypoints(avoid_path_size_))
      {
        ROS_INFO("PLANNING -> AVOIDING, Found path");
        state_ = AstarAvoid::STATE::AVOIDING;
        select_way_ = AstarAvoid::STATE::AVOIDING;
        start_avoid_time = ros::WallTime::now();
      }
      else
      {
        ROS_INFO("PLANNING -> STOPPING, Cannot find path");
        state_ = AstarAvoid::STATE::STOPPING;
        select_way_ = AstarAvoid::STATE::RELAYING;
        avoid_index_ = -1;
      }
    }
    else if (state_ == AstarAvoid::STATE::AVOIDING)
    {
      avoid_index_ = updateCurrentIndex(avoid_waypoints_, current_pose_global_.pose, avoid_index_);
      // ROS_INFO("avoid_index_ = %d", avoid_index_);
      // Check if goal reached
      if (avoid_index_ >= avoid_path_size_)
      {
        ROS_INFO("AVOIDING -> RELAYING, Reached goal");
        state_ = AstarAvoid::STATE::RELAYING;
        select_way_ = AstarAvoid::STATE::RELAYING;
        avoid_index_ = -1;
      }
      else
      {
        if (found_obstacle && avoid_velocity)
        {
          bool replan = ((ros::WallTime::now() - start_avoid_time).toSec() > replan_interval_);
          if (replan)
          {
            ROS_INFO("AVOIDING -> STOPPING, Abort avoiding");
            state_ = AstarAvoid::STATE::STOPPING;
          }
        }
      }
    }
    rate_->sleep();
  }
}

bool AstarAvoid::checkInitialized()
{
  // check for relay mode
  bool initialized = current_pose_initialized_ && base_index_initialized_ && base_waypoints_initialized_;

  if (!initialized)
  {
    if (!current_pose_initialized_)
    {
      ROS_WARN_THROTTLE(5, "Waiting for current_pose topic ...");
    }
    if (!base_index_initialized_)
    {
      ROS_WARN_THROTTLE(5, "Waiting for closest_waypoint topic ...");
    }
    if (!base_waypoints_initialized_)
    {
      ROS_WARN_THROTTLE(5, "Waiting for base_waypoints topic ...");
    }
  }

  // check for avoidance mode, additionally
  if (enable_avoidance_)
  {
    initialized = initialized && current_velocity_initialized_ && costmap_initialized_;

    if (!initialized)
    {
      if (!current_velocity_initialized_)
      {
        ROS_WARN_THROTTLE(5, "Waiting for current_velocity topic ...");
      }
      if (!costmap_initialized_)
      {
        ROS_WARN_THROTTLE(5, "Waiting for costmap topic ...");
      }
    }
  }

  return initialized;
}

bool AstarAvoid::planAvoidWaypoints(int& end_of_avoid_index)
{
  bool found_path = false;

  if (base_index_ < 0 || base_index_ >= static_cast<int>(base_waypoints_.waypoints.size()))
  {
    ROS_ERROR("Invalid base_index_ = %d", base_index_);
    return false;
  }

  auto it =
      base_index_ + obstacle_index_ + stopline_ahead_num_ + 1 > static_cast<int>(base_waypoints_.waypoints.size()) ?
          base_waypoints_.waypoints.end() :
          base_waypoints_.waypoints.begin() + base_index_ + obstacle_index_ + stopline_ahead_num_ + 1;
  if (std::find_if(base_waypoints_.waypoints.begin() + base_index_, it, [](const autoware_msgs::Waypoint& wp) {
        return wp.wpstate.stop_state == autoware_msgs::WaypointState::TYPE_STOPLINE;
      }) != it)
  {
    return false;
  }
  // update goal pose incrementally and execute A* search
  std::vector<geometry_msgs::Pose> goal_poses;
  std::vector<int> goal_indices;

  for (int i = closest_search_size_; i < static_cast<int>(search_waypoints_size_); i += search_waypoints_delta_)
  {
    // update goal index
    // Note: obstacle_index_ is supposed to be relative to base_waypoint_index_.
    //       However, obstacle_index_ is published by velocity_set node. The astar_avoid and velocity_set
    //       should be combined together to prevent this kind of inconsistency.
    int goal_index = base_index_ + obstacle_index_ + i;
    if (goal_index >= static_cast<int>(base_waypoints_.waypoints.size()))
    {
      break;
    }
    auto it2 = goal_index + stopline_ahead_num_ + 1 > static_cast<int>(base_waypoints_.waypoints.size()) ?
                   base_waypoints_.waypoints.end() :
                   base_waypoints_.waypoints.begin() + goal_index + stopline_ahead_num_ + 1;
    auto result = std::find_if(base_waypoints_.waypoints.begin() + goal_index - search_waypoints_delta_, it2,
                               [](const autoware_msgs::Waypoint& wp) {
                                 return wp.wpstate.stop_state == autoware_msgs::WaypointState::TYPE_STOPLINE;
                               });
    if (result != it2)
    {
      break;
    }
    // update goal pose
    goal_pose_global_ = base_waypoints_.waypoints[goal_index].pose;
    goal_pose_local_.header = costmap_.header;
    goal_pose_local_.pose = transformPose(goal_pose_global_.pose,
                                          getTransform(costmap_.header.frame_id, goal_pose_global_.header.frame_id));
    goal_poses.push_back(goal_pose_local_.pose);
    goal_indices.push_back(goal_index);
  }

  if (goal_poses.empty())
  {
    ROS_ERROR("Can't find goal. base_index_ = %d, obstacle_index_ = %d, stopline_ahead_num_ = %d, base_size = %lu",
              base_index_, obstacle_index_, stopline_ahead_num_, base_waypoints_.waypoints.size());
    return false;
  }

  // initialize costmap for A* search
  astar_.initialize(costmap_);

  // execute astar search
  avoid_start_base_index_ = base_index_;
  found_path = astar_.makePlan(current_pose_local_.pose, goal_poses);

  if (found_path && !astar_.getPath().poses.empty())
  {
    debug_pub_.publish(astar_.getPath());
    // Get reached goal index
    avoid_finish_base_index_ = goal_indices.at(astar_.getGoalIndex());
    mergeAvoidWaypoints(astar_.getPath(), avoid_start_base_index_, avoid_finish_base_index_, end_of_avoid_index);
    if (!avoid_waypoints_.waypoints.empty())
    {
      avoid_index_ = avoid_start_base_index_;
      ROS_INFO("Found GOAL at goal_index = %d, current_index = %d, path_size = %lu", avoid_finish_base_index_,
               avoid_start_base_index_, astar_.getPath().poses.size());
      astar_.reset();
      return true;
    }
    else
    {
      ROS_ERROR("Wrong path detected. goal_index = %d, waypoints size = %lu", avoid_finish_base_index_,
                avoid_waypoints_.waypoints.size());
      found_path = false;
    }
  }

  ROS_ERROR("Can't find goal. Retry. base_index_ = %d, obstacle_index_ = %d, stopline_ahead_num_ = %d, base_size = %lu",
            base_index_, obstacle_index_, stopline_ahead_num_, base_waypoints_.waypoints.size());
  astar_.reset();
  return false;
}

void AstarAvoid::mergeAvoidWaypoints(const nav_msgs::Path& path, const int start_index, const int goal_index,
                                     int& end_of_avoid_index)
{
  int start_index_in = start_index;
  if (goal_index == -1 || goal_index < start_index)
  {
    return;
  }
  else if (start_index == -1)
  {
    start_index_in = 0;
  }

  // add waypoints before start index
  avoid_waypoints_.waypoints.clear();
  for (int i = 0; i < start_index_in; ++i)
  {
    avoid_waypoints_.waypoints.push_back(base_waypoints_.waypoints.at(i));
  }

  // set waypoints for avoiding
  if (use_back_)
  {
    int direction = 1;
    for (int i = 0; i < path.poses.size(); ++i)
    {
      autoware_msgs::Waypoint wp;
      wp.pose.header = base_waypoints_.header;
      geometry_msgs::PoseStamped next_pose = path.poses.at(i);
      // if the next_pose.pose.position.z value is smaller than 0, it means that the path is backward
      direction = (next_pose.pose.position.z < 0) ? -1 : 1;
      next_pose.pose.position.z = 0;
      wp.pose.pose =
          transformPose(next_pose.pose, getTransform(base_waypoints_.header.frame_id, next_pose.header.frame_id));
      wp.pose.pose.position.z = current_pose_global_.pose.position.z;         // height = const
      wp.twist.twist.linear.x = direction * avoid_waypoints_velocity_ / 3.6;  // velocity = const
      avoid_waypoints_.waypoints.push_back(wp);
    }
  }
  else
  {
    for (const auto& pose : path.poses)
    {
      autoware_msgs::Waypoint wp;
      wp.pose.header = base_waypoints_.header;
      wp.pose.pose = transformPose(pose.pose, getTransform(base_waypoints_.header.frame_id, pose.header.frame_id));
      wp.pose.pose.position.z = current_pose_global_.pose.position.z;  // height = const
      wp.twist.twist.linear.x = avoid_waypoints_velocity_ / 3.6;       // velocity = const
      avoid_waypoints_.waypoints.push_back(wp);
    }
  }

  // smoothing connection point ( only deceleration )
  double next_velocity = base_waypoints_.waypoints[goal_index].twist.twist.linear.x;
  if (next_velocity - avoid_waypoints_.waypoints.end()->twist.twist.linear.x < 0)
  {
    auto next_position = base_waypoints_.waypoints[goal_index].pose.pose.position;
    for (auto it = avoid_waypoints_.waypoints.rbegin(); it != avoid_waypoints_.waypoints.rend(); ++it)
    {
      double dist = amathutils::find_distance(next_position, it->pose.pose.position);
      double vel = std::sqrt(std::pow(next_velocity, 2.0) - 2 * -decel_limit_ * dist);
      if (vel > it->twist.twist.linear.x)
      {
        break;
      }
      it->twist.twist.linear.x = vel;
      next_velocity = vel;
      next_position = it->pose.pose.position;
    }
  }

  // add waypoints after goal index
  for (int i = goal_index + 1; i < static_cast<int>(base_waypoints_.waypoints.size()); ++i)
  {
    avoid_waypoints_.waypoints.push_back(base_waypoints_.waypoints.at(i));
  }

  // update index for merged waypoints
  end_of_avoid_index = start_index_in + path.poses.size() + 1;
}

void AstarAvoid::publishWaypoints(const ros::TimerEvent& e)
{
  // select waypoints
  autoware_msgs::Lane current_waypoints;
  int current_index;
  if (select_way_ == AstarAvoid::STATE::AVOIDING)
  {
    current_waypoints = avoid_waypoints_;
    current_index = avoid_index_;
  }
  else
  {
    current_waypoints = base_waypoints_;
    current_index = base_index_;
  }
  if (current_index < 0 || current_index >= static_cast<int>(current_waypoints.waypoints.size()))
  {
    ROS_WARN("Invalid index: %d (between 0 and %d)", current_index,
             static_cast<int>(current_waypoints.waypoints.size()));
    select_way_ = AstarAvoid::STATE::RELAYING;
    state_ = AstarAvoid::STATE::STOPPING;
    return;
  }

  // Create local path starting at closest global waypoint
  autoware_msgs::Lane local_waypoints;
  local_waypoints.header = current_waypoints.header;
  local_waypoints.increment = current_waypoints.increment;
  for (int i = current_index;
       i < current_index + safety_waypoints_size_ && i < static_cast<int>(current_waypoints.waypoints.size()); ++i)
  {
    local_waypoints.waypoints.push_back(current_waypoints.waypoints[i]);
  }

  if (!local_waypoints.waypoints.empty())
  {
    safety_waypoints_pub_.publish(local_waypoints);
  }
  else
  {
    ROS_WARN("No waypoints to publish");
    select_way_ = AstarAvoid::STATE::RELAYING;
    state_ = AstarAvoid::STATE::STOPPING;
  }
}

tf::Transform AstarAvoid::getTransform(const std::string& from, const std::string& to)
{
  tf::StampedTransform stf;
  try
  {
    tf_listener_.lookupTransform(from, to, ros::Time(0), stf);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  return stf;
}
