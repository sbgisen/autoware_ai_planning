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

AstarAvoid::AstarAvoid() : nh_(), private_nh_("~")
{
  private_nh_.param<int>("safety_waypoints_size", safety_waypoints_size_, 100);
  private_nh_.param<double>("update_rate", update_rate_, 10.0);

  private_nh_.param<bool>("enable_avoidance", enable_avoidance_, false);
  private_nh_.param<bool>("use_back", use_back_, true);
  private_nh_.param<double>("avoid_waypoints_velocity", avoid_waypoints_velocity_, 10.0);
  private_nh_.param<int>("plan_start_index", plan_start_index_, 100);
  private_nh_.param<double>("replan_interval", replan_interval_, 2.0);
  private_nh_.param<int>("search_waypoints_size", search_waypoints_size_, 50);
  private_nh_.param<int>("search_waypoints_delta", search_waypoints_delta_, 2);
  private_nh_.param<int>("stopline_ahead_num", stopline_ahead_num_, 1);
  private_nh_.param<double>("decel_limit", decel_limit_, 0.1);
  private_nh_.param<double>("accel_limit", accel_limit_, 0.5);
  private_nh_.param<double>("vel_min", vel_min_, 0.72);

  safety_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("safety_waypoints", 1, true);
  costmap_sub_ = nh_.subscribe("costmap", 1, &AstarAvoid::costmapCallback, this);
  current_pose_sub_ = nh_.subscribe("current_pose", 1, &AstarAvoid::currentPoseCallback, this);
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &AstarAvoid::currentVelocityCallback, this);
  global_waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &AstarAvoid::baseWaypointsCallback, this);
  closest_waypoint_sub_ = nh_.subscribe("closest_waypoint", 1, &AstarAvoid::closestIndexCallback, this);
  obstacle_waypoint_sub_ = nh_.subscribe("obstacle_waypoint", 1, &AstarAvoid::obstacleIndexCallback, this);

  rate_ = new ros::Rate(update_rate_);
}

AstarAvoid::~AstarAvoid()
{
  // Join the worker thread if it was started
  if (astar_transition_thread_.joinable())
    astar_transition_thread_.join();
  // Cleanup the rate object
  delete rate_;
  rate_ = nullptr;
}

void AstarAvoid::costmapCallback(const nav_msgs::OccupancyGrid& msg)
{
  costmap_ = msg;
  costmap_initialized_ = true;
  if (!current_pose_global_.header.frame_id.empty() && !costmap_.header.frame_id.empty())
  {
    tf_global2local_ = getTransform(current_pose_global_.header.frame_id, costmap_.header.frame_id);
  }
}

void AstarAvoid::currentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  current_pose_global_ = msg;

  if (!enable_avoidance_)
  {
    current_pose_initialized_ = true;
  }
  else if (!current_pose_global_.header.frame_id.empty() && !costmap_.header.frame_id.empty())
  {
    current_pose_local_.pose = transformPose(current_pose_global_.pose, tf_global2local_.inverse());
    current_pose_local_.header.frame_id = costmap_.header.frame_id;
    current_pose_local_.header.stamp = current_pose_global_.header.stamp;
    current_pose_initialized_ = true;
  }

  // Update waypoint following index
  if (select_way_ == AstarAvoid::WayType::AVOID && !avoid_merged_waypoints_.waypoints.empty())
    avoid_current_merged_index_ =
        updateCurrentIndex(avoid_merged_waypoints_, current_pose_global_.pose, avoid_current_merged_index_);
  if (global_waypoints_initialized_ && !global_waypoints_.waypoints.empty())
    current_global_index_ = updateCurrentIndex(global_waypoints_, current_pose_global_.pose, current_global_index_);
}

void AstarAvoid::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
{
  current_velocity_ = msg;
  current_velocity_initialized_ = true;
}

void AstarAvoid::baseWaypointsCallback(const autoware_msgs::Lane& msg)
{
  global_waypoints_ = msg;
  global_waypoints_initialized_ = true;
}

void AstarAvoid::closestIndexCallback(const std_msgs::Int32& msg)
{
  closest_global_index_ = msg.data;
  closest_global_index_initialized_ = true;
}

void AstarAvoid::obstacleIndexCallback(const std_msgs::Int32& msg)
{
  obstacle_local_index_ = msg.data;
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
  start_avoid_time_ = ros::WallTime::now();

  // reset obstacle index
  obstacle_local_index_ = -1;

  // relaying mode at startup
  astar_plan_status_ = AstarAvoid::AsterPlanStatus::IDLE;
  select_way_ = AstarAvoid::WayType::RELAY;
  is_move_ = false;

  // Kick off a timer to publish final waypoints
  timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &AstarAvoid::publishWaypoints, this);
  astar_transition_thread_ = std::thread(&AstarAvoid::astarAvoidTransitionThread, this);

  while (ros::ok())
  {
    ros::spinOnce();
    rate_->sleep();
  }
}

void AstarAvoid::astarAvoidTransitionThread()
{
  while (ros::ok())
  {
    runAstarAvoidTransition();
    rate_->sleep();
  }
}

void AstarAvoid::runAstarAvoidTransition()
{
  // relay mode
  if (!enable_avoidance_)
  {
    rate_->sleep();
    return;
  }

  // avoidance mode
  bool found_obstacle = (obstacle_local_index_ >= 0);
  bool request_aster_planning = found_obstacle && (obstacle_local_index_ <= search_waypoints_size_);

  // update state
  if ((ros::WallTime::now() - start_avoid_time_).toSec() < replan_interval_)
  {
    obstacle_local_index_ = -1;
    is_move_ = !request_aster_planning;
  }
  else if (request_aster_planning)
  {
    ROS_INFO("Start Plan: Request A* planning");
    if (planAvoidWaypoints())
    {
      ROS_INFO("Plan -> Avoid, Found path");
      astar_plan_status_ = AstarAvoid::AsterPlanStatus::SUCCESS;
      select_way_ = AstarAvoid::WayType::AVOID;
      is_move_ = true;
      obstacle_local_index_ = -1;
      avoid_current_merged_index_ =
          updateCurrentIndex(avoid_merged_waypoints_, current_pose_global_.pose, avoid_current_merged_index_);
    }
    else
    {
      ROS_INFO("Plan -> Relay, Cannot find path");
      astar_plan_status_ = AstarAvoid::AsterPlanStatus::FAILURE;
      select_way_ = AstarAvoid::WayType::RELAY;
      is_move_ = false;
      avoid_current_merged_index_ = -1;
    }
    start_avoid_time_ = ros::WallTime::now();
  }
  // Check if goal reached
  if (select_way_ == AstarAvoid::WayType::AVOID && is_move_)
  {
    if (avoid_current_merged_index_ >= avoid_goal_merged_index_ || current_global_index_ >= avoid_goal_global_index_)
    {
      ROS_INFO("Avoid -> Relay, Reached goal");
      select_way_ = AstarAvoid::WayType::RELAY;
      is_move_ = true;
      avoid_current_merged_index_ = -1;
    }
  }
}

bool AstarAvoid::checkInitialized()
{
  // check for relay mode
  bool initialized = current_pose_initialized_ && closest_global_index_initialized_ && global_waypoints_initialized_;

  if (!initialized)
  {
    if (!current_pose_initialized_)
    {
      ROS_WARN_THROTTLE(5, "Waiting for current_pose topic ...");
    }
    if (!closest_global_index_initialized_)
    {
      ROS_WARN_THROTTLE(5, "Waiting for closest_waypoint topic ...");
    }
    if (!global_waypoints_initialized_)
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

bool AstarAvoid::planAvoidWaypoints()
{
  bool found_path = false;
  tf::Transform tf_global2local_start = tf_global2local_;

  if (current_global_index_ == -1)
  {
    return false;
  }
  int plan_start_global_index = current_global_index_;

  auto it =
      plan_start_global_index + obstacle_local_index_ + stopline_ahead_num_ + 1 > global_waypoints_.waypoints.size() ?
          global_waypoints_.waypoints.end() :
          global_waypoints_.waypoints.begin() + plan_start_global_index + obstacle_local_index_ + stopline_ahead_num_ +
              1;
  if (std::find_if(global_waypoints_.waypoints.begin() + plan_start_global_index, it,
                   [](const autoware_msgs::Waypoint& wp) {
                     return wp.wpstate.stop_state == autoware_msgs::WaypointState::TYPE_STOPLINE;
                   }) != it)
  {
    return false;
  }
  // update goal pose incrementally and execute A* search
  for (int i = search_waypoints_delta_; i < static_cast<int>(search_waypoints_size_); i += search_waypoints_delta_)
  {
    // update goal index
    // Note: obstacle_local_index_ is supposed to be relative to plan_start_global_index.
    //       However, obstacle_local_index_ is published by velocity_set node. The astar_avoid and velocity_set
    //       should be combined together to prevent this kind of inconsistency.
    int obstacle_global_index = plan_start_global_index + obstacle_local_index_ + i;
    if (obstacle_global_index >= static_cast<int>(global_waypoints_.waypoints.size()))
    {
      break;
    }

    auto it2 = obstacle_global_index + stopline_ahead_num_ + 1 > global_waypoints_.waypoints.size() ?
                   global_waypoints_.waypoints.end() :
                   global_waypoints_.waypoints.begin() + obstacle_global_index + stopline_ahead_num_ + 1;
    auto result = std::find_if(global_waypoints_.waypoints.begin() + obstacle_global_index - search_waypoints_delta_,
                               it2, [](const autoware_msgs::Waypoint& wp) {
                                 return wp.wpstate.stop_state == autoware_msgs::WaypointState::TYPE_STOPLINE;
                               });
    if (result != it2)
    {
      break;
    }

    // update goal pose
    goal_pose_global_ = global_waypoints_.waypoints[obstacle_global_index].pose;
    goal_pose_local_.header = costmap_.header;
    goal_pose_local_.pose = transformPose(goal_pose_global_.pose, tf_global2local_start.inverse());

    // initialize costmap for A* search
    astar_.initialize(costmap_);

    // execute astar search
    found_path = astar_.makePlan(current_pose_local_.pose, goal_pose_local_.pose);

    static ros::Publisher pub = nh_.advertise<nav_msgs::Path>("debug", 1, true);

    if (found_path)
    {
      pub.publish(astar_.getPath());
      avoid_start_global_index_ = plan_start_global_index;
      avoid_goal_global_index_ = obstacle_global_index;
      mergeAvoidWaypoints(astar_.getPath(), avoid_start_global_index_, avoid_goal_global_index_);
      if (!avoid_merged_waypoints_.waypoints.empty())
      {
        avoid_current_merged_index_ = avoid_start_global_index_;
        avoid_goal_merged_index_ = avoid_start_global_index_ + astar_.getPath().poses.size();
        ROS_INFO("Found GOAL at avoid_goal_global_index = %d", avoid_goal_global_index_);
        astar_.reset();
        return true;
      }
      else
      {
        found_path = false;
      }
    }
    astar_.reset();
  }

  ROS_ERROR("Can't find goal...");
  return false;
}

void AstarAvoid::mergeAvoidWaypoints(const nav_msgs::Path& path, const int start_index, const int goal_index)
{
  if (start_index == -1 || goal_index == -1)
  {
    return;
  }

  // add waypoints before start index
  avoid_merged_waypoints_.waypoints.clear();
  for (int i = 0; i < start_index; ++i)
  {
    avoid_merged_waypoints_.waypoints.push_back(global_waypoints_.waypoints.at(i));
  }

  // set waypoints for avoiding
  if (use_back_)
  {
    int direction = 1;
    for (const auto& pose : path.poses)
    {
      autoware_msgs::Waypoint wp;
      wp.pose.header = global_waypoints_.header;
      // if the next_pose.pose.position.z value is smaller than 0, it means that the path is backward
      direction = (pose.pose.position.z < 0) ? -1 : 1;
      wp.pose.pose.position.z = 0;
      wp.pose.pose = transformPose(pose.pose, getTransform(global_waypoints_.header.frame_id, pose.header.frame_id));
      wp.pose.pose.position.z = current_pose_global_.pose.position.z;         // height = const
      wp.twist.twist.linear.x = direction * avoid_waypoints_velocity_ / 3.6;  // velocity = const
      avoid_merged_waypoints_.waypoints.push_back(wp);
    }
  }
  else
  {
    for (const auto& pose : path.poses)
    {
      autoware_msgs::Waypoint wp;
      wp.pose.header = global_waypoints_.header;
      wp.pose.pose = transformPose(pose.pose, getTransform(global_waypoints_.header.frame_id, pose.header.frame_id));
      wp.pose.pose.position.z = current_pose_global_.pose.position.z;  // height = const
      wp.twist.twist.linear.x = avoid_waypoints_velocity_ / 3.6;       // velocity = const
      avoid_merged_waypoints_.waypoints.push_back(wp);
    }
  }

  // add waypoints after goal index
  for (int i = goal_index; i < static_cast<int>(global_waypoints_.waypoints.size()); ++i)
  {
    avoid_merged_waypoints_.waypoints.push_back(global_waypoints_.waypoints.at(i));
  }

  // smoothing connection point
  limitPathAccel(avoid_merged_waypoints_, accel_limit_, decel_limit_, (vel_min_ / 3.6));
}

void AstarAvoid::publishWaypoints(const ros::TimerEvent& e)
{
  // select waypoints
  autoware_msgs::Lane current_waypoints;
  int current_index;

  // Update avoiding index
  if (select_way_ == AstarAvoid::WayType::AVOID)
  {
    current_waypoints = avoid_merged_waypoints_;
    current_index = avoid_current_merged_index_;
  }
  else
  {
    current_waypoints = global_waypoints_;
    current_index = current_global_index_;
  }

  if (current_index == -1)
  {
    avoid_current_merged_index_ = -1;
    current_global_index_ = -1;
    return;
  }

  // Create local path starting at closest global waypoint
  autoware_msgs::Lane safety_waypoints;
  safety_waypoints.header = current_waypoints.header;
  safety_waypoints.increment = current_waypoints.increment;

  for (int i = current_index;
       i < current_index + safety_waypoints_size_ && i < static_cast<int>(current_waypoints.waypoints.size()); ++i)
  {
    safety_waypoints.waypoints.push_back(current_waypoints.waypoints[i]);
  }

  if (!safety_waypoints.waypoints.empty())
  {
    safety_waypoints_pub_.publish(safety_waypoints);
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

void AstarAvoid::limitPathAccel(autoware_msgs::Lane& path, double accel, double decel, double vel_min)
{
  if (path.waypoints.size() < 2)
  {
    return;
  }

  // Limit acceleration
  double prev_vel = path.waypoints.front().twist.twist.linear.x;
  for (size_t i = 1; i < path.waypoints.size(); ++i)
  {
    double dist =
        amathutils::find_distance(path.waypoints[i - 1].pose.pose.position, path.waypoints[i].pose.pose.position);

    double travel_time = 1.0;
    if (fabs(prev_vel) > 0.0001)
    {
      travel_time = dist / fabs(prev_vel);
    }

    // Calculate velocity limits based on acceleration
    double current_vel = path.waypoints[i].twist.twist.linear.x;
    double vel_sign = (current_vel < 0) ? -1.0 : 1.0;

    // Clamp the current velocity within the calculated limits
    double vel_limited = current_vel;
    if (vel_sign > 0)
    {
      vel_limited = std::min(vel_limited, prev_vel + accel * travel_time);
      if (vel_limited < vel_min)
      {
        vel_limited = vel_min;
      }
    }
    else
    {
      vel_limited = std::max(vel_limited, prev_vel - accel * travel_time);
      if (vel_limited > -vel_min)
      {
        vel_limited = -vel_min;
      }
    }

    // Only update the velocity if it's above vel_min to prevent unnecessary changes
    if (std::fabs(current_vel) > vel_min)
    {
      path.waypoints[i].twist.twist.linear.x = vel_limited;
    }

    // Update previous velocity for the next iteration
    prev_vel = path.waypoints[i].twist.twist.linear.x;
  }

  // Limit deceleration
  double next_vel = path.waypoints.back().twist.twist.linear.x;
  for (int i = static_cast<int>(path.waypoints.size()) - 2; i >= 0; --i)
  {
    double dist =
        amathutils::find_distance(path.waypoints[i].pose.pose.position, path.waypoints[i + 1].pose.pose.position);

    double travel_time = 1.0;
    if (fabs(next_vel) > 0.0001)
    {
      travel_time = dist / fabs(next_vel);
    }

    // Calculate velocity limits based on deceleration
    double current_vel = path.waypoints[i].twist.twist.linear.x;
    double vel_sign = (current_vel < 0) ? -1.0 : 1.0;

    // Clamp the current velocity within the calculated limits
    double vel_limited = current_vel;
    if (vel_sign > 0)
    {
      vel_limited = std::min(vel_limited, next_vel + decel * travel_time);
      if (vel_limited < vel_min)
      {
        vel_limited = vel_min;
      }
    }
    else
    {
      vel_limited = std::max(vel_limited, next_vel - decel * travel_time);
      if (vel_limited > -vel_min)
      {
        vel_limited = -vel_min;
      }
    }

    // Only update the velocity if it's above vel_min to prevent unnecessary changes
    if (std::fabs(current_vel) > vel_min)
    {
      path.waypoints[i].twist.twist.linear.x = vel_limited;
    }

    // Update next velocity for the next iteration
    next_vel = path.waypoints[i].twist.twist.linear.x;
  }
}
