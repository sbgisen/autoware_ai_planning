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
  private_nh_.param<bool>("prohibit_stopline", prohibit_stopline_, true);
  private_nh_.param<int>("stopline_ahead_num", stopline_ahead_num_, 1);
  private_nh_.param<double>("decel_limit", decel_limit_, 0.1);
  private_nh_.param<double>("accel_limit", accel_limit_, 0.5);
  private_nh_.param<double>("vel_min", vel_min_, 0.72);
  private_nh_.param<int>("max_planning_retry", max_planning_retry_, 10);

  safety_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("safety_waypoints", 1, true);
  debug_pub_ = nh_.advertise<nav_msgs::Path>("debug", 1, true);
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
      planning_retry_count_ = 0;
      avoid_current_merged_index_ =
          updateCurrentIndex(avoid_merged_waypoints_, current_pose_global_.pose, avoid_current_merged_index_);
    }
    else
    {
      planning_retry_count_ += 1;
      if (planning_retry_count_ >= max_planning_retry_)
      {
        ROS_INFO("Plan -> Relay, Cannot find path");
        astar_plan_status_ = AstarAvoid::AsterPlanStatus::FAILURE;
        select_way_ = AstarAvoid::WayType::RELAY;
        is_move_ = false;
        avoid_current_merged_index_ = -1;
        planning_retry_count_ = 0;
      }
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

  if (current_global_index_ < 0 || current_global_index_ >= static_cast<int>(global_waypoints_.waypoints.size()))
  {
    ROS_ERROR("Invalid current_global_index_ = %d", current_global_index_);
    return false;
  }
  if (current_pose_global_.header.frame_id.empty() || costmap_.header.frame_id.empty())
  {
    ROS_ERROR("Invalid frame_id in current_pose(%s) or costmap(%s)", current_pose_global_.header.frame_id.c_str(),
              costmap_.header.frame_id.c_str());
    return false;
  }

  int plan_start_global_index = current_global_index_;

  auto it = plan_start_global_index + obstacle_local_index_ + stopline_ahead_num_ + 1 >
                    static_cast<int>(global_waypoints_.waypoints.size()) ?
                global_waypoints_.waypoints.end() :
                global_waypoints_.waypoints.begin() + plan_start_global_index + obstacle_local_index_ +
                    stopline_ahead_num_ + 1;
  if (prohibit_stopline_)
  {
    if (std::find_if(global_waypoints_.waypoints.begin() + plan_start_global_index, it,
                     [](const autoware_msgs::Waypoint& wp) {
                       return wp.wpstate.stop_state == autoware_msgs::WaypointState::TYPE_STOPLINE;
                     }) != it)
    {
      return false;
    }
  }
  if (plan_start_global_index < 0 || plan_start_global_index >= static_cast<int>(global_waypoints_.waypoints.size()))
  {
    return false;
  }

  // update goal pose incrementally and execute A* search
  std::vector<geometry_msgs::Pose> goal_poses;
  std::vector<int> goal_indices;

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

    if (prohibit_stopline_)
    {
      auto it2 =
          obstacle_global_index + stopline_ahead_num_ + 1 > static_cast<int>(global_waypoints_.waypoints.size()) ?
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
    }

    // update goal pose
    goal_pose_global_ = global_waypoints_.waypoints[obstacle_global_index].pose;
    goal_pose_local_.header = costmap_.header;
    goal_pose_local_.pose = transformPose(goal_pose_global_.pose, tf_global2local_start.inverse());
    goal_poses.push_back(goal_pose_local_.pose);
    goal_indices.push_back(obstacle_global_index);
  }

  if (goal_poses.empty())
  {
    ROS_ERROR("Can't find goal. current_global_index_ = %d, obstacle_local_index_ = %d, global_waypoints_size = %zu",
              current_global_index_, obstacle_local_index_, global_waypoints_.waypoints.size());
    return false;
  }

  // Get transform from base to avoid
  // initialize costmap for A* search
  astar_.initialize(costmap_);

  // execute astar search
  found_path = astar_.makePlan(current_pose_local_.pose, goal_poses);
  if (found_path && !astar_.getPath().poses.empty())
  {
    debug_pub_.publish(astar_.getPath());
    avoid_start_global_index_ = plan_start_global_index;
    // Get reached goal index
    avoid_goal_global_index_ = goal_indices.at(astar_.getGoalIndex());
    avoid_goal_merged_index_ = avoid_start_global_index_ + static_cast<int>(astar_.getPath().poses.size());
    mergeAvoidWaypoints(astar_.getPath(), avoid_start_global_index_, avoid_goal_global_index_, tf_global2local_start);
    if (!avoid_merged_waypoints_.waypoints.empty())
    {
      avoid_current_merged_index_ = avoid_start_global_index_;
      ROS_INFO("Found GOAL at goal_index = %d, current_index = %d, path_size = %zu", avoid_goal_global_index_,
               avoid_start_global_index_, astar_.getPath().poses.size());
      astar_.reset();
      return true;
    }
    else
    {
      ROS_ERROR("Wrong path detected. goal_index = %d, avoid_merged_waypoints_size = %zu", avoid_goal_global_index_,
                avoid_merged_waypoints_.waypoints.size());
      found_path = false;
    }
  }

  ROS_ERROR("Can't find goal. Retry. current_global_index_ = %d, obstacle_local_index_ = %d, global_waypoints_size = "
            "%zu",
            current_global_index_, obstacle_local_index_, global_waypoints_.waypoints.size());
  astar_.reset();
  return false;
}

void AstarAvoid::mergeAvoidWaypoints(const nav_msgs::Path& path, const int start_index, const int goal_index)
{
  tf::Transform global2local = getTransform(global_waypoints_.header.frame_id, path.poses.front().header.frame_id);
  mergeAvoidWaypoints(path, start_index, goal_index, global2local);
}

void AstarAvoid::mergeAvoidWaypoints(const nav_msgs::Path& path, const int start_index, const int goal_index,
                                     tf::Transform global2local)
{
  int start_index_in = start_index;
  if (goal_index == -1 || goal_index < start_index)
    return;
  if (start_index_in == -1)
    start_index_in = 0;

  // add waypoints before start index
  avoid_merged_waypoints_.waypoints.clear();
  for (int i = 0; i < start_index_in; ++i)
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
      wp.pose.pose = transformPose(pose.pose, global2local);
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
      wp.pose.pose = transformPose(pose.pose, global2local);
      wp.pose.pose.position.z = current_pose_global_.pose.position.z;  // height = const
      wp.twist.twist.linear.x = avoid_waypoints_velocity_ / 3.6;       // velocity = const
      avoid_merged_waypoints_.waypoints.push_back(wp);
    }
  }

  // add waypoints after goal index
  for (int i = goal_index + 1; i < static_cast<int>(global_waypoints_.waypoints.size()); ++i)
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
  if (current_index < 0 || current_index >= static_cast<int>(current_waypoints.waypoints.size()))
  {
    ROS_WARN("Invalid index: %d (between 0 and %d)", current_index,
             static_cast<int>(current_waypoints.waypoints.size()));
    astar_plan_status_ = AstarAvoid::AsterPlanStatus::FAILURE;
    select_way_ = AstarAvoid::WayType::RELAY;
    is_move_ = false;
    avoid_current_merged_index_ = -1;
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
    astar_plan_status_ = AstarAvoid::AsterPlanStatus::FAILURE;
    select_way_ = AstarAvoid::WayType::RELAY;
    is_move_ = false;
    avoid_current_merged_index_ = -1;
  }
}

tf::Transform AstarAvoid::getTransform(const std::string& from, const std::string& to)
{
  tf::StampedTransform stf;
  if (from.empty() || to.empty())
  {
    ROS_ERROR("Invalid frame_id: form = %s, to = %s", from.c_str(), to.c_str());
    return stf;
  }
  try
  {
    tf_listener_.lookupTransform(from, to, ros::Time(0), stf);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    ROS_ERROR("Failed to get transform from %s to %s", from.c_str(), to.c_str());
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
