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

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>

#include <limits>
#include <string>
#include <vector>

#include <waypoint_planner/velocity_set/libvelocity_set.h>
#include <waypoint_planner/velocity_set/velocity_set_info.h>
#include <waypoint_planner/velocity_set/velocity_set_path.h>

constexpr int LOOP_RATE = 10;

static lanelet::LaneletMapPtr g_lanelet_map;
static bool g_loaded_lanelet_map;
lanelet::ConstLanelets g_crosswalk_lanelets;

// set color according to given obstacle
void obstacleColorByKind(const EControl kind, std_msgs::ColorRGBA* color, const double alpha = 0.5)
{
  if (kind == EControl::STOP)
  {
    // red
    color->r = 1.0;
    color->g = 0.0;
    color->b = 0.0;
    color->a = alpha;
  }
  else if (kind == EControl::STOPLINE)
  {
    // blue
    color->r = 0.0;
    color->g = 0.0;
    color->b = 1.0;
    color->a = alpha;
  }
  else if (kind == EControl::DECELERATE)
  {
    // yellow
    color->r = 1.0;
    color->g = 1.0;
    color->b = 0.0;
    color->a = alpha;
  }
  else
  {
    // white
    color->r = 1.0;
    color->g = 1.0;
    color->b = 1.0;
    color->a = alpha;
  }
}

// Display a detected obstacle
void displayObstacle(const EControl& kind, const ObstaclePoints& obstacle_points, const ros::Publisher& obstacle_pub)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  static geometry_msgs::Point prev_obstacle_point;
  if (kind == EControl::STOP || kind == EControl::STOPLINE || kind == EControl::DECELERATE)
  {
    marker.pose.position = obstacle_points.getObstaclePoint(kind);
    prev_obstacle_point = marker.pose.position;
  }
  else  // kind == OTHERS
  {
    marker.pose.position = prev_obstacle_point;
  }
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(0.0);
  marker.pose.orientation = quat;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 2.0;
  marker.lifetime = ros::Duration(0.1);
  marker.frame_locked = true;
  obstacleColorByKind(kind, &marker.color, 0.7);

  obstacle_pub.publish(marker);
}

// returns index of waypoint first crosswalk is detected (within 2 meters) from (if any -1 otherwise)
int findClosestCrosswalk(const lanelet::ConstLanelets& crosswalks, const int closest_waypoint,
                         const autoware_msgs::Lane& lane_msg, const int search_distance,
                         lanelet::ConstLanelets* closest_crosswalks, bool multiple_crosswalk_detection)
{
  int wp_near_crosswalk = -1;

  double find_distance = 2.0;  // meter

  // find near crosswalk
  for (int wpi = closest_waypoint;
       wpi < closest_waypoint + search_distance && wpi < static_cast<int>(lane_msg.waypoints.size()); wpi++)
  {
    geometry_msgs::Point waypoint = lane_msg.waypoints[wpi].pose.pose.position;
    lanelet::BasicPoint2d wp2d(waypoint.x, waypoint.y);
    waypoint.z = 0.0;

    for (const auto& ll : crosswalks)
    {
      if (ll.hasAttribute(lanelet::AttributeName::Subtype))
      {
        lanelet::Attribute attr = ll.attribute(lanelet::AttributeName::Subtype);

        if (attr.value() == lanelet::AttributeValueString::Crosswalk)
        {
          double d = lanelet::geometry::distance2d(ll, wp2d);
          if (d < find_distance)
          {
            if (std::find(closest_crosswalks->begin(), closest_crosswalks->end(), ll) == closest_crosswalks->end())
            {
              closest_crosswalks->push_back(ll);
            }
            if (!multiple_crosswalk_detection)
            {
              wp_near_crosswalk = wpi;

              return wp_near_crosswalk;
            }
            if (wp_near_crosswalk == -1)
              wp_near_crosswalk = wpi;
          }
        }
      }
    }
  }

  return wp_near_crosswalk;
}

// obstacle detection for crosswalk
// return EControl::STOP when there are lidar points in crosswalk
// return EControl::Keep otherwise
EControl crossWalkDetection(const VelocitySetInfo& vs_info, const pcl::PointCloud<pcl::PointXYZ>& points,
                            const lanelet::ConstLanelets& closest_crosswalks, ObstaclePoints* obstacle_points)
{
  for (auto lli = closest_crosswalks.begin(); lli != closest_crosswalks.end(); lli++)
  {
    // get polygon in lidar frame
    lanelet::BasicPolygon2d transformed_poly2d;
    lanelet::BasicPolygon3d poly3d = lli->polygon3d().basicPolygon();

    for (const auto& point : poly3d)
    {
      geometry_msgs::Point point_geom, transformed_point_geom;
      lanelet::utils::conversion::toGeomMsgPt(point, &point_geom);
      transformed_point_geom = calcRelativeCoordinate(point_geom, vs_info.getLocalizerPose());
      lanelet::BasicPoint2d transformed_point2d(transformed_point_geom.x, transformed_point_geom.y);
      transformed_poly2d.push_back(transformed_point2d);
    }

    int stop_count = 0;  // number of points in the detection area
    for (const auto& p : points)
    {
      lanelet::BasicPoint2d p2d(p.x, p.y);
      double distance = lanelet::geometry::distance(transformed_poly2d, p2d);

      if (distance < std::numeric_limits<double>::epsilon())
      {
        stop_count++;
        geometry_msgs::Point point_temp;
        point_temp.x = p.x;
        point_temp.y = p.y;
        point_temp.z = p.z;
        obstacle_points->setStopPoint(calcAbsoluteCoordinate(point_temp, vs_info.getLocalizerPose()));
      }
      if (stop_count > vs_info.getPointsThreshold())
      {
        return EControl::STOP;
      }
    }

    obstacle_points->clearStopPoints();
  }

  return EControl::KEEP;  // find no obstacles
}

bool isPointInRectCurrent2Waypoint(const tf::Vector3 robot2point, const geometry_msgs::Pose robot2goal_pose,
                                   const double robot_length, const double robot_width, const double robot_base2back,
                                   const double margin, const double max_search_range)
{
  constexpr double epsilon = 1e-4;  // near-zero guard

  // Check if the point is in the search range
  if (robot2point.length() > max_search_range)
    return false;

  if (robot2point.x() < 0 && robot2goal_pose.position.x > 0)
    return false;

  // Start pose collision detection
  if (robot2point.x() > -robot_base2back - margin && robot2point.x() < robot_length - robot_base2back + margin &&
      fabs(robot2point.y()) < robot_width * 0.5 + margin)
  {
    // The robot can move in the direction away from the obstacle
    if (robot2point.x() > 0 && fabs(robot2point.y()) < (robot_width * 0.5))
      return (robot2goal_pose.position.x > 0);
    else if (robot2point.x() < 0 && fabs(robot2point.y()) < (robot_width * 0.5))
      return (robot2goal_pose.position.x < 0);
    else if (fabs(robot2point.y()) > (robot_width * 0.5))
      return (robot2point.x() * robot2goal_pose.position.x * robot2point.y() * robot2goal_pose.position.y > 0);
    return true;
  }

  // Goal pose collision detection
  tf::Vector3 robot2goal(robot2goal_pose.position.x, robot2goal_pose.position.y, 0);
  if (robot2goal.length() < epsilon)
    return false;
  tf::Vector3 goal2point_robot_coord = robot2point - robot2goal;
  double robot2goal_direction = normalizeAngle(atan2(robot2goal.y(), robot2goal.x()));
  double goal_yaw = normalizeAngle(robot2goal_direction * 2.0);
  tf::Vector3 goal2point(goal2point_robot_coord.x() * cos(-goal_yaw) - goal2point_robot_coord.y() * sin(-goal_yaw),
                         goal2point_robot_coord.x() * sin(-goal_yaw) + goal2point_robot_coord.y() * cos(-goal_yaw), 0);
  if (goal2point.x() > (-robot_base2back - margin) && goal2point.x() < (robot_length - robot_base2back + margin) &&
      fabs(goal2point.y()) < (robot_width * 0.5 + margin))
    return true;

  // Check if the point is in the robot's trajectory
  // Straight movement
  if (fabs(robot2goal_direction) < epsilon || fabs(robot2goal.y()) < epsilon)
  {
    return fabs(robot2point.y()) < (robot_width * 0.5 + margin) && robot2point.x() < std::max(0.0, robot2goal.x()) &&
           robot2point.x() > std::min(0.0, robot2goal.x());
  }

  // Arc movement
  tf::Vector3 robot2arc_center(0.0, 0.0, 0.0);
  if (robot2goal.y() > 0)
    robot2arc_center.setY((robot2goal.x() * robot2goal.x() + robot2goal.y() * robot2goal.y()) / (2.0 * robot2goal.y()));
  else
    robot2arc_center.setY(-(robot2goal.x() * robot2goal.x() + robot2goal.y() * robot2goal.y()) /
                          (2.0 * robot2goal.y()));
  double robot_outer_radius =
      sqrt(pow(std::max(robot_length - robot_base2back, robot_base2back), 2.0) + pow(robot_width / 2.0, 2.0));
  double zone_outer_radius = fabs(robot2arc_center.y()) + robot_outer_radius + margin;
  double zone_inner_radius = std::max(0.0, fabs(robot2arc_center.y()) - robot_width / 2.0 - margin);
  tf::Vector3 arc_center2point = robot2point - robot2arc_center;
  if (arc_center2point.length() > zone_inner_radius && arc_center2point.length() < zone_outer_radius)
  {
    if (robot2goal.x() > 0)
    {
      if (robot2goal.y() > 0)
      {
        // Forward Left
        if (robot2point.x() > 0 && goal2point.x() < robot_length - robot_base2back + margin &&
            !(robot2point.y() < -robot_width * 0.5 - margin) &&
            !(goal2point.y() > (robot_width * 0.5 + margin) && goal2point.x() > 0))
          return true;
      }
      else
      {
        // Forward Right
        if (robot2point.x() > 0 && goal2point.x() < robot_length - robot_base2back + margin &&
            !(robot2point.y() > robot_width * 0.5 + margin) &&
            !(goal2point.y() < -robot_width * 0.5 - margin && goal2point.x() > 0))
          return true;
      }
    }
    else
    {
      if (robot2goal.y() > 0)
      {
        // Backward Left
        if (robot2point.x() < 0 && goal2point.x() > -robot_base2back - margin &&
            !(robot2point.y() < -robot_width * 0.5 - margin) &&
            !(goal2point.y() > (robot_width * 0.5 + margin) && goal2point.x() < 0))
          return true;
      }
      else
      {
        // Backward Right
        if (robot2point.x() < 0 && goal2point.x() > -robot_base2back - margin &&
            !(robot2point.y() > robot_width * 0.5 + margin) &&
            !(goal2point.y() < -robot_width * 0.5 - margin && goal2point.x() < 0))
          return true;
      }
    }
  }
  return false;
}

bool isPointInRectWaypoint2Waypoint(const tf::Vector3 robot2point, const geometry_msgs::Pose robot2start_pose,
                                    const geometry_msgs::Pose robot2goal_pose, const double robot_length,
                                    const double robot_width, const double robot_base2back, const double margin,
                                    const double max_search_range)
{
  constexpr double epsilon = 1e-4;  // near-zero guard

  // Range gate
  if (robot2point.length() > max_search_range)
    return false;

  tf::Vector3 robot2start(robot2start_pose.position.x, robot2start_pose.position.y, 0);
  tf::Vector3 robot2goal(robot2goal_pose.position.x, robot2goal_pose.position.y, 0);

  double robot2start_yaw = tf::getYaw(robot2start_pose.orientation);
  tf::Vector3 start2goal_robot_coord = robot2goal - robot2start;
  tf::Vector3 start2goal(
      start2goal_robot_coord.x() * cos(-robot2start_yaw) - start2goal_robot_coord.y() * sin(-robot2start_yaw),
      start2goal_robot_coord.x() * sin(-robot2start_yaw) + start2goal_robot_coord.y() * cos(-robot2start_yaw), 0);
  tf::Vector3 start2point_robot_coord = robot2point - robot2start;
  tf::Vector3 start2point(
      start2point_robot_coord.x() * cos(-robot2start_yaw) - start2point_robot_coord.y() * sin(-robot2start_yaw),
      start2point_robot_coord.x() * sin(-robot2start_yaw) + start2point_robot_coord.y() * cos(-robot2start_yaw), 0);

  // If obstacle is behind start and goal is ahead, we can move away
  if (start2point.x() < 0 && start2goal.x() > 0)
    return false;

  // Start pose collision detection (AND)
  if (start2point.x() > (-robot_base2back - margin) && start2point.x() < (robot_length - robot_base2back + margin) &&
      fabs(start2point.y()) < (robot_width * 0.5 + margin))
    return true;

  // Goal pose collision detection
  if (start2goal.length() < epsilon)
    return false;

  double start2goal_direction = normalizeAngle(atan2(start2goal.y(), start2goal.x()));
  double start2goal_yaw = normalizeAngle(start2goal_direction * 2.0);
  double goal_yaw = normalizeAngle(robot2start_yaw + start2goal_yaw);
  tf::Vector3 goal2point_robot_coord = robot2point - robot2goal;
  tf::Vector3 goal2point(goal2point_robot_coord.x() * cos(-goal_yaw) - goal2point_robot_coord.y() * sin(-goal_yaw),
                         goal2point_robot_coord.x() * sin(-goal_yaw) + goal2point_robot_coord.y() * cos(-goal_yaw), 0);
  if (goal2point.x() > (-robot_base2back - margin) && goal2point.x() < (robot_length - robot_base2back + margin) &&
      fabs(goal2point.y()) < (robot_width * 0.5 + margin))
    return true;

  // Check if the point is in the robot's trajectory (START frame)
  // Straight movement
  if (fabs(start2goal_direction) < epsilon || fabs(start2goal.y()) < epsilon)
  {
    return fabs(start2point.y()) < (robot_width * 0.5 + margin) && start2point.x() < std::max(0.0, start2goal.x()) &&
           start2point.x() > std::min(0.0, start2goal.x());
  }

  // Arc movement
  tf::Vector3 start2arc_center(0.0, 0.0, 0.0);
  if (start2goal.y() > 0)
    start2arc_center.setY((start2goal.x() * start2goal.x() + start2goal.y() * start2goal.y()) / (2.0 * start2goal.y()));
  else
    start2arc_center.setY(-(start2goal.x() * start2goal.x() + start2goal.y() * start2goal.y()) /
                          (2.0 * start2goal.y()));
  double robot_outer_radius =
      sqrt(pow(std::max(robot_length - robot_base2back, robot_base2back), 2.0) + pow(robot_width / 2.0, 2.0));
  double zone_outer_radius = fabs(start2arc_center.y()) + robot_outer_radius + margin;
  double zone_inner_radius = std::max(0.0, fabs(start2arc_center.y()) - robot_width / 2.0 - margin);
  tf::Vector3 arc_center2point = start2point - start2arc_center;
  if (arc_center2point.length() > zone_inner_radius && arc_center2point.length() < zone_outer_radius)
  {
    if (start2goal.x() > 0)
    {
      if (start2goal.y() > 0)
      {
        // Forward Left
        if (start2point.x() > 0 && goal2point.x() < robot_length - robot_base2back + margin &&
            !(start2point.y() < -robot_width * 0.5 - margin) &&
            !(goal2point.y() > robot_width * 0.5 + margin && goal2point.x() > 0))
          return true;
      }
      else
      {
        // Forward Right
        if (start2point.x() > 0 && goal2point.x() < robot_length - robot_base2back + margin &&
            !(start2point.y() > robot_width * 0.5 + margin) &&
            !(goal2point.y() < -robot_width * 0.5 - margin && goal2point.x() > 0))
          return true;
      }
    }
    else
    {
      // Backward Right
      if (start2goal.y() < 0)
      {
        if (start2point.x() < 0 && goal2point.x() > -robot_base2back - margin &&
            !(start2point.y() > robot_width * 0.5 + margin) &&
            !(goal2point.y() < -robot_width * 0.5 - margin && goal2point.x() < 0))
          return true;
      }
      // Backward Left
      else
      {
        if (start2point.x() < 0 && goal2point.x() > -robot_base2back - margin &&
            !(start2point.y() < -robot_width * 0.5 - margin) &&
            !(goal2point.y() > robot_width * 0.5 + margin && goal2point.x() < 0))
          return true;
      }
    }
  }
  return false;
}

bool isPointInCurcleCurrent2Waypoint(tf::Vector3 robot2point, const geometry_msgs::Pose robot2goal_pose,
                                     const double robot_radius, const double max_search_range)
{
  constexpr double epsilon = 1e-4;  // near-zero guard
  // Check if the point is in the search range
  if (robot2point.length() > max_search_range)
    return false;

  // The robot can move in the direction away from the obstacle
  if (robot2point.x() < 0 && robot2goal_pose.position.x > 0)
    return false;
  else if (robot2point.length() < robot_radius)
  {
    if (robot2point.x() > 0 && robot2point.y() > 0)
      return (robot2goal_pose.position.x > 0 && robot2goal_pose.position.y > 0);
    else if (robot2point.x() < 0 && robot2point.y() > 0)
      return (robot2goal_pose.position.x < 0 && robot2goal_pose.position.y > 0);
    else if (robot2point.x() < 0 && robot2point.y() < 0)
      return (robot2goal_pose.position.x < 0 && robot2goal_pose.position.y < 0);
    else if (robot2point.x() > 0 && robot2point.y() < 0)
      return (robot2goal_pose.position.x > 0 && robot2goal_pose.position.y < 0);
  }

  // Goal pose collision detection
  tf::Vector3 robot2goal(robot2goal_pose.position.x, robot2goal_pose.position.y, 0);
  if (robot2goal.length() < epsilon)
    return false;
  tf::Vector3 goal2point_robot_coord = robot2point - robot2goal;
  if (goal2point_robot_coord.length() < robot_radius)
    return true;
  double robot2goal_direction = normalizeAngle(atan2(robot2goal.y(), robot2goal.x()));
  double goal_yaw = normalizeAngle(robot2goal_direction * 2.0);
  tf::Vector3 goal2point(goal2point_robot_coord.x() * cos(-goal_yaw) - goal2point_robot_coord.y() * sin(-goal_yaw),
                         goal2point_robot_coord.x() * sin(-goal_yaw) + goal2point_robot_coord.y() * cos(-goal_yaw), 0);
  // Check if the point is in the robot's trajectory
  // Straight movement
  if (fabs(robot2goal_direction) < epsilon || fabs(robot2goal.y()) < epsilon)
  {
    return fabs(robot2point.y()) < robot_radius && robot2point.x() < std::max(0.0, robot2goal.x()) &&
           robot2point.x() > std::min(0.0, robot2goal.x());
  }

  // Arc movement
  // Center of the arc of the robot's trajectory
  tf::Vector3 robot2arc_center(0.0, 0.0, 0.0);
  if (robot2goal.y() > 0)
    robot2arc_center.setY((robot2goal.x() * robot2goal.x() + robot2goal.y() * robot2goal.y()) / (2.0 * robot2goal.y()));
  else
    robot2arc_center.setY(-(robot2goal.x() * robot2goal.x() + robot2goal.y() * robot2goal.y()) /
                          (2.0 * robot2goal.y()));
  // Maximum radius of the arc of the robot's trajectory
  double zone_outer_radius = fabs(robot2arc_center.y()) + robot_radius;
  double zone_inner_radius = std::max(0.0, fabs(robot2arc_center.y()) - robot_radius);
  tf::Vector3 arc_center2point = robot2point - robot2arc_center;
  if (arc_center2point.length() > zone_inner_radius && arc_center2point.length() < zone_outer_radius)
  {
    if (robot2goal.x() > 0)
    {
      if (robot2goal.y() > 0)
      {
        // Forward Left
        if (robot2point.x() > 0 && goal2point.x() < 0)
          return true;
      }
      else
      {
        // Forward Right
        if (robot2point.x() > 0 && goal2point.x() < 0)
          return true;
      }
    }
    else
    {
      if (robot2goal.y() > 0)
      {
        // Backward Left
        if (robot2point.x() < 0 && goal2point.x() > 0)
          return true;
      }
      else
      {
        // Backward Right
        if (robot2point.x() < 0 && goal2point.x() > 0)
          return true;
      }
    }
  }
  return false;
}

bool isPointInCurcleWaypoint2Waypoint(tf::Vector3 robot2point, const geometry_msgs::Pose robot2start_pose,
                                      const geometry_msgs::Pose robot2goal_pose, const double robot_radius,
                                      const double max_search_range)
{
  constexpr double epsilon = 1e-4;  // near-zero guard
  // Check if the point is in the search range
  if (robot2point.length() > max_search_range)
    return false;

  tf::Vector3 robot2start(robot2start_pose.position.x, robot2start_pose.position.y, 0);
  tf::Vector3 robot2goal(robot2goal_pose.position.x, robot2goal_pose.position.y, 0);

  double robot2start_yaw = tf::getYaw(robot2start_pose.orientation);
  tf::Vector3 start2goal_robot_coord = robot2goal - robot2start;
  tf::Vector3 start2goal(
      start2goal_robot_coord.x() * cos(-robot2start_yaw) - start2goal_robot_coord.y() * sin(-robot2start_yaw),
      start2goal_robot_coord.x() * sin(-robot2start_yaw) + start2goal_robot_coord.y() * cos(-robot2start_yaw), 0);
  tf::Vector3 start2point_robot_coord = robot2point - robot2start;
  tf::Vector3 start2point(
      start2point_robot_coord.x() * cos(-robot2start_yaw) - start2point_robot_coord.y() * sin(-robot2start_yaw),
      start2point_robot_coord.x() * sin(-robot2start_yaw) + start2point_robot_coord.y() * cos(-robot2start_yaw), 0);

  // The robot can move in the direction away from the obstacle
  if (start2point.x() < 0 && start2goal.x() > 0)
    return false;

  // Start pose collision detection
  if (start2point.length() < robot_radius)
    return true;
  if (start2goal.length() < epsilon)
    return false;

  double start2goal_direction = normalizeAngle(atan2(start2goal.y(), start2goal.x()));
  double start2goal_yaw = normalizeAngle(start2goal_direction * 2.0);
  double goal_yaw = normalizeAngle(robot2start_yaw + start2goal_yaw);
  tf::Vector3 goal2point_robot_coord = robot2point - robot2goal;
  tf::Vector3 goal2point(goal2point_robot_coord.x() * cos(-goal_yaw) - goal2point_robot_coord.y() * sin(-goal_yaw),
                         goal2point_robot_coord.x() * sin(-goal_yaw) + goal2point_robot_coord.y() * cos(-goal_yaw), 0);
  // Goal pose collision detection
  if (goal2point.length() < robot_radius)
    return true;

  // Check if the point is in the robot's trajectory
  // Straight movement
  if (fabs(start2goal_direction) < epsilon || fabs(start2goal.y()) < epsilon)
  {
    return fabs(start2point.y()) < robot_radius && start2point.x() < std::max(0.0, start2goal.x()) &&
           start2point.x() > std::min(0.0, start2goal.x());
  }

  // Arc movement
  // Center of the arc of the robot's trajectory
  tf::Vector3 start2arc_center(0.0, 0.0, 0.0);
  if (start2goal.y() > 0)
    start2arc_center.setY((start2goal.x() * start2goal.x() + start2goal.y() * start2goal.y()) / (2.0 * start2goal.y()));
  else
    start2arc_center.setY(-(start2goal.x() * start2goal.x() + start2goal.y() * start2goal.y()) /
                          (2.0 * start2goal.y()));
  double zone_outer_radius = fabs(start2arc_center.y()) + robot_radius;
  double zone_inner_radius = fabs(start2arc_center.y()) - robot_radius;
  tf::Vector3 arc_center2point = start2point - start2arc_center;
  if (arc_center2point.length() > zone_inner_radius && arc_center2point.length() < zone_outer_radius)
  {
    if (start2goal.x() > 0)
    {
      if (start2goal.y() > 0)
      {
        // Forward Left
        if (start2point.x() > 0 && goal2point.x() < 0)
          return true;
      }
      else
      {
        // Forward Right
        if (start2point.x() > 0 && goal2point.x() < 0)
          return true;
      }
    }
    else
    {
      if (start2goal.y() > 0)
      {
        // Backward Left
        if (start2point.x() < 0 && goal2point.x() > 0)
          return true;
      }
      else
      {
        // Backward Right
        if (start2point.x() < 0 && goal2point.x() > 0)
          return true;
      }
    }
  }
  return false;
}

// same as velocity_set.cpp - except for no reference to vector maps or crosswalk
int detectStopObstacle(const VelocitySetInfo& vs_info, const pcl::PointCloud<pcl::PointXYZ>& points,
                       const int closest_waypoint, int detection_waypoint, const autoware_msgs::Lane& lane,
                       const lanelet::ConstLanelets& closest_crosswalks, ObstaclePoints* obstacle_points,
                       EObstacleType* obstacle_type, const int stop_search_distance)
{
  int stop_obstacle_waypoint = -1;
  *obstacle_type = EObstacleType::NONE;
  // start search from the closest waypoint
  for (int i = closest_waypoint; i < closest_waypoint + stop_search_distance; i++)
  {
    // reach the end of waypoints
    if (i >= static_cast<int>(lane.waypoints.size()))
      break;

    // detection from other nodes
    if (vs_info.getDetectionResultByOtherNodes() >= 0 &&
        lane.waypoints.at(i).gid == vs_info.getDetectionResultByOtherNodes())
    {
      stop_obstacle_waypoint = i;
      *obstacle_type = EObstacleType::STOPLINE;
      obstacle_points->setStopPoint(lane.waypoints.at(i).pose.pose.position);  // for vizuialization
      break;
    }

    // Detection for cross walk
    if (i == detection_waypoint)
    {
      // found an obstacle in the cross walk
      if (crossWalkDetection(vs_info, points, closest_crosswalks, obstacle_points) == EControl::STOP)
      {
        stop_obstacle_waypoint = i;
        *obstacle_type = EObstacleType::ON_CROSSWALK;
        break;
      }
    }

    // Get the coordinates of the current waypoint
    geometry_msgs::Pose waypoint_pose = getRelativePose(vs_info.getLocalizerPose(), lane.waypoints[i].pose.pose);
    waypoint_pose.position.z = 0;

    // Detect obstacles at the current waypoint
    int stop_point_count = 0;
    for (const auto& p : points)
    {
      tf::Vector3 point_vector(p.x, p.y, 0);
      bool in_collision = false;

      if (vs_info.getUseRobotShape())
      {
        double robot_shape_margin = std::max(0.0, (vs_info.getStopRange() - vs_info.getRobotWidth() * 0.5));
        if (i == closest_waypoint)
        {
          in_collision = isPointInRectCurrent2Waypoint(point_vector, waypoint_pose, vs_info.getRobotLength(),
                                                       vs_info.getRobotWidth(), vs_info.getRobotBase2Back(),
                                                       robot_shape_margin, vs_info.getMaxSearchRange());
        }
        if (i < static_cast<int>(lane.waypoints.size()) - 1 && !in_collision)
        {
          geometry_msgs::Pose next_waypoint_pose =
              getRelativePose(vs_info.getLocalizerPose(), lane.waypoints[i + 1].pose.pose);
          next_waypoint_pose.position.z = 0;
          in_collision = isPointInRectWaypoint2Waypoint(
              point_vector, waypoint_pose, next_waypoint_pose, vs_info.getRobotLength(), vs_info.getRobotWidth(),
              vs_info.getRobotBase2Back(), robot_shape_margin, vs_info.getMaxSearchRange());
        }
      }
      else
      {
        if (i == closest_waypoint)
        {
          in_collision = isPointInCurcleCurrent2Waypoint(point_vector, waypoint_pose, vs_info.getStopRange(),
                                                         vs_info.getMaxSearchRange());
        }
        if (i < static_cast<int>(lane.waypoints.size()) - 1 && !in_collision)
        {
          geometry_msgs::Pose next_waypoint_pose =
              getRelativePose(vs_info.getLocalizerPose(), lane.waypoints[i + 1].pose.pose);
          next_waypoint_pose.position.z = 0;
          in_collision = isPointInCurcleWaypoint2Waypoint(point_vector, waypoint_pose, next_waypoint_pose,
                                                          vs_info.getStopRange(), vs_info.getMaxSearchRange());
        }
      }

      double current_index_vel = lane.waypoints[i].twist.twist.linear.x;
      bool moving_forward = (current_index_vel >= 0.0);
      if (in_collision)
      {
        if ((moving_forward && p.x > 0.0) || (!moving_forward && p.x < 0.0))
        {
          stop_point_count++;
          geometry_msgs::Point point_temp;
          point_temp.x = p.x;
          point_temp.y = p.y;
          point_temp.z = p.z;
          obstacle_points->setStopPoint(calcAbsoluteCoordinate(point_temp, vs_info.getLocalizerPose()));
        }
      }
    }

    // there is an obstacle if the number of points exceeded the threshold
    if (stop_point_count > vs_info.getPointsThreshold())
    {
      stop_obstacle_waypoint = i;
      *obstacle_type = EObstacleType::ON_WAYPOINTS;
      break;
    }

    obstacle_points->clearStopPoints();

    // check next waypoint...
  }

  return stop_obstacle_waypoint;
}

//  same as velocity_set.cpp - expect for no reference to vector maps
int detectDecelerateObstacle(const VelocitySetInfo& vs_info, const pcl::PointCloud<pcl::PointXYZ>& points,
                             const int closest_waypoint, const autoware_msgs::Lane& lane,
                             ObstaclePoints* obstacle_points, const int deceleration_search_distance,
                             const bool disable_side_deceleration)
{
  int decelerate_obstacle_waypoint = -1;
  // start search from the closest waypoint
  for (int i = closest_waypoint; i < closest_waypoint + deceleration_search_distance; i++)
  {
    // reach the end of waypoints
    if (i >= static_cast<int>(lane.waypoints.size()))
      break;

    // Get the coordinates of the current waypoint
    geometry_msgs::Pose waypoint_pose = getRelativePose(vs_info.getLocalizerPose(), lane.waypoints[i].pose.pose);
    waypoint_pose.position.z = 0;

    // Detect obstacles at the current waypoint
    int decelerate_point_count = 0;
    for (const auto& p : points)
    {
      tf::Vector3 point_vector(p.x, p.y, 0);
      bool in_collision = false;

      if (vs_info.getUseRobotShape())
      {
        double robot_shape_margin = std::max(0.0, (vs_info.getDecelerationRange() - vs_info.getRobotWidth() * 0.5));
        if (i == closest_waypoint)
        {
          in_collision = isPointInRectCurrent2Waypoint(point_vector, waypoint_pose, vs_info.getRobotLength(),
                                                       vs_info.getRobotWidth(), vs_info.getRobotBase2Back(),
                                                       robot_shape_margin, vs_info.getMaxSearchRange());
        }
        if (i < static_cast<int>(lane.waypoints.size()) - 1 && !in_collision)
        {
          geometry_msgs::Pose next_waypoint_pose =
              getRelativePose(vs_info.getLocalizerPose(), lane.waypoints[i + 1].pose.pose);
          next_waypoint_pose.position.z = 0;
          in_collision = isPointInRectWaypoint2Waypoint(
              point_vector, waypoint_pose, next_waypoint_pose, vs_info.getRobotLength(), vs_info.getRobotWidth(),
              vs_info.getRobotBase2Back(), robot_shape_margin, vs_info.getMaxSearchRange());
        }
        if (in_collision && disable_side_deceleration)
        {
          if (p.x > -vs_info.getRobotBase2Back() && p.x < vs_info.getRobotLength() - vs_info.getRobotBase2Back())
          {
            in_collision = false;
          }
        }
      }
      else
      {
        if (i == closest_waypoint)
        {
          in_collision = isPointInCurcleCurrent2Waypoint(point_vector, waypoint_pose, vs_info.getDecelerationRange(),
                                                         vs_info.getMaxSearchRange());
        }
        if (i < static_cast<int>(lane.waypoints.size()) - 1 && !in_collision)
        {
          geometry_msgs::Pose next_waypoint_pose =
              getRelativePose(vs_info.getLocalizerPose(), lane.waypoints[i + 1].pose.pose);
          next_waypoint_pose.position.z = 0;
          in_collision = isPointInCurcleWaypoint2Waypoint(point_vector, waypoint_pose, next_waypoint_pose,
                                                          vs_info.getDecelerationRange(), vs_info.getMaxSearchRange());
        }
        if (in_collision && disable_side_deceleration)
        {
          if (p.x > -vs_info.getDecelerationRange() && p.x < vs_info.getDecelerationRange())
          {
            in_collision = false;
          }
        }
      }

      double current_index_vel = lane.waypoints[i].twist.twist.linear.x;
      if (in_collision && current_index_vel * p.x > 0)
      {
        decelerate_point_count++;
        geometry_msgs::Point point_temp;
        point_temp.x = p.x;
        point_temp.y = p.y;
        point_temp.z = p.z;
        obstacle_points->setDeceleratePoint(calcAbsoluteCoordinate(point_temp, vs_info.getLocalizerPose()));
      }
    }

    // there is an obstacle if the number of points exceeded the threshold
    if (decelerate_point_count > vs_info.getPointsThreshold())
    {
      decelerate_obstacle_waypoint = i;
      break;
    }

    obstacle_points->clearDeceleratePoints();

    // check next waypoint...
  }

  return decelerate_obstacle_waypoint;
}

//-------------------------------------------------------------------------
//  Detect an obstacle by using pointcloud
//  same as velocity_set.cpp - no reference to vector maps or crosswalk
//-------------------------------------------------------------------------
EControl pointsDetection(const VelocitySetInfo& vs_info, const int closest_waypoint, const int detection_waypoint,
                         const autoware_msgs::Lane& lane, const lanelet::ConstLanelets& closest_crosswalks,
                         int* obstacle_waypoint, ObstaclePoints* obstacle_points,
                         const int deceleration_search_distance, const int stop_search_distance,
                         const bool disable_side_deceleration)
{
  // no input for detection || no closest waypoint
  if ((vs_info.getPoints().empty() == true && vs_info.getDetectionResultByOtherNodes() == -1) || closest_waypoint < 0)
    return EControl::KEEP;

  EObstacleType obstacle_type = EObstacleType::NONE;
  int stop_obstacle_waypoint =
      detectStopObstacle(vs_info, vs_info.getPoints(), closest_waypoint, detection_waypoint, lane, closest_crosswalks,
                         obstacle_points, &obstacle_type, stop_search_distance);

  // skip searching deceleration range
  if (vs_info.getDecelerationRange() < 0.01)
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    if (stop_obstacle_waypoint < 0)
      return EControl::KEEP;
    else if (obstacle_type == EObstacleType::ON_WAYPOINTS || obstacle_type == EObstacleType::ON_CROSSWALK)
      return EControl::STOP;
    else if (obstacle_type == EObstacleType::STOPLINE)
      return EControl::STOPLINE;
    else
      return EControl::OTHERS;
  }

  int decelerate_obstacle_waypoint =
      detectDecelerateObstacle(vs_info, vs_info.getPoints(), closest_waypoint, lane, obstacle_points,
                               deceleration_search_distance, disable_side_deceleration);

  // stop obstacle was not found
  if (stop_obstacle_waypoint < 0)
  {
    *obstacle_waypoint = decelerate_obstacle_waypoint;
    return decelerate_obstacle_waypoint < 0 ? EControl::KEEP : EControl::DECELERATE;
  }

  // stop obstacle was found but decelerate obstacle was not found
  if (decelerate_obstacle_waypoint < 0)
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    return EControl::STOP;
  }

  // about 5.0 meter
  double waypoint_interval =
      getPlaneDistance(lane.waypoints[0].pose.pose.position, lane.waypoints[1].pose.pose.position);
  int stop_decelerate_threshold = 5 / waypoint_interval;

  // both were found
  if (stop_obstacle_waypoint - decelerate_obstacle_waypoint > stop_decelerate_threshold)
  {
    *obstacle_waypoint = decelerate_obstacle_waypoint;
    return EControl::DECELERATE;
  }
  else
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    return EControl::STOP;
  }
}

// visualization of stoplines, crosswalks, and detection range
void displayDetectionRange(const VelocitySetInfo& vs_info, const autoware_msgs::Lane& lane,
                           const lanelet::ConstLanelets& closest_crosswalks, const int closest_waypoint,
                           const int detection_waypoint, const EControl& kind, const int obstacle_waypoint,
                           const ros::Publisher& detection_range_pub, const int deceleration_search_distance,
                           const int stop_search_distance)
{
  // set up for marker array
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker crosswalk_marker;
  visualization_msgs::Marker waypoint_marker_stop;
  visualization_msgs::Marker waypoint_marker_decelerate;
  visualization_msgs::Marker stop_line;
  crosswalk_marker.header.frame_id = "map";
  crosswalk_marker.header.stamp = ros::Time();
  crosswalk_marker.id = 0;
  crosswalk_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  crosswalk_marker.action = visualization_msgs::Marker::ADD;
  waypoint_marker_stop = crosswalk_marker;
  waypoint_marker_decelerate = crosswalk_marker;
  stop_line = crosswalk_marker;
  stop_line.type = visualization_msgs::Marker::CUBE;

  // set each namespace
  crosswalk_marker.ns = "Crosswalk Detection";
  waypoint_marker_stop.ns = "Stop Detection";
  waypoint_marker_decelerate.ns = "Decelerate Detection";
  stop_line.ns = "Stop Line";

  // set scale and color
  double scale = 2.0 * vs_info.getStopRange();
  waypoint_marker_stop.scale.x = scale;
  waypoint_marker_stop.scale.y = scale;
  waypoint_marker_stop.scale.z = scale;
  waypoint_marker_stop.color.a = 0.2;
  waypoint_marker_stop.color.r = 0.0;
  waypoint_marker_stop.color.g = 1.0;
  waypoint_marker_stop.color.b = 0.0;
  waypoint_marker_stop.frame_locked = true;

  scale = 2.0 * (vs_info.getStopRange() + vs_info.getDecelerationRange());
  waypoint_marker_decelerate.scale.x = scale;
  waypoint_marker_decelerate.scale.y = scale;
  waypoint_marker_decelerate.scale.z = scale;
  waypoint_marker_decelerate.color.a = 0.15;
  waypoint_marker_decelerate.color.r = 1.0;
  waypoint_marker_decelerate.color.g = 1.0;
  waypoint_marker_decelerate.color.b = 0.0;
  waypoint_marker_decelerate.frame_locked = true;

  if (obstacle_waypoint > -1)
  {
    stop_line.pose.position = lane.waypoints[obstacle_waypoint].pose.pose.position;
    stop_line.pose.orientation = lane.waypoints[obstacle_waypoint].pose.pose.orientation;
  }
  stop_line.pose.position.z += 1.0;
  stop_line.scale.x = 0.1;
  stop_line.scale.y = 15.0;
  stop_line.scale.z = 2.0;
  stop_line.lifetime = ros::Duration(0.1);
  stop_line.frame_locked = true;
  obstacleColorByKind(kind, &stop_line.color, 0.3);

  // int crosswalk_id = crosswalk.getDetectionCrossWalkID();
  crosswalk_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  crosswalk_marker.scale.x = 1.0;
  crosswalk_marker.scale.y = 1.0;
  crosswalk_marker.scale.z = 1.0;
  crosswalk_marker.color.a = 0.5;
  crosswalk_marker.color.r = 0.0;
  crosswalk_marker.color.g = 1.0;
  crosswalk_marker.color.b = 0.0;

  for (const auto& crosswalk_ll : closest_crosswalks)
  {
    std::vector<geometry_msgs::Polygon> triangles;
    lanelet::visualization::lanelet2Triangle(crosswalk_ll, &triangles);
    for (const auto& t : triangles)
    {
      for (const auto& p_32 : t.points)
      {
        geometry_msgs::Point pt;
        lanelet::utils::conversion::toGeomMsgPt(p_32, &pt);
        crosswalk_marker.points.push_back(pt);
      }
    }
  }

  // inverse to correct direction of triangle
  std::reverse(crosswalk_marker.points.begin(), crosswalk_marker.points.end());
  crosswalk_marker.frame_locked = true;

  // set marker points coordinate
  for (int i = 0; i < stop_search_distance; i++)
  {
    if (closest_waypoint < 0 || i + closest_waypoint > static_cast<int>(lane.waypoints.size()) - 1)
      break;

    geometry_msgs::Point point;
    point = lane.waypoints[closest_waypoint + i].pose.pose.position;

    waypoint_marker_stop.points.push_back(point);

    if (i > deceleration_search_distance)
      continue;
    waypoint_marker_decelerate.points.push_back(point);
  }

  marker_array.markers.push_back(crosswalk_marker);
  marker_array.markers.push_back(waypoint_marker_stop);
  marker_array.markers.push_back(waypoint_marker_decelerate);
  if (kind != EControl::KEEP)
    marker_array.markers.push_back(stop_line);
  detection_range_pub.publish(marker_array);
  marker_array.markers.clear();
}

// same as velocity_set.cpp - except for no reference to vector maps
EControl obstacleDetection(const VelocitySetInfo vs_info, int closest_waypoint, int detection_waypoint,
                           const autoware_msgs::Lane& lane, const lanelet::ConstLanelets& closest_crosswalks,
                           const ros::Publisher& detection_range_pub, const ros::Publisher& obstacle_pub,
                           int* obstacle_waypoint, const int deceleration_search_distance,
                           const int stop_search_distance, const bool disable_side_deceleration)

{
  ObstaclePoints obstacle_points;

  EControl detection_result =
      pointsDetection(vs_info, closest_waypoint, detection_waypoint, lane, closest_crosswalks, obstacle_waypoint,
                      &obstacle_points, deceleration_search_distance, stop_search_distance, disable_side_deceleration);

  displayDetectionRange(vs_info, lane, closest_crosswalks, closest_waypoint, detection_waypoint, detection_result,
                        *obstacle_waypoint, detection_range_pub, deceleration_search_distance, stop_search_distance);

  static int false_count = 0;
  static EControl prev_detection = EControl::KEEP;
  static int prev_obstacle_waypoint = -1;

  if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE)
  {
    // stop because we found obstacles
    displayObstacle(detection_result, obstacle_points, obstacle_pub);
    prev_detection = detection_result;
    false_count = 0;
    prev_obstacle_waypoint = *obstacle_waypoint;
    return detection_result;
  }
  else if (prev_detection == EControl::STOP || prev_detection == EControl::STOPLINE)
  {
    // there are no obstacles, but wait a little for safety
    false_count++;
    if (false_count < 5)
    {
      *obstacle_waypoint = prev_obstacle_waypoint;
      displayObstacle(prev_detection, obstacle_points, obstacle_pub);
      return prev_detection;
    }
  }
  else if (detection_result == EControl::DECELERATE)
  {
    // decelerate because we found obstacles
    displayObstacle(detection_result, obstacle_points, obstacle_pub);
    prev_detection = detection_result;
    false_count = 0;
    prev_obstacle_waypoint = *obstacle_waypoint;
    return detection_result;
  }
  else if (prev_detection == EControl::DECELERATE)
  {
    // there are no obstacles, but wait a little for safety
    false_count++;
    if (false_count < 5)
    {
      *obstacle_waypoint = prev_obstacle_waypoint;
      displayObstacle(prev_detection, obstacle_points, obstacle_pub);
      return prev_detection;
    }
  }

  // there are no obstacles, so we move forward
  *obstacle_waypoint = -1;
  false_count = 0;
  prev_detection = EControl::KEEP;

  return detection_result;
}

// change velocity according to detected obstacles and stoplines
// same as velocity_set.cpp
void changeWaypoints(const VelocitySetInfo& vs_info, const EControl& detection_result, int closest_waypoint,
                     int obstacle_waypoint, const ros::Publisher& final_waypoints_pub, VelocitySetPath* vs_path)
{
  double deceleration = vs_info.getVelocityChangeLimit();

  if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE)  // STOP for obstacle/stopline
  {  // stop_waypoint is about stop_distance meter away from obstacles/stoplines
    // change waypoints to stop by the stop_waypoint
    deceleration = (detection_result == EControl::STOPLINE) ? vs_info.getDecelerationStopline() :
                                                              vs_info.getDecelerationObstacle();
    double stop_distance = (detection_result == EControl::STOPLINE) ? vs_info.getStopDistanceStopline() :
                                                                      vs_info.getStopDistanceObstacle();
    int stop_first_index = calcWaypointIndexReverse(vs_path->getPrevWaypoints(), obstacle_waypoint, stop_distance);
    vs_path->changeWaypointsForStopping(stop_first_index, closest_waypoint, deceleration);
  }
  else if (detection_result == EControl::DECELERATE)  // DECELERATE for obstacles
  {
    vs_path->initializeNewWaypoints();
    deceleration = vs_info.getDecelerationObstacle();
    double decel_distance = vs_info.getDecelerationDistanceObstacle();
    int decel_first_index = calcWaypointIndexReverse(vs_path->getPrevWaypoints(), obstacle_waypoint, decel_distance);
    vs_path->changeWaypointsForDeceleration(decel_first_index, obstacle_waypoint, closest_waypoint,
                                            vs_info.getDecelerationObstacle());
  }
  else
  {  // KEEP
    vs_path->initializeNewWaypoints();
  }

  vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), deceleration, closest_waypoint);
  vs_path->avoidSuddenAcceleration(deceleration, closest_waypoint);
  vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());
  final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
}

void binMapCallback(const autoware_lanelet2_msgs::MapBin& msg)
{
  g_lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(msg, g_lanelet_map);
  g_loaded_lanelet_map = true;
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(g_lanelet_map);
  g_crosswalk_lanelets = lanelet::utils::query::crosswalkLanelets(all_lanelets);
  ROS_INFO("velocity_set_lanelet2: lanelet map loaded\n");
}

int main(int argc, char** argv)
{
  g_loaded_lanelet_map = false;

  ros::init(argc, argv, "velocity_set");
  ros::NodeHandle rosnode;

  ros::NodeHandle private_rosnode("~");

  // parameters from ros param
  bool use_crosswalk_detection;
  bool enable_multiple_crosswalk_detection;
  bool enablePlannerDynamicSwitch;
  bool disable_side_deceleration;
  std::string points_topic;
  int deceleration_search_distance;
  int stop_search_distance;

  private_rosnode.param<bool>("use_crosswalk_detection", use_crosswalk_detection, true);
  private_rosnode.param<bool>("enable_multiple_crosswalk_detection", enable_multiple_crosswalk_detection, true);
  private_rosnode.param<bool>("enablePlannerDynamicSwitch", enablePlannerDynamicSwitch, false);
  private_rosnode.param<bool>("disable_side_deceleration", disable_side_deceleration, false);
  private_rosnode.param<std::string>("points_topic", points_topic, "points_lanes");
  private_rosnode.param<int>("deceleration_search_distance", deceleration_search_distance, 30);
  private_rosnode.param<int>("stop_search_distance", stop_search_distance, 60);

  VelocitySetPath vs_path;
  VelocitySetInfo vs_info;

  // map subscriber
  ros::Subscriber bin_map_sub = rosnode.subscribe("lanelet_map_bin", 1, binMapCallback);

  // velocity set path subscriber
  ros::Subscriber waypoints_sub =
      rosnode.subscribe("safety_waypoints", 1, &VelocitySetPath::waypointsCallback, &vs_path);
  ros::Subscriber current_vel_sub =
      rosnode.subscribe("current_velocity", 1, &VelocitySetPath::currentVelocityCallback, &vs_path);

  // velocity set info subscriber
  ros::Subscriber config_sub = rosnode.subscribe("config/velocity_set", 1, &VelocitySetInfo::configCallback, &vs_info);
  ros::Subscriber points_sub = rosnode.subscribe(points_topic, 1, &VelocitySetInfo::pointsCallback, &vs_info);
  ros::Subscriber control_pose_sub =
      rosnode.subscribe("current_pose", 1, &VelocitySetInfo::controlPoseCallback, &vs_info);
  ros::Subscriber detectionresult_sub =
      rosnode.subscribe("state/stopline_wpidx", 1, &VelocitySetInfo::detectionCallback, &vs_info);

  // TF Listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // publisher
  ros::Publisher detection_range_pub = rosnode.advertise<visualization_msgs::MarkerArray>("detection_range", 1);
  ros::Publisher obstacle_pub = rosnode.advertise<visualization_msgs::Marker>("obstacle", 1);
  ros::Publisher obstacle_waypoint_pub = rosnode.advertise<std_msgs::Int32>("obstacle_waypoint", 1, true);
  ros::Publisher stopline_waypoint_pub = rosnode.advertise<std_msgs::Int32>("stopline_waypoint", 1, true);

  ros::Publisher final_waypoints_pub;
  final_waypoints_pub = rosnode.advertise<autoware_msgs::Lane>("final_waypoints", 1, true);

  ros::Rate loop_rate(LOOP_RATE);
  int closest_waypoint = -1;
  while (ros::ok())
  {
    ros::spinOnce();

    try
    {
      geometry_msgs::TransformStamped map_to_lidar_tf =
          tfBuffer.lookupTransform("map", "velodyne", ros::Time::now(), ros::Duration(2.0));
      vs_info.setLocalizerPose(map_to_lidar_tf);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("Failed to get map->lidar transform. skip computation: %s", ex.what());
      continue;
    }

    closest_waypoint = updateCurrentIndex(vs_path.getPrevWaypoints(), vs_info.getControlPose().pose, closest_waypoint);

    if (!vs_info.getSetPose() || !vs_path.getSetPath())
    {
      loop_rate.sleep();
      continue;
    }

    int detection_waypoint = -1;
    lanelet::ConstLanelets closest_crosswalks;
    if (use_crosswalk_detection)
    {
      if (!g_loaded_lanelet_map)
      {
        ROS_WARN("use_crosswalk_detection is true, but lanelet map is not loaded!");
      }
      detection_waypoint =
          findClosestCrosswalk(g_crosswalk_lanelets, closest_waypoint, vs_path.getPrevWaypoints(), stop_search_distance,
                               &closest_crosswalks, enable_multiple_crosswalk_detection);
    }

    int obstacle_waypoint = -1;
    EControl detection_result =
        obstacleDetection(vs_info, closest_waypoint, detection_waypoint, vs_path.getPrevWaypoints(), closest_crosswalks,
                          detection_range_pub, obstacle_pub, &obstacle_waypoint, deceleration_search_distance,
                          stop_search_distance, disable_side_deceleration);

    changeWaypoints(vs_info, detection_result, closest_waypoint, obstacle_waypoint, final_waypoints_pub, &vs_path);

    vs_info.clearPoints();

    // publish obstacle waypoint index
    std_msgs::Int32 obstacle_waypoint_index;
    std_msgs::Int32 stopline_waypoint_index;
    if (detection_result == EControl::STOP)
    {
      obstacle_waypoint_index.data = obstacle_waypoint;
      stopline_waypoint_index.data = -1;
    }
    else if (detection_result == EControl::STOPLINE)
    {
      obstacle_waypoint_index.data = -1;
      stopline_waypoint_index.data = obstacle_waypoint;
    }
    else
    {
      obstacle_waypoint_index.data = -1;
      stopline_waypoint_index.data = -1;
    }
    obstacle_waypoint_pub.publish(obstacle_waypoint_index);
    stopline_waypoint_pub.publish(stopline_waypoint_index);

    vs_path.resetFlag();
    loop_rate.sleep();
  }

  return 0;
}
