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

#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <string>
#include <vector>

#include "autoware_config_msgs/ConfigLaneStop.h"
#include "autoware_msgs/LaneArray.h"
#include "autoware_msgs/TrafficLight.h"
#include "libwaypoint_follower/libwaypoint_follower.h"

namespace
{
ros::Publisher g_local_mark_pub;
ros::Publisher g_global_mark_pub;

constexpr int32_t TRAFFIC_LIGHT_RED = 0;
constexpr int32_t TRAFFIC_LIGHT_GREEN = 1;
constexpr int32_t TRAFFIC_LIGHT_UNKNOWN = 2;

int _closest_waypoint = -1;

visualization_msgs::MarkerArray g_global_marker_array;
visualization_msgs::MarkerArray g_local_waypoints_marker_array;

bool g_config_manual_detection = true;
int32_t g_current_traffic_light = TRAFFIC_LIGHT_UNKNOWN;

enum class ChangeFlag : int32_t
{
  straight,
  right,
  left,

  unknown = -1,
};

typedef std::underlying_type<ChangeFlag>::type ChangeFlagInteger;

void setLifetime(double sec, visualization_msgs::MarkerArray* marker_array)
{
  ros::Duration lifetime(sec);
  for (auto& marker : marker_array->markers)
  {
    marker.lifetime = lifetime;
  }
}

void publishMarkerArray(const visualization_msgs::MarkerArray& marker_array, const ros::Publisher& publisher,
                        bool delete_markers = false)
{
  visualization_msgs::MarkerArray msg;

  // insert local marker
  msg.markers.insert(msg.markers.end(), marker_array.markers.begin(), marker_array.markers.end());

  if (delete_markers)
  {
    for (auto& marker : msg.markers)
    {
      marker.action = visualization_msgs::Marker::DELETE;
    }
  }

  publisher.publish(msg);
}

void createGlobalLaneArrayVelocityMarker(const autoware_msgs::LaneArray& lane_waypoints_array)
{
  visualization_msgs::MarkerArray tmp_marker_array;
  // display by markers the velocity of each waypoint.
  visualization_msgs::Marker velocity_marker;
  velocity_marker.header.frame_id = "map";
  velocity_marker.header.stamp = ros::Time::now();
  velocity_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  velocity_marker.action = visualization_msgs::Marker::ADD;
  velocity_marker.scale.z = 0.1;
  velocity_marker.color.r = 1;
  velocity_marker.color.g = 1;
  velocity_marker.color.b = 1;
  velocity_marker.color.a = 1.0;
  velocity_marker.frame_locked = true;

  int count = 1;
  for (const auto& lane : lane_waypoints_array.lanes)
  {
    velocity_marker.ns = "global_velocity_lane_" + std::to_string(count);
    for (int i = 0; i < static_cast<int>(lane.waypoints.size()); i++)
    {
      velocity_marker.id = i;
      geometry_msgs::Point relative_p;
      relative_p.x = 0;
      relative_p.y = 0;
      relative_p.z = 0.1;
      velocity_marker.pose.position = calcAbsoluteCoordinate(relative_p, lane.waypoints[i].pose.pose);
      velocity_marker.pose.position.z += 0.2;

      // double to string
      std::string vel = std::to_string(mps2kmph(lane.waypoints[i].twist.twist.linear.x));
      velocity_marker.text = vel.erase(vel.find_first_of(".") + 2);

      tmp_marker_array.markers.push_back(velocity_marker);
    }
    count++;
  }

  g_global_marker_array.markers.insert(g_global_marker_array.markers.end(), tmp_marker_array.markers.begin(),
                                       tmp_marker_array.markers.end());
}

void createGlobalLaneArrayIndexMarker(const autoware_msgs::LaneArray& lane_waypoints_array)
{
  visualization_msgs::MarkerArray tmp_marker_array;
  // display by markers the index of each waypoint.
  visualization_msgs::Marker index_marker;
  index_marker.header.frame_id = "map";
  index_marker.header.stamp = ros::Time::now();
  index_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  index_marker.action = visualization_msgs::Marker::ADD;
  index_marker.scale.z = 0.1;
  index_marker.color.r = 1;
  index_marker.color.g = 1;
  index_marker.color.b = 1;
  index_marker.color.a = 0.6;
  index_marker.frame_locked = true;

  int count = 1;
  for (const auto& lane : lane_waypoints_array.lanes)
  {
    index_marker.ns = "global_index_lane_" + std::to_string(count);
    for (int i = 0; i < static_cast<int>(lane.waypoints.size()); i++)
    {
      index_marker.id = i;
      geometry_msgs::Point relative_p;
      relative_p.x = -0.2;
      index_marker.pose.position = calcAbsoluteCoordinate(relative_p, lane.waypoints[i].pose.pose);
      index_marker.pose.position.z += 0.2;

      // double to string
      std::string str = std::to_string(i);
      index_marker.text = str;

      tmp_marker_array.markers.push_back(index_marker);
    }
    count++;
  }

  g_global_marker_array.markers.insert(g_global_marker_array.markers.end(), tmp_marker_array.markers.begin(),
                                       tmp_marker_array.markers.end());
}

void createGlobalLaneArrayChangeFlagMarker(const autoware_msgs::LaneArray& lane_waypoints_array)
{
  visualization_msgs::MarkerArray tmp_marker_array;
  // display by markers the velocity of each waypoint.
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.z = 0.4;
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 1;
  marker.color.a = 1.0;
  marker.frame_locked = true;

  int count = 1;
  for (const auto& lane : lane_waypoints_array.lanes)
  {
    marker.ns = "global_change_flag_lane_" + std::to_string(count);
    for (int i = 0; i < static_cast<int>(lane.waypoints.size()); i++)
    {
      marker.id = i;
      geometry_msgs::Point relative_p;
      relative_p.x = -0.1;
      marker.pose.position = calcAbsoluteCoordinate(relative_p, lane.waypoints[i].pose.pose);
      marker.pose.position.z += 0.2;

      // double to string
      std::string str = "";
      if (lane.waypoints[i].change_flag == static_cast<ChangeFlagInteger>(ChangeFlag::straight))
      {
        str = "S";
      }
      else if (lane.waypoints[i].change_flag == static_cast<ChangeFlagInteger>(ChangeFlag::right))
      {
        str = "R";
      }
      else if (lane.waypoints[i].change_flag == static_cast<ChangeFlagInteger>(ChangeFlag::left))
      {
        str = "L";
      }
      else if (lane.waypoints[i].change_flag == static_cast<ChangeFlagInteger>(ChangeFlag::unknown))
      {
        str = "U";
      }

      marker.text = str;

      tmp_marker_array.markers.push_back(marker);
    }
    count++;
  }

  g_global_marker_array.markers.insert(g_global_marker_array.markers.end(), tmp_marker_array.markers.begin(),
                                       tmp_marker_array.markers.end());
}

void createLocalWaypointVelocityMarker(std_msgs::ColorRGBA color, const autoware_msgs::Lane& lane_waypoint)
{
  // no lane = nothing to draw
  if (lane_waypoint.waypoints.empty())
    return;  // use closest waypoint if available, otherwise use the very first waypoint
  int wp_index = _closest_waypoint;
  if (wp_index < 0 || wp_index >= static_cast<int>(lane_waypoint.waypoints.size()))
  {
    wp_index = 0;  // fallback
  }
  const geometry_msgs::Pose& base_pose = lane_waypoint.waypoints[wp_index].pose.pose;

  // display by markers the velocity of each waypoint.
  visualization_msgs::Marker velocity;
  velocity.header.frame_id = "map";
  velocity.header.stamp = ros::Time::now();
  velocity.ns = "local_waypoint_velocity";
  velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  velocity.action = visualization_msgs::Marker::ADD;
  velocity.scale.z = 0.1;
  velocity.color = color;
  velocity.frame_locked = true;

  for (int i = 0; i < static_cast<int>(lane_waypoint.waypoints.size()); i++)
  {
    velocity.id = i;
    geometry_msgs::Point relative_p;
    relative_p.x = 0;
    relative_p.y = 0;
    relative_p.z = base_pose.position.z + 0.1;
    if (lane_waypoint.waypoints[i].twist.twist.linear.x > 0)
    {
      double velocity_scaled = std::max(1.0, 0.2 + lane_waypoint.waypoints[i].twist.twist.linear.x / 3.6);

      velocity.color.r = 0.2;
      velocity.color.g = 0.2;
      velocity.color.b = velocity_scaled;
    }
    else if (lane_waypoint.waypoints[i].twist.twist.linear.x < 0)
    {
      double velocity_scaled = std::max(1.0, 0.2 - lane_waypoint.waypoints[i].twist.twist.linear.x / 3.6);
      velocity.color.r = velocity_scaled;
      velocity.color.g = 0.2;
      velocity.color.b = 0.2;
    }
    velocity.pose.position = calcAbsoluteCoordinate(relative_p, lane_waypoint.waypoints[i].pose.pose);
    velocity.pose.position.z += 0.2;

    // double to string
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << mps2kmph(lane_waypoint.waypoints[i].twist.twist.linear.x);
    velocity.text = oss.str();

    g_local_waypoints_marker_array.markers.push_back(velocity);
  }
}

void createGlobalLaneArrayMarker(std_msgs::ColorRGBA color, const autoware_msgs::LaneArray& lane_waypoints_array)
{
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time::now();
  lane_waypoint_marker.ns = "global_lane_array_marker";
  lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 1.0;
  lane_waypoint_marker.color = color;
  lane_waypoint_marker.frame_locked = true;

  int count = 0;
  for (const auto& lane : lane_waypoints_array.lanes)
  {
    lane_waypoint_marker.points.clear();
    lane_waypoint_marker.id = count;

    for (const auto& el : lane.waypoints)
    {
      geometry_msgs::Point point;
      point = el.pose.pose.position;
      lane_waypoint_marker.points.push_back(point);
    }
    g_global_marker_array.markers.push_back(lane_waypoint_marker);
    count++;
  }
}

void createGlobalLaneArrayOrientationMarker(const autoware_msgs::LaneArray& lane_waypoints_array)
{
  visualization_msgs::MarkerArray tmp_marker_array;
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time::now();
  lane_waypoint_marker.type = visualization_msgs::Marker::ARROW;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.25;
  lane_waypoint_marker.scale.y = 0.05;
  lane_waypoint_marker.scale.z = 0.05;
  lane_waypoint_marker.color.r = 1.0;
  lane_waypoint_marker.color.a = 1.0;
  lane_waypoint_marker.frame_locked = true;

  int count = 1;
  for (const auto& lane : lane_waypoints_array.lanes)
  {
    lane_waypoint_marker.ns = "global_lane_waypoint_orientation_marker_" + std::to_string(count);

    for (int i = 0; i < static_cast<int>(lane.waypoints.size()); i++)
    {
      lane_waypoint_marker.id = i;
      lane_waypoint_marker.pose = lane.waypoints[i].pose.pose;
      tmp_marker_array.markers.push_back(lane_waypoint_marker);
    }
    count++;
  }

  g_global_marker_array.markers.insert(g_global_marker_array.markers.end(), tmp_marker_array.markers.begin(),
                                       tmp_marker_array.markers.end());
}

void createGlobalLaneArrayTurnMarker(const autoware_msgs::LaneArray& lane_waypoints_array)
{
  visualization_msgs::MarkerArray tmp_marker_array;
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time::now();
  lane_waypoint_marker.type = visualization_msgs::Marker::ARROW;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.25;
  lane_waypoint_marker.scale.y = 0.05;
  lane_waypoint_marker.scale.z = 0.05;
  lane_waypoint_marker.color.r = 1.0;
  lane_waypoint_marker.color.g = 1.0;
  lane_waypoint_marker.color.a = 1.0;
  lane_waypoint_marker.frame_locked = true;

  int count = 1;
  for (const auto& lane : lane_waypoints_array.lanes)
  {
    lane_waypoint_marker.ns = "global_lane_waypoint_turn_marker_" + std::to_string(count);

    for (int i = 0; i < static_cast<int>(lane.waypoints.size()); i++)
    {
      uint8_t steering_state = lane.waypoints[i].wpstate.steering_state;

      if (steering_state == autoware_msgs::WaypointState::STR_LEFT ||
          steering_state == autoware_msgs::WaypointState::STR_RIGHT)
      {
        lane_waypoint_marker.id = i;
        lane_waypoint_marker.pose = lane.waypoints[i].pose.pose;

        tf2::Quaternion directional_offset;
        tf2::Quaternion wp_orientation;
        tf2::convert(lane_waypoint_marker.pose.orientation, wp_orientation);

        if (steering_state == autoware_msgs::WaypointState::STR_LEFT)
        {
          directional_offset.setRPY(0, 0, M_PI / 2);
        }
        else if (steering_state == autoware_msgs::WaypointState::STR_RIGHT)
        {
          directional_offset.setRPY(0, 0, -M_PI / 2);
        }
        wp_orientation *= directional_offset;
        wp_orientation.normalize();

        tf2::convert(wp_orientation, lane_waypoint_marker.pose.orientation);
        tmp_marker_array.markers.push_back(lane_waypoint_marker);
      }
    }
    count++;
  }

  g_global_marker_array.markers.insert(g_global_marker_array.markers.end(), tmp_marker_array.markers.begin(),
                                       tmp_marker_array.markers.end());
}

void createLocalPathMarker(std_msgs::ColorRGBA color, const autoware_msgs::Lane& lane_waypoint)
{
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time::now();
  lane_waypoint_marker.ns = "local_path_marker";
  lane_waypoint_marker.id = 0;
  lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.06;
  lane_waypoint_marker.scale.y = 0.06;
  lane_waypoint_marker.scale.z = 0.06;
  lane_waypoint_marker.color = color;
  lane_waypoint_marker.frame_locked = true;

  if (lane_waypoint.waypoints.empty())
    return;

  const double base_z = lane_waypoint.waypoints.front().pose.pose.position.z;

  for (const auto& wp : lane_waypoint.waypoints)
  {
    geometry_msgs::Point point = wp.pose.pose.position;
    point.z = base_z;  // align with robot height
    lane_waypoint_marker.points.push_back(point);
  }

  g_local_waypoints_marker_array.markers.push_back(lane_waypoint_marker);
}

void createLocalPointMarker(const autoware_msgs::Lane& lane_waypoint)
{
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time::now();
  lane_waypoint_marker.ns = "local_point_marker";
  lane_waypoint_marker.id = 0;
  lane_waypoint_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.12;
  lane_waypoint_marker.scale.y = 0.12;
  lane_waypoint_marker.scale.z = 0.12;
  lane_waypoint_marker.frame_locked = true;

  if (lane_waypoint.waypoints.empty())
    return;

  const double base_z = lane_waypoint.waypoints.front().pose.pose.position.z;
  const double stop_threshold_mps = 1.0e-4;

  for (const auto& wp : lane_waypoint.waypoints)
  {
    geometry_msgs::Point point = wp.pose.pose.position;
    point.z = base_z;
    lane_waypoint_marker.points.push_back(point);

    const double v = wp.twist.twist.linear.x;
    std_msgs::ColorRGBA c;
    c.a = 0.5;

    if (std::fabs(v) < stop_threshold_mps)
    {
      // stop: red
      c.r = 1.0;
      c.g = 0.0;
      c.b = 0.0;
    }
    else if (v > 0.0)
    {
      // forward: green
      c.r = 0.0;
      c.g = 1.0;
      c.b = 0.0;
    }
    else
    {
      // backward: yellow
      c.r = 1.0;
      c.g = 1.0;
      c.b = 0.0;
    }

    lane_waypoint_marker.colors.push_back(c);
  }

  g_local_waypoints_marker_array.markers.push_back(lane_waypoint_marker);
}

void createLocalTrafficLightIndicatorMarker(const autoware_msgs::Lane& lane_waypoint)
{
  // no lane = nothing to draw
  if (lane_waypoint.waypoints.empty())
    return;

  // use closest waypoint if available, otherwise use the very first waypoint
  int wp_index = _closest_waypoint;
  if (wp_index < 0 || wp_index >= static_cast<int>(lane_waypoint.waypoints.size()))
  {
    wp_index = 0;  // fallback
  }

  const geometry_msgs::Pose& base_pose = lane_waypoint.waypoints[wp_index].pose.pose;

  // traffic light body
  visualization_msgs::Marker body;
  body.header.frame_id = "map";
  body.header.stamp = ros::Time::now();
  body.ns = "local_traffic_light_indicator";
  body.id = 0;
  body.type = visualization_msgs::Marker::CUBE;
  body.action = visualization_msgs::Marker::ADD;
  body.frame_locked = true;

  body.pose = base_pose;
  body.pose.position.z += 0.5;

  // box size (slightly tall to fit three lamps)
  body.scale.x = 0.02;
  body.scale.y = 0.4;
  body.scale.z = 1.1;

  body.color.r = 0.1f;
  body.color.g = 0.1f;
  body.color.b = 0.1f;
  body.color.a = 0.8f;

  g_local_waypoints_marker_array.markers.push_back(body);

  // helper lambda to create one lamp
  auto makeLamp = [&](int id, double dz, float r, float g, float b, float a) {
    visualization_msgs::Marker lamp;
    lamp.header = body.header;
    lamp.ns = "local_traffic_light_indicator";
    lamp.id = id;
    lamp.type = visualization_msgs::Marker::SPHERE;
    lamp.action = visualization_msgs::Marker::ADD;
    lamp.frame_locked = true;

    lamp.pose = body.pose;
    lamp.pose.position.z += dz;

    lamp.scale.x = 0.3;
    lamp.scale.y = 0.3;
    lamp.scale.z = 0.3;

    lamp.color.r = r;
    lamp.color.g = g;
    lamp.color.b = b;
    lamp.color.a = a;

    g_local_waypoints_marker_array.markers.push_back(lamp);
  };

  // brightness settings
  const float on_alpha = 1.0f;
  const float off_alpha = 0.2f;

  // which lamp is on?
  const bool red_on = (g_current_traffic_light == TRAFFIC_LIGHT_RED);
  const bool green_on = (g_current_traffic_light == TRAFFIC_LIGHT_GREEN);
  // use yellow when we do not have explicit state (UNKNOWN)
  const bool yellow_on = (g_current_traffic_light == TRAFFIC_LIGHT_UNKNOWN);

  // layout: top = red, middle = yellow, bottom = green
  // offsets relative to body center
  const double dz_red = +0.35;
  const double dz_yellow = 0.0;
  const double dz_green = -0.35;

  // red lamp (top)
  makeLamp(1, dz_red, 1.0f, 0.0f, 0.0f, red_on ? on_alpha : off_alpha);

  // yellow lamp (middle)
  makeLamp(2, dz_yellow, 1.0f, 1.0f, 0.0f, yellow_on ? on_alpha : off_alpha);

  // green lamp (bottom)
  makeLamp(3, dz_green, 0.0f, 1.0f, 0.0f, green_on ? on_alpha : off_alpha);
}

void createLocalDirectionMarker(const autoware_msgs::Lane& lane_waypoint)
{
  if (lane_waypoint.waypoints.empty())
    return;

  int wp_index = _closest_waypoint;
  if (wp_index < 0 || wp_index >= static_cast<int>(lane_waypoint.waypoints.size()))
  {
    wp_index = 0;
  }

  const auto& wp = lane_waypoint.waypoints[wp_index];
  const double v = wp.twist.twist.linear.x;
  const double stop_threshold_mps = 0.1;

  // ============================================================
  // Arrow parameters
  // ============================================================
  const double shaft_length = 1.0;     // shaft length
  const double shaft_diameter = 0.10;  // shaft diameter
  const double head_length = 0.45;     // head length
  const double head_diameter = 0.3;    // head diameter

  // ============================================================
  // Create arrow marker (points mode)
  // ============================================================
  visualization_msgs::Marker arrow;
  arrow.header.frame_id = "map";
  arrow.header.stamp = ros::Time::now();
  arrow.ns = "direction_indicator";
  arrow.id = 1;
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.action = visualization_msgs::Marker::ADD;
  arrow.frame_locked = true;

  // base position
  geometry_msgs::Point p0;
  p0.x = wp.pose.pose.position.x;
  p0.y = wp.pose.pose.position.y;
  p0.z = wp.pose.pose.position.z + 1.6;

  geometry_msgs::Point p1 = p0;
  double yaw = tf2::getYaw(wp.pose.pose.orientation);

  p1.x += std::cos(yaw) * shaft_length;
  p1.y += std::sin(yaw) * shaft_length;

  // ============================================================
  // Color & direction
  // ============================================================
  if (std::fabs(v) < stop_threshold_mps)
  {
    // STOP → 薄い白（半透明）
    arrow.color.r = 1.0;
    arrow.color.g = 1.0;
    arrow.color.b = 1.0;
    arrow.color.a = 0.35;  // 透明度高めで“薄い白”
  }
  else if (v > 0.0)
  {
    // Forward → green
    arrow.color.r = 0.0;
    arrow.color.g = 1.0;
    arrow.color.b = 0.0;
    arrow.color.a = 1.0;
  }
  else
  {
    // Backward → yellow + reverse direction
    arrow.color.r = 1.0;
    arrow.color.g = 1.0;
    arrow.color.b = 0.0;
    arrow.color.a = 1.0;

    // reverse vector
    std::swap(p0, p1);
  }

  arrow.points.push_back(p0);
  arrow.points.push_back(p1);

  // ============================================================
  // scale in points-mode
  // ============================================================
  arrow.scale.x = shaft_diameter;
  arrow.scale.y = head_diameter;
  arrow.scale.z = head_length;

  g_local_waypoints_marker_array.markers.push_back(arrow);
}

void lightCallback(const autoware_msgs::TrafficLightConstPtr& msg)
{
  g_current_traffic_light = msg->traffic_light;
}

void receiveAutoDetection(const autoware_msgs::TrafficLightConstPtr& msg)
{
  if (!g_config_manual_detection)
    lightCallback(msg);
}

void receiveManualDetection(const autoware_msgs::TrafficLightConstPtr& msg)
{
  if (g_config_manual_detection)
    lightCallback(msg);
}

void configParameter(const autoware_config_msgs::ConfigLaneStopConstPtr& msg)
{
  g_config_manual_detection = msg->manual_detection;
}

void laneArrayCallback(const autoware_msgs::LaneArrayConstPtr& msg)
{
  publishMarkerArray(g_global_marker_array, g_global_mark_pub, true);
  g_global_marker_array.markers.clear();
  // createGlobalLaneArrayVelocityMarker(*msg);
  // createGlobalLaneArrayOrientationMarker(*msg);
  // createGlobalLaneArrayChangeFlagMarker(*msg);
  // createGlobalLaneArrayTurnMarker(*msg);
  createGlobalLaneArrayIndexMarker(*msg);
  publishMarkerArray(g_global_marker_array, g_global_mark_pub);
}

void finalCallback(const autoware_msgs::LaneConstPtr& msg)
{
  g_local_waypoints_marker_array.markers.clear();

  // path at robot height
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 1.0;
  color.a = 0.8;

  createLocalPathMarker(color, *msg);

  // colored spheres
  createLocalPointMarker(*msg);

  // velocity visualization
  createLocalWaypointVelocityMarker(color, *msg);

  // indicator on top of the robot
  createLocalTrafficLightIndicatorMarker(*msg);
  createLocalDirectionMarker(*msg);

  setLifetime(0.5, &g_local_waypoints_marker_array);
  publishMarkerArray(g_local_waypoints_marker_array, g_local_mark_pub);
}

void closestCallback(const std_msgs::Int32ConstPtr& msg)
{
  _closest_waypoint = msg->data;
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoints_marker_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // subscribe traffic light
  ros::Subscriber light_sub = nh.subscribe("light_color", 10, receiveAutoDetection);
  ros::Subscriber light_managed_sub = nh.subscribe("light_color_managed", 10, receiveManualDetection);

  // subscribe global waypoints
  ros::Subscriber lane_array_sub = nh.subscribe("lane_waypoints_array", 10, laneArrayCallback);
  ros::Subscriber traffic_array_sub = nh.subscribe("traffic_waypoints_array", 10, laneArrayCallback);

  // subscribe local waypoints
  ros::Subscriber final_sub = nh.subscribe("final_waypoints", 10, finalCallback);
  ros::Subscriber closest_sub = nh.subscribe("closest_waypoint", 10, closestCallback);

  // subscribe config
  ros::Subscriber config_sub = nh.subscribe("config/lane_stop", 10, configParameter);

  g_local_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("local_waypoints_mark", 10, true);
  g_global_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("global_waypoints_mark", 10, true);

  ros::spin();
}
