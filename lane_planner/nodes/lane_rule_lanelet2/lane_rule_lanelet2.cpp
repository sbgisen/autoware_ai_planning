/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
 *
 * Authors: Simon Thompson
 *
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>

#include <cmath>
#include <string>
#include <vector>

#include <autoware_config_msgs/ConfigLaneRule.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/Waypoint.h>

// use of routing graph in detecting lanelets for each waypoint
// following routing graph may be computationally cheaper if very large map
// (computation cost may be in unique insert used to store connected lanelets)
// for smaller maps - faster to just find nearest lanelet for each waypoint.
#define LANE_RULES_USE_ROUTING_GRAPH false

static double g_config_acceleration = 1;            // m/s^2
static double g_config_stopline_search_radius = 1;  // meter
static int g_config_number_of_zeros_ahead = 0;
static int g_config_number_of_zeros_behind = 0;
static int g_config_number_of_smoothing_count = 0;

static int g_config_number_of_slowdown = 40;           // meter
static double g_config_slowdown_velocity_scale = 0.5;  // ratio

static std::string g_frame_id = "map";

static ros::Publisher g_traffic_pub;
static ros::Publisher g_red_pub;
static ros::Publisher g_green_pub;
static autoware_msgs::LaneArray g_cached_waypoint;

static lanelet::LaneletMapPtr g_lanelet_map;
static lanelet::routing::RoutingGraphPtr g_routing_graph;
static bool g_loaded_lanelet_map = false;

// create new lane with given lane and header
autoware_msgs::Lane create_new_lane(const autoware_msgs::Lane& lane, const std_msgs::Header& header)
{
  autoware_msgs::Lane l = lane;
  l.header = header;

  for (autoware_msgs::Waypoint& w : l.waypoints)
  {
    w.pose.header = header;
    w.twist.header = header;
  }

  return l;
}

// create array of waypoints in Eigen Vector3d type
void create_waypoint_array(const autoware_msgs::Lane& lane, std::vector<Eigen::Vector3d>* waypoints)
{
  if (waypoints == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": waypoints is null pointer!");
    return;
  }

  waypoints->clear();
  for (const autoware_msgs::Waypoint& w : lane.waypoints)
  {
    geometry_msgs::Point gp = w.pose.pose.position;
    Eigen::Vector3d p(gp.x, gp.y, gp.z);

    waypoints->push_back(p);
  }
}

// find nearest lanelets to a given point
lanelet::Lanelets find_nearest_lanelets(const Eigen::Vector3d& p, int n)
{
  return (g_lanelet_map->laneletLayer.nearest(lanelet::BasicPoint2d(p.x(), p.y()), n));
}

// find nearest lanelet to a given point
lanelet::Lanelet find_actual_nearest_lanelet(const Eigen::Vector3d& p)
{
  std::vector<std::pair<double, lanelet::Lanelet> > actual_nearest_lls =
      lanelet::geometry::findNearest(g_lanelet_map->laneletLayer, lanelet::BasicPoint2d(p.x(), p.y()), 1);

  if (actual_nearest_lls.empty())
  {
    lanelet::Lanelet empty_lanelet;
    return empty_lanelet;
  }
  else
  {
    return actual_nearest_lls.front().second;
  }
}

void insert_unique_lanelet(const lanelet::ConstLanelet& ll, lanelet::ConstLanelets* lls)
{
  if (lls == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": lls is null pointer!");
    return;
  }
  if (std::find(lls->begin(), lls->end(), ll) == lls->end())
    lls->push_back(ll);
}

void add_connecting_lanelets(const lanelet::ConstLanelet& lanelet, lanelet::ConstLanelets* candidate_lanelets)
{
  if (candidate_lanelets == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": candidate_lanelets is null pointer!");
    return;
  }

  lanelet::Optional<lanelet::ConstLanelet> ll_opt;

  ll_opt = g_routing_graph->right(lanelet);
  if (!!ll_opt)
    insert_unique_lanelet(ll_opt.get(), candidate_lanelets);

  ll_opt = g_routing_graph->left(lanelet);
  if (!!ll_opt)
    insert_unique_lanelet(ll_opt.get(), candidate_lanelets);

  lanelet::ConstLanelets following_ll = g_routing_graph->following(lanelet);
  for (auto fll_i = following_ll.begin(); fll_i < following_ll.end(); fll_i++)
    insert_unique_lanelet((*fll_i), candidate_lanelets);
}

std::vector<size_t> check_waypoints_for_stoplines(const std::vector<Eigen::Vector3d>& waypoints)
{
  lanelet::ConstLanelet current_lanelet = find_actual_nearest_lanelet(waypoints.front());
  lanelet::ConstLanelets current_lanelets;
  current_lanelets.push_back(current_lanelet);
  std::vector<size_t> waypoint_stopline_indexes;

  size_t wp_index = 0;

  // TODO:
  // perhaps need to test boundary condition when waypoint is last in lanelet?
  for (auto wp_i = waypoints.begin(); wp_i < waypoints.end() - 1; wp_i++)
  {
    lanelet::Point3d wp_p0(lanelet::utils::getId(), (*wp_i).x(), (*wp_i).y(), (*wp_i).z());
    lanelet::Point3d wp_p1(lanelet::utils::getId(), (*(wp_i + 1)).x(), (*(wp_i + 1)).y(), (*(wp_i + 1)).z());
    lanelet::BasicPoint2d wp_p02((*wp_i).x(), (*wp_i).y());
    lanelet::ConstLanelets candidate_lanelets;

    // check if waypoint is in current lanelets - if not discard
    int ll_count = 0;

    // if no current lanelets - search again
    if (current_lanelets.size() == 0)
    {
      current_lanelets.push_back(find_actual_nearest_lanelet(*wp_i));
    }

    for (auto curr_i = current_lanelets.begin(); curr_i < current_lanelets.end();)
    {
      current_lanelet = (*curr_i);
      ll_count++;

      if (!lanelet::geometry::within(wp_p02, lanelet::utils::toHybrid(current_lanelet.polygon2d())))
      {
        current_lanelets.erase(curr_i);
      }
      else
      {
        std::vector<lanelet::ConstLineString3d> current_lanelet_stoplines =
            lanelet::utils::query::getTrafficLightStopLines(current_lanelet);

        // check if waypoint segment intersects with stoplines
        // kinda awkward to force use of boost::geometry intersect.
        // 3d line segment intersection not implememented
        lanelet::ConstLineString3d wp_ls(lanelet::utils::getId(), { wp_p0, wp_p1 });
        auto wp_ls2d = lanelet::utils::to2D(wp_ls);

        for (auto sl_i = current_lanelet_stoplines.begin(); sl_i < current_lanelet_stoplines.end(); sl_i++)
        {
          auto sl_ls2d = lanelet::utils::to2D(*sl_i);
          if (lanelet::geometry::intersects(lanelet::utils::toHybrid(wp_ls2d), lanelet::utils::toHybrid(sl_ls2d)))
          {
            waypoint_stopline_indexes.push_back(wp_index);
          }
        }

        // build up a list of possible connecting lanelets that might contain current waypoint
        if (LANE_RULES_USE_ROUTING_GRAPH)
          add_connecting_lanelets(current_lanelet, &candidate_lanelets);

        // increment iterator here, erase call increments if wp not in current lanelet
        curr_i++;
      }
    }

    // add candidate lanelets to current for next waypoint (in case path moves to next lanelets)
    if (LANE_RULES_USE_ROUTING_GRAPH)
    {
      for (auto cand_i = candidate_lanelets.begin(); cand_i < candidate_lanelets.end(); cand_i++)
      {
        insert_unique_lanelet((*cand_i), &current_lanelets);
      }
    }

    wp_index++;
  }  // for each waypoint

  return waypoint_stopline_indexes;
}

// apply acceleration from given starting point
// same as lane_rule.cpp
autoware_msgs::Lane apply_acceleration(const autoware_msgs::Lane& lane, double acceleration, size_t start_index,
                                       size_t fixed_cnt, double fixed_vel)
{
  autoware_msgs::Lane l = lane;

  if (fixed_cnt == 0)
    return l;

  double square_vel = fixed_vel * fixed_vel;
  double distance = 0;
  for (size_t i = start_index; i < l.waypoints.size(); ++i)
  {
    if (i - start_index < fixed_cnt)
    {
      l.waypoints[i].twist.twist.linear.x = fixed_vel;
      continue;
    }
    geometry_msgs::Point a = l.waypoints[i - 1].pose.pose.position;
    geometry_msgs::Point b = l.waypoints[i].pose.pose.position;
    distance += hypot(b.x - a.x, b.y - a.y);

    const int sgn = (l.waypoints[i].twist.twist.linear.x < 0.0) ? -1 : 1;

    double v = sqrt(square_vel + 2 * acceleration * distance);
    if (v < l.waypoints[i].twist.twist.linear.x)
    {
      l.waypoints[i].twist.twist.linear.x = sgn * v;
    }
    else
      break;
  }

  return l;
}

autoware_msgs::Lane apply_stop_and_slowdown(const autoware_msgs::Lane& lane,
                                            const std::vector<size_t>& stopline_indexes, double acceleration,
                                            double deceleration, size_t stop_after_cnt, size_t stop_before_cnt,
                                            size_t slow_before_cnt, double slowdown_scale)
{
  autoware_msgs::Lane output_lane = lane;
  const int N = static_cast<int>(output_lane.waypoints.size());
  if (N <= 0)
    return output_lane;

  const double a_acc = std::max(0.0, acceleration);
  const double a_dec = std::max(0.0, deceleration);
  if (a_dec <= 0.0)
    return output_lane;

  if (!(slowdown_scale > 0.0 && slowdown_scale < 1.0))
    return output_lane;

  // Store original velocities as upper limits
  std::vector<double> orig_vel(N, 0.0);
  for (int k = 0; k < N; ++k)
    orig_vel[k] = lane.waypoints[k].twist.twist.linear.x;

  std::vector<double> output_vel = orig_vel;

  for (const size_t idx : stopline_indexes)
  {
    const int i = static_cast<int>(idx);
    if (i < 0 || i >= N)
      continue;

    // 1. Index ranges
    int zero_start_idx = std::max(0, i - static_cast<int>(stop_before_cnt));
    int zero_end_idx = std::min(N - 1, i + static_cast<int>(stop_after_cnt));
    if (zero_start_idx >= N)
      continue;

    int slow_start_idx = zero_start_idx - static_cast<int>(slow_before_cnt);
    if (slow_start_idx < 0)
      slow_start_idx = 0;

    int slow_end_idx = zero_start_idx - 1;
    if (slow_end_idx < slow_start_idx)
      slow_end_idx = slow_start_idx - 1;  // means "no low-speed section"
    const bool has_low_speed_section = (slow_end_idx >= slow_start_idx);

    // 2. Stop section: set 0 m/s
    for (int j = zero_start_idx; j <= zero_end_idx; ++j)
    {
      output_lane.waypoints[j].twist.twist.linear.x = 0.0;
    }

    if (has_low_speed_section)
    {
      // 3. Constant low-speed section [slow_start_idx, slow_end_idx]
      const double v_orig_ref = orig_vel[slow_start_idx];
      if (std::fabs(v_orig_ref) < 1.0e-4)
        continue;

      const double v_low = std::fabs(v_orig_ref) * slowdown_scale;
      const double v_low_sq = v_low * v_low;

      // 3-a. Set constant low speed (not exceeding original)
      for (int j = slow_start_idx; j <= slow_end_idx; ++j)
      {
        const int sgn = (orig_vel[j] >= 0.0) ? 1 : -1;
        const double v_mag = std::min(std::min(std::fabs(output_vel[j]), std::fabs(orig_vel[j])), v_low);

        output_vel[j] = sgn * v_mag;
        output_lane.waypoints[j].twist.twist.linear.x = output_vel[j];
      }

      // 4. Apply deceleration-to-zero limit *inside* the low-speed section
      //    so the vehicle can stop by zero_start_idx
      //    v^2 <= 2 * a_dec * d_to_stop
      double dist_to_stop = 0.0;
      for (int j = zero_start_idx - 1; j >= slow_start_idx; --j)
      {
        const geometry_msgs::Point& p0 = output_lane.waypoints[j].pose.pose.position;
        const geometry_msgs::Point& p1 = output_lane.waypoints[j + 1].pose.pose.position;
        dist_to_stop += hypot(p1.x - p0.x, p1.y - p0.y);

        const int sgn = (orig_vel[j] >= 0.0) ? 1 : -1;
        const double v_brake = std::sqrt(std::max(0.0, 2.0 * a_dec * dist_to_stop));
        const double v_mag = std::min(std::min(std::fabs(output_vel[j]), std::fabs(orig_vel[j])), v_brake);

        output_vel[j] = sgn * v_mag;
        output_lane.waypoints[j].twist.twist.linear.x = output_vel[j];
      }

      // 5. Upstream of the low-speed section:
      //    decelerate down to the entry speed of low-speed section
      //    v^2 = v_entry^2 + 2 * a_dec * d
      if (slow_start_idx - 1 >= 0)
      {
        const double v_entry = std::fabs(output_lane.waypoints[slow_start_idx].twist.twist.linear.x);
        const double v_entry_sq = v_entry * v_entry;

        double dist = 0.0;
        for (int j = slow_start_idx - 1; j >= 0; --j)
        {
          const geometry_msgs::Point& p0 = output_lane.waypoints[j].pose.pose.position;
          const geometry_msgs::Point& p1 = output_lane.waypoints[j + 1].pose.pose.position;
          dist += hypot(p1.x - p0.x, p1.y - p0.y);

          const int sgn = (orig_vel[j] >= 0.0) ? 1 : -1;
          const double v_brake = std::sqrt(std::max(0.0, v_entry_sq + 2.0 * a_dec * dist));
          if (std::fabs(orig_vel[j]) <= v_brake)
            break;

          const double v_mag = std::min(std::min(std::fabs(output_vel[j]), std::fabs(orig_vel[j])), v_brake);
          output_vel[j] = sgn * v_mag;
          output_lane.waypoints[j].twist.twist.linear.x = output_vel[j];
        }
      }
    }
    else
    {
      // No low-speed section: just brake to zero before the stop
      if (zero_start_idx - 1 >= 0)
      {
        double dist_to_stop = 0.0;
        for (int j = zero_start_idx - 1; j >= 0; --j)
        {
          const geometry_msgs::Point& p0 = output_lane.waypoints[j].pose.pose.position;
          const geometry_msgs::Point& p1 = output_lane.waypoints[j + 1].pose.pose.position;
          dist_to_stop += hypot(p1.x - p0.x, p1.y - p0.y);

          const int sgn = (orig_vel[j] >= 0.0) ? 1 : -1;
          const double v_brake = std::sqrt(std::max(0.0, 2.0 * a_dec * dist_to_stop));
          if (std::fabs(orig_vel[j]) <= v_brake)
            break;

          const double v_mag = std::min(std::min(std::fabs(output_vel[j]), std::fabs(orig_vel[j])), v_brake);
          output_vel[j] = sgn * v_mag;
          output_lane.waypoints[j].twist.twist.linear.x = output_vel[j];
        }
      }
    }

    // 6. Downstream after stop: accelerate back from 0
    //    v^2 = 2 * a_acc * d
    if (a_acc > 0.0 && zero_end_idx + 1 < N)
    {
      double dist = 0.0;
      for (int j = zero_end_idx + 1; j < N; ++j)
      {
        const geometry_msgs::Point& p0 = output_lane.waypoints[j - 1].pose.pose.position;
        const geometry_msgs::Point& p1 = output_lane.waypoints[j].pose.pose.position;
        dist += hypot(p1.x - p0.x, p1.y - p0.y);

        const int sgn = (orig_vel[j] >= 0.0) ? 1 : -1;
        const double v_brake = std::sqrt(std::max(0.0, 2.0 * a_acc * dist));
        if (std::fabs(orig_vel[j]) <= v_brake)
          break;

        const double v_mag = std::min(std::min(std::fabs(output_vel[j]), std::fabs(orig_vel[j])), v_brake);
        output_vel[j] = sgn * v_mag;
        output_lane.waypoints[j].twist.twist.linear.x = output_vel[j];
      }
    }
  }

  return output_lane;
}

autoware_msgs::Lane apply_slowdown_only(const autoware_msgs::Lane& lane, const std::vector<size_t>& stopline_indexes,
                                        double acceleration, double deceleration, size_t stop_after_cnt,
                                        size_t stop_before_cnt, size_t slow_before_cnt, double slowdown_scale)
{
  autoware_msgs::Lane output_lane = lane;
  const int N = static_cast<int>(output_lane.waypoints.size());
  if (N <= 0)
    return output_lane;

  const double a_acc = std::max(0.0, acceleration);
  const double a_dec = std::max(0.0, deceleration);
  if (a_dec <= 0.0)
    return output_lane;

  if (!(slowdown_scale > 0.0 && slowdown_scale < 1.0))
    return output_lane;

  // Original velocities (upper limits)
  std::vector<double> orig_vel(N, 0.0);
  for (int k = 0; k < N; ++k)
    orig_vel[k] = lane.waypoints[k].twist.twist.linear.x;
  std::vector<double> output_vel = orig_vel;

  for (const size_t idx : stopline_indexes)
  {
    const int i = static_cast<int>(idx);
    if (i < 0 || i >= N)
      continue;

    // "would-be stop block" for red lane
    const int zero_start_idx = std::max(0, i - static_cast<int>(stop_before_cnt));
    const int zero_end_idx = std::min(N - 1, i + static_cast<int>(stop_after_cnt));
    if (zero_start_idx >= N)
      continue;

    // Slowdown region: just before the would-be stop block
    int slow_start_idx = zero_start_idx - static_cast<int>(slow_before_cnt);
    if (slow_start_idx < 0)
      slow_start_idx = 0;

    int slow_end_idx = zero_start_idx - 1;
    if (slow_end_idx < slow_start_idx)
      slow_end_idx = slow_start_idx - 1;  // no low-speed section
    const bool has_low_speed_section = (slow_end_idx >= slow_start_idx);

    // Decide reference index for low speed
    int ref_idx = has_low_speed_section ? slow_end_idx : zero_start_idx;
    if (ref_idx < 0 || ref_idx >= N)
      continue;

    const double v_orig_ref = orig_vel[ref_idx];
    if (std::fabs(v_orig_ref) < 1.0e-4)
      continue;

    const double v_low = std::fabs(v_orig_ref) * slowdown_scale;
    const double v_low_sq = v_low * v_low;

    // ------------------------------------------------
    // 1. Low-speed section: [slow_start_idx, slow_end_idx]
    //    clamp to v_low but NEVER to 0
    // ------------------------------------------------
    if (has_low_speed_section)
    {
      for (int j = slow_start_idx; j <= slow_end_idx; ++j)
      {
        const int sgn = (orig_vel[j] >= 0.0) ? 1 : -1;
        const double v_mag = std::min(std::min(std::fabs(output_vel[j]), std::fabs(orig_vel[j])), v_low);
        output_vel[j] = sgn * v_mag;
        output_lane.waypoints[j].twist.twist.linear.x = output_vel[j];
      }
    }

    // ------------------------------------------------
    // 2. Upstream: smoothly decelerate down to v_low
    //    v^2 = v_low^2 + 2 * a_dec * d
    // ------------------------------------------------
    if (slow_start_idx - 1 >= 0)
    {
      double dist = 0.0;
      for (int j = slow_start_idx - 1; j >= 0; --j)
      {
        const geometry_msgs::Point& p0 = output_lane.waypoints[j].pose.pose.position;
        const geometry_msgs::Point& p1 = output_lane.waypoints[j + 1].pose.pose.position;
        dist += hypot(p1.x - p0.x, p1.y - p0.y);

        const int sgn = (orig_vel[j] >= 0.0) ? 1 : -1;
        const double v_brake = std::sqrt(std::max(0.0, v_low_sq + 2.0 * a_dec * dist));
        if (std::fabs(orig_vel[j]) <= v_brake)
          break;

        const double v_mag = std::min(std::min(std::fabs(output_vel[j]), std::fabs(orig_vel[j])), v_brake);
        output_vel[j] = sgn * v_mag;
        output_lane.waypoints[j].twist.twist.linear.x = output_vel[j];
      }
    }

    // ------------------------------------------------
    // 3. Downstream: accelerate back from v_low toward original
    //    start from zero_start_idx (no 0 m/s zone here)
    //    v^2 = v_low^2 + 2 * a_acc * d
    // ------------------------------------------------
    if (a_acc > 0.0 && zero_start_idx < N)
    {
      double dist = 0.0;
      for (int j = zero_start_idx; j < N; ++j)
      {
        // distance from zero_start_idx along the lane
        if (j > zero_start_idx)
        {
          const geometry_msgs::Point& p0 = output_lane.waypoints[j - 1].pose.pose.position;
          const geometry_msgs::Point& p1 = output_lane.waypoints[j].pose.pose.position;
          dist += hypot(p1.x - p0.x, p1.y - p0.y);
        }

        const int sgn = (orig_vel[j] >= 0.0) ? 1 : -1;
        const double v_brake = std::sqrt(std::max(0.0, v_low_sq + 2.0 * a_acc * dist));
        if (std::fabs(orig_vel[j]) <= v_brake)
          break;

        const double v_mag = std::min(std::min(std::fabs(output_vel[j]), std::fabs(orig_vel[j])), v_brake);
        output_vel[j] = sgn * v_mag;
        output_lane.waypoints[j].twist.twist.linear.x = output_vel[j];
      }
    }
  }

  return output_lane;
}

// apply deceleration before stopline and acceleration after stopline
// // same as lane_rule.cpp
autoware_msgs::Lane apply_stopline_acceleration(const autoware_msgs::Lane& lane,
                                                const std::vector<size_t>& stopline_indexes, double acceleration,
                                                size_t ahead_cnt, size_t behind_cnt)
{
  autoware_msgs::Lane l = lane;
  if (stopline_indexes.empty())
    return l;

  for (const size_t i : stopline_indexes)
    l = apply_acceleration(l, acceleration, i, behind_cnt + 1, 0);

  // reverse in order to apply deceleration using apply_acceleration function
  std::reverse(l.waypoints.begin(), l.waypoints.end());

  std::vector<size_t> reverse_indexes;
  for (const size_t i : stopline_indexes)
    reverse_indexes.push_back(l.waypoints.size() - i - 1);
  std::reverse(reverse_indexes.begin(), reverse_indexes.end());

  for (const size_t i : reverse_indexes)
    l = apply_acceleration(l, acceleration, i, ahead_cnt + 1, 0);

  // undo reverse
  std::reverse(l.waypoints.begin(), l.waypoints.end());

  return l;
}

// create traffic_waypoint, red_waypoint, and green_waypoint
void create_waypoint(const autoware_msgs::LaneArray& msg)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = g_frame_id;
  g_cached_waypoint.lanes.clear();
  g_cached_waypoint.lanes.shrink_to_fit();
  g_cached_waypoint.id = msg.id;

  // update cache
  for (const autoware_msgs::Lane& l : msg.lanes)
    g_cached_waypoint.lanes.push_back(create_new_lane(l, header));

  // check validity of lanelet_map
  if (!g_loaded_lanelet_map || !g_lanelet_map || g_lanelet_map->laneletLayer.empty() ||
      g_lanelet_map->pointLayer.empty() || g_lanelet_map->regulatoryElementLayer.empty())
  {
    g_traffic_pub.publish(g_cached_waypoint);
    return;
  }

  autoware_msgs::LaneArray traffic_waypoint;
  autoware_msgs::LaneArray red_waypoint;
  autoware_msgs::LaneArray green_waypoint;
  traffic_waypoint.id = red_waypoint.id = green_waypoint.id = msg.id;
  // for each lane (lane is a collection of waypoints
  for (size_t i = 0; i < msg.lanes.size(); ++i)
  {
    autoware_msgs::Lane waypoint_lane = create_new_lane(msg.lanes[i], header);

    // choose to use vectors -> perhaps a lanelet data type better?
    std::vector<Eigen::Vector3d> waypoints;
    create_waypoint_array(waypoint_lane, &waypoints);
    if (waypoints.size() < 2)
    {
      traffic_waypoint.lanes.push_back(waypoint_lane);
      continue;
    }

    lanelet::Lanelets nearest_lanelets = find_nearest_lanelets(waypoints.front(), 1);
    if (nearest_lanelets.size() <= 0)
    {
      traffic_waypoint.lanes.push_back(waypoint_lane);
      continue;
    }

    // velocity smoothing
    for (int k = 0; k < g_config_number_of_smoothing_count; ++k)
    {
      autoware_msgs::Lane temp_lane = waypoint_lane;
      if (waypoint_lane.waypoints.size() >= 3)
      {
        for (size_t j = 1; j < waypoint_lane.waypoints.size() - 1; ++j)
        {
          if (waypoint_lane.waypoints.at(j).twist.twist.linear.x != 0)
          {
            waypoint_lane.waypoints[j].twist.twist.linear.x =
                (temp_lane.waypoints.at(j - 1).twist.twist.linear.x + temp_lane.waypoints.at(j).twist.twist.linear.x +
                 temp_lane.waypoints.at(j + 1).twist.twist.linear.x) /
                3;
          }
        }
      }
    }

    // check if any waypoint segments are intersected by stoplines
    std::vector<size_t> waypoint_stopline_indexes = check_waypoints_for_stoplines(waypoints);

    for (const size_t i : waypoint_stopline_indexes)
    {
      waypoint_lane.waypoints[i].wpstate.stop_state = autoware_msgs::WaypointState::TYPE_STOPLINE;
    }

    traffic_waypoint.lanes.push_back(waypoint_lane);
    // apply slowdown only to waypoint velocities to consider stoplines
    autoware_msgs::Lane green_waypoint_lane = waypoint_lane;
    green_waypoint_lane =
        apply_slowdown_only(waypoint_lane, waypoint_stopline_indexes, g_config_acceleration, g_config_acceleration,
                            g_config_number_of_zeros_behind, g_config_number_of_zeros_ahead,
                            g_config_number_of_slowdown, g_config_slowdown_velocity_scale);
    green_waypoint.lanes.push_back(green_waypoint_lane);
    // apply acceleration to waypoint velocities to consider stoplines
    autoware_msgs::Lane red_waypoint_lane = waypoint_lane;
    red_waypoint_lane =
        apply_stop_and_slowdown(waypoint_lane, waypoint_stopline_indexes, g_config_acceleration, g_config_acceleration,
                                g_config_number_of_zeros_behind, g_config_number_of_zeros_ahead,
                                g_config_number_of_slowdown, g_config_slowdown_velocity_scale);

    red_waypoint.lanes.push_back(red_waypoint_lane);
  }

  // publish traffic waypoint
  g_traffic_pub.publish(traffic_waypoint);
  g_red_pub.publish(red_waypoint);
  g_green_pub.publish(green_waypoint);
}

// callback for lanelet2 map.
void binMapCallback(const autoware_lanelet2_msgs::MapBin& msg)
{
  g_lanelet_map = std::make_shared<lanelet::LaneletMap>();

  lanelet::utils::conversion::fromBinMsg(msg, g_lanelet_map);
  g_loaded_lanelet_map = true;
  ROS_INFO("loaded lanelet map\n");

  if (!g_cached_waypoint.lanes.empty())
  {
    autoware_msgs::LaneArray update_waypoint = g_cached_waypoint;
    create_waypoint(update_waypoint);
  }
}

// callback for config topic
void config_parameter(const autoware_config_msgs::ConfigLaneRule& msg)
{
  g_config_acceleration = msg.acceleration;
  g_config_stopline_search_radius = msg.stopline_search_radius;
  g_config_number_of_zeros_ahead = msg.number_of_zeros_ahead;
  g_config_number_of_zeros_behind = msg.number_of_zeros_behind;
  g_config_number_of_smoothing_count = msg.number_of_smoothing_count;

  if (!g_cached_waypoint.lanes.empty())
  {
    autoware_msgs::LaneArray update_waypoint = g_cached_waypoint;
    create_waypoint(update_waypoint);
  }
}

//-------------------------------------------------------------------------
// follows logic of vector map based implmenentation
// but ignores acceleration adjustment for cross roads
// and does not reduce velocity for curvature
// assumed that this is implemented in local planner before
// path is published
// -> do only stopline acceleration control
//-------------------------------------------------------------------------
int main(int argc, char** argv)
{
  g_loaded_lanelet_map = false;

  ros::init(argc, argv, "lane_rule");
  ros::NodeHandle rosnode;
  ros::NodeHandle pnh("~");

  int sub_waypoint_queue_size;
  pnh.param<int>("sub_waypoint_queue_size", sub_waypoint_queue_size, 1);
  int sub_config_queue_size;
  pnh.param<int>("sub_config_queue_size", sub_config_queue_size, 1);
  int pub_waypoint_queue_size;
  pnh.param<int>("pub_waypoint_queue_size", pub_waypoint_queue_size, 1);
  bool pub_waypoint_latch;
  pnh.param<bool>("pub_waypoint_latch", pub_waypoint_latch, true);

  pnh.param<std::string>("frame_id", g_frame_id, "map");
  pnh.param<double>("acceleration", g_config_acceleration, 1);
  pnh.param<int>("number_of_smoothing_count", g_config_number_of_smoothing_count, 0);
  pnh.param<int>("number_of_zeros_ahead", g_config_number_of_zeros_ahead, 0);
  pnh.param<int>("number_of_zeros_behind", g_config_number_of_zeros_behind, 0);

  g_traffic_pub = rosnode.advertise<autoware_msgs::LaneArray>("traffic_waypoints_array", pub_waypoint_queue_size,
                                                              pub_waypoint_latch);
  g_red_pub =
      rosnode.advertise<autoware_msgs::LaneArray>("red_waypoints_array", pub_waypoint_queue_size, pub_waypoint_latch);
  g_green_pub =
      rosnode.advertise<autoware_msgs::LaneArray>("green_waypoints_array", pub_waypoint_queue_size, pub_waypoint_latch);

  ros::Subscriber bin_map_sub = rosnode.subscribe("lanelet_map_bin", 10, binMapCallback);
  ros::Subscriber waypoint_sub = rosnode.subscribe("lane_waypoints_array", sub_waypoint_queue_size, create_waypoint);
  ros::Subscriber config_sub = rosnode.subscribe("config/lane_rule", sub_config_queue_size, config_parameter);

  if (LANE_RULES_USE_ROUTING_GRAPH)
  {
    lanelet::traffic_rules::TrafficRulesPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    g_routing_graph = lanelet::routing::RoutingGraph::build(*g_lanelet_map, *traffic_rules);
  }

  ros::spin();

  return 0;
}
