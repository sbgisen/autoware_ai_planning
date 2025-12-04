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

#include <waypoint_planner/velocity_set/velocity_set_path.h>

VelocitySetPath::VelocitySetPath()
{
  ros::NodeHandle private_nh_("~");
  private_nh_.param<double>("velocity_offset", velocity_offset_, 1.2);
  private_nh_.param<double>("decelerate_vel_min", decelerate_vel_min_, 1.3);
}

// check if waypoint number is valid
bool VelocitySetPath::checkWaypoint(int wp_num) const
{
  if (wp_num < 0 || wp_num >= getPrevWaypointsSize())
  {
    return false;
  }
  return true;
}

// set about '_temporal_waypoints_size' meter waypoints from closest waypoint
void VelocitySetPath::setTemporalWaypoints(int temporal_waypoints_size, int closest_waypoint,
                                           geometry_msgs::PoseStamped control_pose)
{
  if (closest_waypoint < 0)
    return;

  temporal_waypoints_.waypoints.clear();
  temporal_waypoints_.header = updated_waypoints_.header;
  temporal_waypoints_.increment = updated_waypoints_.increment;

  // push current pose
  autoware_msgs::Waypoint current_point;
  current_point.pose = control_pose;
  current_point.twist = updated_waypoints_.waypoints[closest_waypoint].twist;
  current_point.dtlane = updated_waypoints_.waypoints[closest_waypoint].dtlane;
  temporal_waypoints_.waypoints.push_back(std::move(current_point));

  int total_waypoints = getNewWaypointsSize();
  for (int i = 0; i < temporal_waypoints_size; i++)
  {
    if (closest_waypoint + i >= total_waypoints)
      return;

    temporal_waypoints_.waypoints.push_back(updated_waypoints_.waypoints[closest_waypoint + i]);
  }

  return;
}

double VelocitySetPath::calcChangedVelocity(const double& current_vel, const double& accel,
                                            const std::array<int, 2>& range) const
{
  static double current_velocity = current_vel;
  static double square_vel = current_vel * current_vel;
  if (current_velocity != current_vel)
  {
    current_velocity = current_vel;
    square_vel = current_vel * current_vel;
  }
  return std::sqrt(square_vel + 2.0 * accel * calcInterval(range.at(0), range.at(1)));
}

void VelocitySetPath::changeWaypointsForDeceleration(int decel_first_index, int decel_last_index, int closest_waypoint,
                                                     double deceleration)
{
  if (closest_waypoint < 0)
    return;

  // decelerate with constant deceleration
  for (int index = decel_last_index; index >= closest_waypoint; index--)
  {
    if (!checkWaypoint(index))
      continue;

    double original_vel = original_waypoints_.waypoints[index].twist.twist.linear.x;
    double previous_vel = updated_waypoints_.waypoints[index].twist.twist.linear.x;
    const int sgn = (original_vel < 0) ? -1 : 1;
    if (index > decel_first_index)
    {
      // After decel_last_index, set the speed of extra points to decelerate_vel_min_.
      if (fabs(previous_vel) > decelerate_vel_min_)
      {
        updated_waypoints_.waypoints[index].twist.twist.linear.x = sgn * decelerate_vel_min_;
      }
      continue;
    }
    // v = sqrt( (v0)^2 + 2ax )
    // Keep the car at decelerate_vel_min_ when approaching the obstacles.
    // without decelerate_vel_min_ term, changed_vel becomes zero if index == decel_last_index.
    std::array<int, 2> range = { index, decel_last_index };
    double changed_vel = calcChangedVelocity(decelerate_vel_min_, deceleration, range);
    if (fabs(changed_vel) < fabs(previous_vel))
    {
      updated_waypoints_.waypoints[index].twist.twist.linear.x = sgn * std::abs(changed_vel);
    }
  }
}

void VelocitySetPath::avoidSuddenAcceleration(double accel_limit, int closest_waypoint)
{
  constexpr double eps = 1e-9;

  if (closest_waypoint < 0)
    return;
  if (!checkWaypoint(closest_waypoint))
    return;

  const int N = getNewWaypointsSize();
  if (N <= 0)
    return;

  const double a_lim = std::max(0.0, accel_limit);
  if (a_lim <= eps)
    return;

  auto sign_of = [](double v) -> int { return (v > 0.0) ? 1 : ((v < 0.0) ? -1 : 0); };

  auto v_orig = [&](int i) -> double { return original_waypoints_.waypoints[i].twist.twist.linear.x; };

  auto v_upd = [&](int i) -> double& { return updated_waypoints_.waypoints[i].twist.twist.linear.x; };

  // ------------------------------------------------------------
  // Determine travel direction based on the first non-zero
  // original velocity after closest_waypoint.
  // We only smooth the region that matches this direction.
  // ------------------------------------------------------------
  int dir_sign = 0;
  for (int i = closest_waypoint; i < N; ++i)
  {
    if (!checkWaypoint(i))
      return;

    const int s = sign_of(v_orig(i));
    if (s != 0)
    {
      dir_sign = s;
      break;
    }
  }
  if (dir_sign == 0)
    return;  // All speeds are zero → nothing to process.

  // ------------------------------------------------------------
  // Step 1:
  // Clip the speed at closest_waypoint so that it never exceeds
  // the original waypoint speed (in magnitude).
  // ------------------------------------------------------------
  {
    double& v0 = v_upd(closest_waypoint);
    const double vo0 = v_orig(closest_waypoint);
    const int s0 = sign_of(vo0);

    if (s0 == dir_sign)
    {
      const double mag = std::min(std::fabs(v0), std::fabs(vo0));
      v0 = (dir_sign > 0) ? mag : -mag;
    }
  }

  // ------------------------------------------------------------
  // Step 2:
  // Forward pass to suppress sudden acceleration.
  //
  // For each waypoint i (i > closest_waypoint):
  //  - Stop if direction changes (original speed sign changes)
  //  - Clip updated speed so it does not exceed original speed
  //  - Compute maximum reachable speed from previous waypoint:
  //
  //        v_i^2 <= v_{i-1}^2 + 2 * accel_limit * distance
  //
  //    If updated speed is above this, reduce it.
  // ------------------------------------------------------------
  for (int i = closest_waypoint + 1; i < N; ++i)
  {
    if (!checkWaypoint(i) || !checkWaypoint(i - 1))
      return;

    const double vo = v_orig(i);
    const int s = sign_of(vo);

    // Stop smoothing if direction changes or becomes zero.
    if (s == 0 || s != dir_sign)
      break;

    double& v = v_upd(i);
    double& v_prev = v_upd(i - 1);

    // Clip updated velocity by original velocity magnitude.
    double mag = std::min(std::fabs(v), std::fabs(vo));
    v = (dir_sign > 0) ? mag : -mag;

    const double ds = calcInterval(i - 1, i);
    if (ds <= eps)
      continue;

    const double mag_prev = std::fabs(v_prev);
    const double vmax = std::sqrt(std::max(0.0, mag_prev * mag_prev + 2.0 * a_lim * ds));

    double mag_now = std::fabs(v);
    if (mag_now > vmax)
    {
      mag_now = vmax;
      v = (dir_sign > 0) ? mag_now : -mag_now;
    }
  }
}

/**
 * @brief Smooths a sudden speed drop by enforcing a deceleration limit, while preserving
 *        each waypoint’s original sign and freezing waypoints whose original speed
 *        magnitude is below the stop threshold. Acceleration is not limited.
 *
 * Core guarantees:
 *  - Never flip the sign of any waypoint velocity (0 is allowed).
 *  - If a waypoint’s ORIGINAL |v| < |decelerate_vel_min_|, do not modify that waypoint.
 *  - If a mandatory stop or direction flip exists ahead, allow using stronger decel
 *    than the nominal limit to guarantee stopping at that point.
 *  - In a switchback (flip) scenario, enforce |v| >= |decelerate_vel_min_| at all
 *    waypoints except the exact flip point, which is allowed to be 0.
 *
 * Heuristics:
 *  - The current→closest distance is approximated by the closest→closest+1 distance
 *    when limiting the closest waypoint’s speed. This avoids the constant-speed
 *    plateau and produces a smooth decay starting at the closest.
 *  - Pairwise backward pass caps upstream speeds to respect the decel limit into the
 *    next waypoint. A forward feasibility pass raises speeds only when they were
 *    lowered too much to be reachable within the limit.
 */
void VelocitySetPath::avoidSuddenDeceleration(double deceleration, int closest_waypoint)
{
  constexpr double eps = 1e-9;

  // ------------------------------------------------------------
  // How many waypoints we allow to look backward for smoothing.
  // This enlarges the smoothing region so deceleration is not
  // only corrected after closest_waypoint but also slightly before.
  // ------------------------------------------------------------
  const int max_decel_lookback_waypoints = 10;

  if (closest_waypoint < 0)
    return;
  if (!checkWaypoint(closest_waypoint))
    return;

  const int N = getNewWaypointsSize();
  if (N <= 0)
    return;

  const double a_lim = std::fabs(deceleration);
  if (a_lim <= eps)
    return;

  auto sign_of = [](double v) -> int { return (v > 0.0) ? 1 : ((v < 0.0) ? -1 : 0); };

  auto v_orig = [&](int i) -> double { return original_waypoints_.waypoints[i].twist.twist.linear.x; };

  auto v_upd = [&](int i) -> double& { return updated_waypoints_.waypoints[i].twist.twist.linear.x; };

  // ------------------------------------------------------------
  // Determine travel direction from the first non-zero original
  // velocity at or after closest_waypoint.
  // ------------------------------------------------------------
  int dir_sign = 0;
  for (int i = closest_waypoint; i < N; ++i)
  {
    if (!checkWaypoint(i))
      return;

    int s = sign_of(v_orig(i));
    if (s != 0)
    {
      dir_sign = s;
      break;
    }
  }
  if (dir_sign == 0)
    return;

  // ------------------------------------------------------------
  // Define smoothing range:
  //   backward: up to max_decel_lookback_waypoints
  //   forward: until direction changes or list ends
  // ------------------------------------------------------------
  int start_idx = std::max(0, closest_waypoint - max_decel_lookback_waypoints);

  int end_idx = -1;
  std::vector<double> allowed(N, 0.0);

  for (int i = start_idx; i < N; ++i)
  {
    if (!checkWaypoint(i))
      return;

    double vo = v_orig(i);
    int s = sign_of(vo);

    if (s == 0 || s != dir_sign)
      break;

    double mag_org = std::fabs(vo);
    double mag_upd = std::fabs(v_upd(i));

    allowed[i] = std::min(mag_upd, mag_org);
    end_idx = i;
  }

  if (end_idx < 0)
    return;

  // ------------------------------------------------------------
  // Backward pass to enforce deceleration limit:
  //
  //   v_i^2 <= v_{i+1}^2 + 2 * a_lim * ds
  //
  // We only reduce upstream velocities (never increase).
  // ------------------------------------------------------------
  for (int i = end_idx - 1; i >= start_idx; --i)
  {
    if (!checkWaypoint(i) || !checkWaypoint(i + 1))
      return;

    double ds = calcInterval(i, i + 1);
    if (ds <= eps)
      continue;

    double v_down = allowed[i + 1];
    double vmax = std::sqrt(std::max(0.0, v_down * v_down + 2.0 * a_lim * ds));

    if (allowed[i] > vmax)
      allowed[i] = vmax;
  }

  // ------------------------------------------------------------
  // Write back results:
  // - Apply only within [start_idx .. end_idx]
  // - Maintain direction sign
  // - Ensure updated speed never exceeds original
  // ------------------------------------------------------------
  for (int i = start_idx; i <= end_idx; ++i)
  {
    double& v = v_upd(i);
    double vo = v_orig(i);
    int s = sign_of(vo);

    if (s == 0)
    {
      v = 0.0;
      continue;
    }

    double mag = allowed[i];
    v = (s > 0) ? mag : -mag;
  }
}

void VelocitySetPath::limitDecelerationFromCurrentVelocity(double velocity_change_limit, int closest_waypoint)
{
  constexpr double eps = 1e-6;

  if (closest_waypoint < 0 || !checkWaypoint(closest_waypoint))
    return;

  const int N = getNewWaypointsSize();
  if (N <= 0)
    return;

  const double v_curr = current_vel_;
  if (std::fabs(v_curr) < eps)
    return;

  const double a_limit = std::max(0.0, velocity_change_limit);
  if (a_limit < eps)
    return;

  auto sign_of = [](double v) -> int { return (v > 0.0) ? 1 : ((v < 0.0) ? -1 : 0); };

  const int dir = sign_of(v_curr);
  if (dir == 0)
    return;

  auto v_orig = [&](int i) -> double { return original_waypoints_.waypoints[i].twist.twist.linear.x; };

  auto v_upd = [&](int i) -> double& { return updated_waypoints_.waypoints[i].twist.twist.linear.x; };

  // Assume that the velocity at the previous waypoint of closest_waypoint
  // was equal to current_vel_.
  const int anchor_index = (closest_waypoint > 0) ? (closest_waypoint - 1) : closest_waypoint;

  // Fixed threshold to detect "stop point" / "near zero".
  const double stop_threshold = 1e-3;  // [m/s]

  // ------------------------------------------------------------
  // Case 3:
  //   "We already must have been stopped" is defined as:
  //   original velocity at closest_waypoint is (almost) zero.
  //
  //   → Do nothing. The profile already means "stop here".
  // ------------------------------------------------------------
  if (std::fabs(v_orig(closest_waypoint)) <= stop_threshold)
  {
    return;
  }

  // ------------------------------------------------------------
  // Step 1: find stop/turn point AHEAD of closest_waypoint.
  //   - near zero speed in original path (|v_orig| <= stop_threshold)
  //   - or direction change w.r.t current velocity
  //
  // Distance is measured from anchor_index (where we assume v_curr).
  // ------------------------------------------------------------
  int stop_index_ahead = -1;
  double dist_to_stop = 0.0;

  {
    double accum = 0.0;

    for (int i = anchor_index; i < N; ++i)
    {
      if (!checkWaypoint(i))
        return;

      if (i > anchor_index)
      {
        if (!checkWaypoint(i - 1))
          return;
        accum += calcInterval(i - 1, i);
      }

      // Only care about stop/turn points at or after closest_waypoint.
      if (i < closest_waypoint)
        continue;

      const double vo = v_orig(i);
      const int s = sign_of(vo);

      const bool is_stop = std::fabs(vo) <= stop_threshold;
      const bool is_turn = (s != 0 && s != dir);

      if (is_stop || is_turn)
      {
        stop_index_ahead = i;
        dist_to_stop = accum;  // distance from anchor_index to this stop/turn
        break;
      }
    }
  }

  // ------------------------------------------------------------
  // Step 2:
  //   If there is a stop/turn point ahead, check whether
  //   velocity_change_limit is enough to reach v=0 at that point
  //   when we assume v_curr at anchor_index.
  // ------------------------------------------------------------
  bool need_stronger_decel = false;
  double a_required = 0.0;

  if (stop_index_ahead >= 0 && dist_to_stop > eps)
  {
    const double v0 = std::fabs(v_curr);
    // Magnitude of constant deceleration to reach 0 at stop_index_ahead
    a_required = (v0 * v0) / (2.0 * dist_to_stop);

    if (a_required > a_limit + 1e-6)
      need_stronger_decel = true;
  }

  // ------------------------------------------------------------
  // Case 2:
  //   We are heading toward a stop/turn point and a_limit is NOT
  //   enough to reach 0 there.
  //
  //   → Allow stronger deceleration and build a constant-decel
  //     profile that hits v=0 exactly at stop_index_ahead.
  // ------------------------------------------------------------
  if (need_stronger_decel)
  {
    const double v0 = std::fabs(v_curr);
    const double a = a_required;

    // Distance from anchor_index to closest_waypoint
    double base_dist = 0.0;
    if (closest_waypoint > anchor_index)
    {
      for (int j = anchor_index + 1; j <= closest_waypoint; ++j)
      {
        if (!checkWaypoint(j) || !checkWaypoint(j - 1))
          return;
        base_dist += calcInterval(j - 1, j);
      }
    }

    double accum = base_dist;

    for (int i = closest_waypoint; i <= stop_index_ahead; ++i)
    {
      if (!checkWaypoint(i))
        return;

      if (i > closest_waypoint)
      {
        if (!checkWaypoint(i - 1))
          return;
        accum += calcInterval(i - 1, i);
      }

      const double v_sq = std::max(0.0, v0 * v0 - 2.0 * a * accum);
      const double v_mag = std::sqrt(v_sq);

      v_upd(i) = (dir > 0) ? v_mag : -v_mag;
    }
    // After stop_index_ahead we keep the existing profile.
    return;
  }

  // ------------------------------------------------------------
  // Case 1:
  //   Either:
  //     - there is no stop/turn point ahead, or
  //     - a_limit is sufficient to stop before it if needed.
  //
  //   In this case we only enforce:
  //     "deceleration magnitude <= a_limit"
  //
  //   We build a lower-bound envelope:
  //
  //       v_env(s)^2 = v_curr^2 - 2 * a_limit * s
  //
  //   where s is the distance from anchor_index
  //   (closest_waypoint-1, or 0 if closest_waypoint==0).
  //
  //   For each waypoint at distance s from anchor, if |v(i)| is
  //   below v_env(s) we RAISE it up to v_env(s), BUT ONLY IF:
  //     - original speed there is not near zero
  //     - updated speed there is not near zero
  //     - sign(new_speed) == sign(original_speed)
  //     - |new_speed| >= |current updated speed|
  //
  //   We NEVER zero velocities here, and if no point violates the
  //   envelope, the profile is left completely unchanged.
  //
  //   When a stop/turn point exists ahead and a_limit is enough,
  //   we DO NOT touch the stop point itself
  //   (we stop smoothing at stop_index_ahead - 1).
  // ------------------------------------------------------------
  const double v0 = std::fabs(v_curr);

  int last_index = N - 1;
  if (stop_index_ahead >= 0)
  {
    last_index = stop_index_ahead - 1;
  }

  if (last_index < closest_waypoint)
  {
    // Nothing to smooth.
    return;
  }

  // Distance from anchor_index to closest_waypoint
  double base_dist = 0.0;
  if (closest_waypoint > anchor_index)
  {
    for (int j = anchor_index + 1; j <= closest_waypoint; ++j)
    {
      if (!checkWaypoint(j) || !checkWaypoint(j - 1))
        return;
      base_dist += calcInterval(j - 1, j);
    }
  }

  double accum = base_dist;

  for (int i = closest_waypoint; i <= last_index; ++i)
  {
    if (!checkWaypoint(i))
      return;

    if (i > closest_waypoint)
    {
      if (!checkWaypoint(i - 1))
        return;
      accum += calcInterval(i - 1, i);
    }

    const double v_env_sq = std::max(0.0, v0 * v0 - 2.0 * a_limit * accum);
    const double v_env = std::sqrt(v_env_sq);

    double& v = v_upd(i);
    const double vo = v_orig(i);

    double mag = std::fabs(v);
    const double mag_orig = std::fabs(vo);
    const int s_orig = sign_of(vo);
    const int s_v = sign_of(v);

    // Do not modify near-zero original speeds (keep stop points etc.).
    if (mag_orig <= stop_threshold)
      continue;

    // Do not "wake up" segments that are already (almost) stopped.
    if (mag <= stop_threshold)
      continue;

    // If current updated velocity sign is different from original sign,
    // do not interfere (likely another logic such as switchback).
    if (s_orig != 0 && s_v != 0 && s_v != s_orig)
      break;

    // If the current speed already satisfies the decel envelope, nothing to do.
    if (mag >= v_env)
      continue;

    // Raise speed, but:
    //  - keep the original sign
    //  - never make it slower than the current updated speed (monotonic)
    double new_mag = v_env;
    if (new_mag < mag)
      new_mag = mag;

    if (s_orig == 0)
    {
      // If original sign is zero, we avoid changing direction.
      // Just keep current velocity as is.
      continue;
    }

    v = (s_orig > 0) ? new_mag : -new_mag;
  }
}

void VelocitySetPath::changeWaypointsForStopping(int stop_first_index, int stop_last_index, int closest_waypoint,
                                                 double deceleration)
{
  if (closest_waypoint < 0)
    return;

  // decelerate with constant deceleration
  for (int index = stop_last_index; index >= closest_waypoint; index--)
  {
    if (!checkWaypoint(index))
      continue;

    double original_vel = original_waypoints_.waypoints[index].twist.twist.linear.x;
    double previous_vel = updated_waypoints_.waypoints[index].twist.twist.linear.x;
    const int sgn = (original_vel < 0) ? -1 : 1;
    if (index > stop_first_index)
    {
      updated_waypoints_.waypoints[index].twist.twist.linear.x = 0.0;
      continue;
    }
    // v = (v0)^2 + 2ax, and v0 = 0
    std::array<int, 2> range = { index, stop_first_index };
    const double changed_vel = calcChangedVelocity(0.0, deceleration, range);
    if (fabs(changed_vel) < fabs(previous_vel))
    {
      updated_waypoints_.waypoints[index].twist.twist.linear.x = sgn * std::abs(changed_vel);
    }
  }
}

void VelocitySetPath::initializeNewWaypoints()
{
  updated_waypoints_ = original_waypoints_;
}

double VelocitySetPath::calcInterval(const int begin, const int end) const
{
  // check index
  if (begin < 0 || begin >= getPrevWaypointsSize() || end < 0 || end >= getPrevWaypointsSize() || begin > end)
  {
    ROS_WARN("Invalid input index range: begin = %d, end = %d, PrevWaypointsSize = %d", begin, end,
             getPrevWaypointsSize());
    return 0.0;
  }

  // Calculate the inteval of waypoints
  double dist_sum = 0.0;
  for (int i = begin; i < end; i++)
  {
    tf::Vector3 v1(original_waypoints_.waypoints[i].pose.pose.position.x,
                   original_waypoints_.waypoints[i].pose.pose.position.y, 0);

    tf::Vector3 v2(original_waypoints_.waypoints[i + 1].pose.pose.position.x,
                   original_waypoints_.waypoints[i + 1].pose.pose.position.y, 0);

    dist_sum += tf::tfDistance(v1, v2);
  }

  return dist_sum;
}

void VelocitySetPath::resetFlag()
{
  set_path_ = false;
}

void VelocitySetPath::waypointsCallback(const autoware_msgs::LaneConstPtr& msg)
{
  original_waypoints_ = *msg;
  // temporary, edit waypoints velocity later
  updated_waypoints_ = *msg;

  set_path_ = true;
}

void VelocitySetPath::currentVelocityCallback(const geometry_msgs::TwistStampedConstPtr& msg)
{
  current_vel_ = msg->twist.linear.x;
}
