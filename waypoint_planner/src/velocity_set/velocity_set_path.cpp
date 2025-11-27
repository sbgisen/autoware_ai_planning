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

void VelocitySetPath::changeWaypointsForDeceleration(double deceleration, int closest_waypoint, int obstacle_waypoint)
{
  int extra = 4;  // for safety

  // decelerate with constant deceleration
  for (int index = obstacle_waypoint + extra; index >= closest_waypoint; index--)
  {
    if (!checkWaypoint(index))
      continue;
    if (index > obstacle_waypoint)
    {
      // After obstacle_waypoint, set the speed of extra points to decelerate_vel_min_.
      updated_waypoints_.waypoints[index].twist.twist.linear.x = decelerate_vel_min_;
      continue;
    }
    // v = sqrt( (v0)^2 + 2ax )
    // Keep the car at decelerate_vel_min_ when approaching the obstacles.
    // without decelerate_vel_min_ term, changed_vel becomes zero if index == obstacle_waypoint.
    std::array<int, 2> range = { index, obstacle_waypoint };
    double changed_vel = calcChangedVelocity(decelerate_vel_min_, deceleration, range);

    double prev_vel = original_waypoints_.waypoints[index].twist.twist.linear.x;
    const int sgn = (prev_vel < 0) ? -1 : 1;
    updated_waypoints_.waypoints[index].twist.twist.linear.x = sgn * std::min(std::abs(prev_vel), changed_vel);
  }
}

void VelocitySetPath::avoidSuddenAcceleration(double deceleration, int closest_waypoint)
{
  for (int i = 0;; i++)
  {
    if (!checkWaypoint(closest_waypoint + i))
      return;

    // accelerate with constant acceleration
    // v = root((v0)^2 + 2ax)
    // Without velocity_offset_ term, changed_vel becomes current_vel_ when i == 0. For example, the car will not move
    // if current_vel_ == 0.
    std::array<int, 2> range = { closest_waypoint, closest_waypoint + i };
    double changed_vel = calcChangedVelocity(current_vel_, deceleration, range) + velocity_offset_;

    const double target_vel = updated_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x;
    // Don't exceed original velocity
    if (changed_vel > std::abs(target_vel))
      return;

    const int sgn = (target_vel < 0) ? -1 : 1;
    updated_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x = sgn * changed_vel;
  }

  return;
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
void VelocitySetPath::avoidSuddenDeceleration(double velocity_change_limit, double deceleration, int closest_waypoint)
{
  constexpr double epsilon = 1e-9;

  if (closest_waypoint < 0)
    return;
  if (!checkWaypoint(closest_waypoint))
    return;

  // Threshold used both to detect “stop points” and to freeze waypoints
  // whose original velocity is already effectively stopped.
  const double stop_thresh = std::max(1e-6, std::abs(decelerate_vel_min_));

  // Small helpers
  auto sign_of = [](double v) -> int { return (v > 0.0) ? 1 : ((v < 0.0) ? -1 : 0); };
  auto v_orig = [&](int idx) -> double { return original_waypoints_.waypoints[idx].twist.twist.linear.x; };
  auto v_upd_ref = [&](int idx) -> double& { return updated_waypoints_.waypoints[idx].twist.twist.linear.x; };

  // Write helpers (both preserve the original sign and freeze “small original” waypoints)

  // Lower-only: cap to proposed_mag (>=0). Original |v| < stop_thresh → do nothing.
  auto lower_preserve = [&](int idx, double proposed_mag) {
    double& v = v_upd_ref(idx);
    const double vo = v_orig(idx);
    const double mag_o = std::abs(vo);
    const int sgn_o = sign_of(vo);

    if (mag_o < stop_thresh)
    {
      v = vo;
      return;
    }  // freeze this waypoint

    const double cur_mag = std::abs(v);
    const double new_mag = std::min(cur_mag, std::max(0.0, proposed_mag));

    // Do not push below the stop threshold here; the decision to actually stop
    // is made by the “mandatory stop/flip” envelope logic.
    if (new_mag < stop_thresh)
      return;

    v = (sgn_o == 0) ? ((new_mag == 0.0) ? 0.0 : new_mag) : (sgn_o * new_mag);
  };

  // Raise-only: lift to proposed_mag (>=0). Original |v| < stop_thresh → do nothing.
  auto raise_preserve = [&](int idx, double proposed_mag) {
    double& v = v_upd_ref(idx);
    const double vo = v_orig(idx);
    const double mag_o = std::abs(vo);
    const int sgn_o = sign_of(vo);

    if (mag_o < stop_thresh)
    {
      v = vo;
      return;
    }  // freeze this waypoint

    const double cur_mag = std::abs(v);
    const double new_mag = std::max(cur_mag, std::max(0.0, proposed_mag));

    if (new_mag < stop_thresh)
      return;

    v = (sgn_o == 0) ? ((new_mag == 0.0) ? 0.0 : new_mag) : (sgn_o * new_mag);
  };

  // Snapshot
  const double current = current_vel_;
  const int sign_now = sign_of(current);
  const double closest_now = v_upd_ref(closest_waypoint);

  // ------------------------------
  // A) Detect first stop (|v| < stop_thresh) and first direction flip ahead
  // ------------------------------
  int stop_index = -1;
  for (int i = closest_waypoint; i < getNewWaypointsSize(); ++i)
  {
    if (!checkWaypoint(i))
      return;
    if (std::abs(v_upd_ref(i)) < stop_thresh)
    {
      stop_index = i;
      break;
    }
  }

  int dir_change_index = -1;
  if (sign_now != 0)
  {
    for (int i = closest_waypoint; i < getNewWaypointsSize(); ++i)
    {
      if (!checkWaypoint(i))
        return;
      const int sv = sign_of(v_upd_ref(i));
      if (sv != 0 && sv != sign_now)
      {
        dir_change_index = i;
        break;
      }
    }
  }

  // Choose nearest “stop target”: either the stop point or the flip point.
  int stop_target_index = -1;
  double stop_dist_from_closest = std::numeric_limits<double>::infinity();
  if (stop_index != -1)
  {
    stop_target_index = stop_index;
    stop_dist_from_closest = calcInterval(closest_waypoint, stop_index);
  }
  if (dir_change_index != -1)
  {
    const double d_flip = calcInterval(closest_waypoint, dir_change_index);
    if (d_flip < stop_dist_from_closest)
    {
      stop_target_index = dir_change_index;
      stop_dist_from_closest = d_flip;
    }
  }

  const bool is_switchback = (stop_target_index != -1 && stop_target_index == dir_change_index);
  const double v0_mag_for_stop = std::abs(v_upd_ref(closest_waypoint));
  const bool have_mandatory_stop = (stop_target_index != -1) && std::isfinite(stop_dist_from_closest) &&
                                   (stop_dist_from_closest > epsilon) && (v0_mag_for_stop > 0.0);

  // ------------------------------
  // B) Local cap at the closest waypoint
  //     – treat (current→closest) distance same as (closest→closest+1);
  //       cap downward only; do not flip sign; do not modify frozen waypoints.
  // ------------------------------
  double closest_len = 0.0;
  if (closest_waypoint + 1 < getNewWaypointsSize())
    closest_len = calcInterval(closest_waypoint, closest_waypoint + 1);

  if (closest_len > epsilon && std::abs(current) > epsilon)
  {
    const bool is_decel_step = (std::abs(closest_now) < std::abs(current)) || (current * closest_now < 0.0);
    if (is_decel_step)
    {
      const double a_lim = std::abs(velocity_change_limit);
      const double v0 = std::abs(current);
      const double v_allow = std::sqrt(std::max(0.0, v0 * v0 - 2.0 * a_lim * closest_len));
      lower_preserve(closest_waypoint, v_allow);
    }
  }

  // ------------------------------
  // C) Effective decel magnitude: raise above the nominal limit iff a mandatory
  //    stop/flip must be reached in the available distance.
  // ------------------------------
  double a_eff = std::abs(velocity_change_limit);
  if (have_mandatory_stop)
  {
    const double required = (v0_mag_for_stop * v0_mag_for_stop) / (2.0 * stop_dist_from_closest);
    a_eff = std::max(a_eff, required);
  }

  // ------------------------------
  // D) With mandatory stop/flip: traverse from stop_target back to closest
  //    using the tighter of forward/backward envelopes:
  //      v_fwd(s) = sqrt(max(0, v0^2 - 2 a_eff * s))
  //      v_bwd(d) = sqrt(max(0, 2 a_eff * d))
  //    Switchback rule: every index except the exact flip point must satisfy
  //    |v| >= |decelerate_vel_min_|.
  //    Writes are “lower-only”, preserving original sign and freezing originals
  //    below stop_thresh.
  // ------------------------------
  if (have_mandatory_stop)
  {
    const double switchback_floor = std::abs(decelerate_vel_min_);
    double d_acc = 0.0;

    for (int idx = stop_target_index; idx >= closest_waypoint; --idx)
    {
      if (!checkWaypoint(idx))
        return;

      const double d_rem = d_acc;
      const double s_acc = std::max(0.0, stop_dist_from_closest - d_rem);

      double v_fwd = std::sqrt(std::max(0.0, v0_mag_for_stop * v0_mag_for_stop - 2.0 * a_eff * s_acc));
      double v_bwd = std::sqrt(std::max(0.0, 2.0 * a_eff * d_rem));
      double v_env = std::min(v_fwd, v_bwd);

      if (is_switchback)
      {
        // Only the exact flip index may be zero; all others must respect the minimum flip speed.
        if (idx == stop_target_index)
        {
          v_env = 0.0;
        }
        else if (v_env < switchback_floor)
        {
          v_env = switchback_floor;
        }
      }
      else
      {
        // Pure stop case: make the target exactly zero.
        if (idx == stop_target_index)
          v_env = 0.0;
      }

      lower_preserve(idx, v_env);

      if (idx > closest_waypoint)
        d_acc += calcInterval(idx - 1, idx);
    }
    return;
  }

  // ------------------------------
  // E) No mandatory stop/flip: enforce pairwise decel limit backward.
  //    For each segment (i-1 → i), cap |v_{i-1}| ≤ sqrt(|v_i|^2 + 2 a_lim * ds).
  //    Writes are “lower-only”, preserving original sign and freezing originals
  //    below stop_thresh. Acceleration is not limited anywhere.
  // ------------------------------
  const int last_idx = getNewWaypointsSize() - 1;

  // Optional range trimming to direction-consistent section for minor savings.
  int range_end = last_idx;
  if (sign_now != 0)
  {
    for (int i = closest_waypoint + 1; i <= last_idx; ++i)
    {
      if (!checkWaypoint(i))
        return;
      const int sv = sign_of(v_upd_ref(i));
      if (sv != 0 && sv != sign_now)
      {
        range_end = i - 1;
        break;
      }
    }
  }

  // Start from the smallest-|v| index to reduce work.
  int end_idx = range_end;
  if (range_end > closest_waypoint)
  {
    int min_idx = closest_waypoint + 1;
    double min_mag = std::abs(v_upd_ref(min_idx));
    for (int i = min_idx + 1; i <= range_end; ++i)
    {
      if (!checkWaypoint(i))
        return;
      const double mag = std::abs(v_upd_ref(i));
      if (mag < min_mag)
      {
        min_mag = mag;
        min_idx = i;
      }
      if (min_mag < stop_thresh)
        break;
    }
    end_idx = min_idx;
  }

  const double a_lim = std::abs(deceleration);

  for (int i = end_idx; i > closest_waypoint; --i)
  {
    if (!checkWaypoint(i) || !checkWaypoint(i - 1))
      return;

    const double ds = calcInterval(i - 1, i);
    if (ds <= epsilon)
      continue;

    const double v_next = v_upd_ref(i);
    const double v_allow = std::sqrt(std::max(0.0, v_next * v_next + 2.0 * a_lim * ds));

    lower_preserve(i - 1, v_allow);
  }

  // ------------------------------
  // F) Forward feasibility pass: raise only when the current plan would require
  //    more than the decel limit to reach from current_vel_. Stop early once
  //    the plan becomes reachable. Writes preserve the original sign and freeze
  //    originals below stop_thresh.
  // ------------------------------
  int end_idx2 = last_idx;
  if (sign_now != 0)
  {
    for (int k = closest_waypoint + 1; k <= last_idx; ++k)
    {
      if (!checkWaypoint(k))
        return;
      const int sv = sign_of(v_upd_ref(k));
      if (sv != 0 && sv != sign_now)
      {
        end_idx2 = k - 1;
        break;
      }
    }
  }

  const double v0_mag2 = std::abs(current);
  double s_acc = 0.0;

  for (int i = closest_waypoint + 1; i <= end_idx2; ++i)
  {
    if (!checkWaypoint(i))
      return;
    s_acc += calcInterval(i - 1, i);

    const double v_min_mag = std::sqrt(std::max(0.0, v0_mag2 * v0_mag2 - 2.0 * a_lim * s_acc));

    const double cur_mag = std::abs(v_upd_ref(i));
    if (cur_mag + 1e-12 < v_min_mag)
    {
      // Lift only to the minimum reachable magnitude; sign preserved; freeze if original is “stopped”.
      raise_preserve(i, v_min_mag);
    }
    else
    {
      break;  // from here on, deceleration within limit is feasible
    }
  }
}

void VelocitySetPath::changeWaypointsForStopping(int stop_waypoint, int obstacle_waypoint, int closest_waypoint,
                                                 double deceleration)
{
  if (closest_waypoint < 0)
    return;

  // decelerate with constant deceleration
  for (int index = stop_waypoint; index >= closest_waypoint; index--)
  {
    if (!checkWaypoint(index))
      continue;

    // v = (v0)^2 + 2ax, and v0 = 0
    std::array<int, 2> range = { index, stop_waypoint };
    const double changed_vel = calcChangedVelocity(0.0, deceleration, range);
    const double prev_vel = original_waypoints_.waypoints[index].twist.twist.linear.x;
    const int sgn = (prev_vel < 0) ? -1 : 1;
    updated_waypoints_.waypoints[index].twist.twist.linear.x = sgn * std::min(std::abs(prev_vel), changed_vel);
  }

  // fill velocity with 0 for stopping waypoint and the rest.
  for (auto it = updated_waypoints_.waypoints.begin() + stop_waypoint; it != updated_waypoints_.waypoints.end(); ++it)
  {
    it->twist.twist.linear.x = 0.0;
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
