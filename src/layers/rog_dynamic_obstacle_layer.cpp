// Copyright 2026 RM27 Navigation Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_nav2_plugins/layers/rog_dynamic_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb_nav2_costmap_2d
{

void RogDynamicObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("ROG dynamic obstacle layer requires a live node");
  }

  clock_ = node->get_clock();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(topic_));
  declareParameter("obstacle_threshold", rclcpp::ParameterValue(obstacle_threshold_));
  declareParameter("stale_timeout", rclcpp::ParameterValue(stale_timeout_));
  declareParameter("tf_timeout", rclcpp::ParameterValue(tf_timeout_));
  declareParameter(
    "obstacle_marking_max_range", rclcpp::ParameterValue(obstacle_marking_max_range_));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".obstacle_threshold", obstacle_threshold_);
  node->get_parameter(name_ + ".stale_timeout", stale_timeout_);
  node->get_parameter(name_ + ".tf_timeout", tf_timeout_);
  node->get_parameter(name_ + ".obstacle_marking_max_range", obstacle_marking_max_range_);
  node->get_parameter(name_ + ".footprint_clearing_enabled", footprint_clearing_enabled_);

  enabled_ = enabled_ && std::isfinite(stale_timeout_) && stale_timeout_ > 0.0;
  stale_timeout_ = std::max(0.05, stale_timeout_);
  tf_timeout_ = std::max(0.0, std::isfinite(tf_timeout_) ? tf_timeout_ : 0.05);
  obstacle_marking_max_range_ =
    std::max(0.0, std::isfinite(obstacle_marking_max_range_) ? obstacle_marking_max_range_ : 0.0);
  obstacle_threshold_ = std::clamp(obstacle_threshold_, 1, 100);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_, qos, std::bind(&RogDynamicObstacleLayer::onGrid, this, std::placeholders::_1));
  matchSize();
  current_ = true;

  RCLCPP_INFO(
    logger_,
    "ROG dynamic obstacle layer enabled=%s topic=%s threshold=%d stale_timeout=%.2fs "
    "obstacle_marking_max_range=%.2fm (0=unlimited) footprint_clearing=%s",
    enabled_ ? "true" : "false", topic_.c_str(), obstacle_threshold_, stale_timeout_,
    obstacle_marking_max_range_, footprint_clearing_enabled_ ? "true" : "false");
}

void RogDynamicObstacleLayer::onGrid(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (
    msg->info.width == 0U || msg->info.height == 0U ||
    msg->data.size() != static_cast<size_t>(msg->info.width) * msg->info.height ||
    !std::isfinite(msg->info.resolution) || msg->info.resolution <= 0.0F) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Ignoring malformed ROG obstacle grid");
    return;
  }

  std::lock_guard<std::mutex> lock(grid_mutex_);
  latest_grid_ = std::move(msg);
  ++latest_grid_sequence_;
  latest_grid_rx_ = std::chrono::steady_clock::now();
}

void RogDynamicObstacleLayer::matchSize()
{
  nav2_costmap_2d::CostmapLayer::matchSize();
  resetMaps();
  clearance_override_mask_.assign(static_cast<size_t>(getSizeInCellsX()) * getSizeInCellsY(), 0U);
  have_projected_grid_ = false;
  projected_grid_sequence_ = 0U;
}

void RogDynamicObstacleLayer::reset()
{
  resetMaps();
  clearance_override_mask_.assign(static_cast<size_t>(getSizeInCellsX()) * getSizeInCellsY(), 0U);
  have_projected_grid_ = false;
  projected_grid_sequence_ = 0U;
  std::lock_guard<std::mutex> lock(grid_mutex_);
  latest_grid_.reset();
  latest_grid_sequence_ = 0U;
  latest_grid_rx_ = {};
}

bool RogDynamicObstacleLayer::transformPoint(
  const geometry_msgs::msg::Point & input, const geometry_msgs::msg::TransformStamped & transform,
  double & x, double & y) const
{
  geometry_msgs::msg::PointStamped in;
  geometry_msgs::msg::PointStamped out;
  in.header = transform.header;
  in.point = input;
  try {
    tf2::doTransform(in, out, transform);
  } catch (const tf2::TransformException &) {
    return false;
  }
  x = out.point.x;
  y = out.point.y;
  return std::isfinite(x) && std::isfinite(y);
}

bool RogDynamicObstacleLayer::rebuildFromLatestGrid(double robot_x, double robot_y)
{
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
  std::uint64_t sequence = 0U;
  std::chrono::steady_clock::time_point received;
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    grid = latest_grid_;
    sequence = latest_grid_sequence_;
    received = latest_grid_rx_;
  }

  if (!grid || sequence == 0U) {
    if (!warned_no_grid_) {
      RCLCPP_WARN(logger_, "ROG dynamic obstacle grid has not been received");
      warned_no_grid_ = true;
    }
    return false;
  }
  warned_no_grid_ = false;

  const double receive_age =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - received).count();
  if (!std::isfinite(receive_age) || receive_age > stale_timeout_) {
    // Never keep merging an old dynamic snapshot after its freshness budget
    // has elapsed.  Leaving have_projected_grid_ set here made the layer
    // continue feeding stale lethal cells to the master even though Nav2 had
    // already marked the layer non-current.
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> layer_lock(*getMutex());
    resetMaps();
    clearance_override_mask_.assign(static_cast<size_t>(getSizeInCellsX()) * getSizeInCellsY(), 0U);
    have_projected_grid_ = false;
    projected_grid_sequence_ = 0U;
    current_ = false;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "ROG dynamic obstacle grid stale (age=%.3fs timeout=%.3fs); cleared layer", receive_age,
      stale_timeout_);
    return false;
  }

  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  geometry_msgs::msg::TransformStamped transform;
  try {
    if (grid->header.frame_id.empty() || grid->header.frame_id == global_frame) {
      transform.header.frame_id = global_frame;
      transform.child_frame_id = global_frame;
      transform.transform.rotation.w = 1.0;
    } else {
      const rclcpp::Time stamp(grid->header.stamp, clock_->get_clock_type());
      transform = tf_->lookupTransform(
        global_frame, grid->header.frame_id, stamp, tf2::durationFromSec(tf_timeout_));
    }
  } catch (const tf2::TransformException & ex) {
    current_ = false;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "ROG obstacle grid TF %s -> %s unavailable: %s",
      grid->header.frame_id.c_str(), global_frame.c_str(), ex.what());
    return false;
  }

  // The costmap layer is read by the MINCO visualizer from a different
  // callback group than the Nav2 costmap update thread.  Protect the complete
  // reset-and-rebuild transaction; otherwise the visualizer can observe a
  // half-cleared layer and report (or publish) a false dynamic inflation
  // result while the master costmap is being merged.
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> layer_lock(*getMutex());
  resetMaps();
  clearance_override_mask_.assign(static_cast<size_t>(getSizeInCellsX()) * getSizeInCellsY(), 0U);
  size_t source_obstacles = 0U;
  size_t source_clearances = 0U;
  size_t marked_obstacles = 0U;
  size_t marked_clearances = 0U;
  size_t range_filtered_obstacles = 0U;
  size_t outside_costmap = 0U;
  size_t transform_failures = 0U;
  const bool limit_obstacle_range =
    obstacle_marking_max_range_ > 0.0 && std::isfinite(robot_x) && std::isfinite(robot_y);
  const double yaw = tf2::getYaw(grid->info.origin.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  for (uint32_t my = 0U; my < grid->info.height; ++my) {
    for (uint32_t mx = 0U; mx < grid->info.width; ++mx) {
      const int8_t value = grid->data[static_cast<size_t>(my) * grid->info.width + mx];
      const bool obstacle = value >= obstacle_threshold_;
      const bool clearance = value == 0;
      if (!obstacle && !clearance) {
        continue;
      }
      if (obstacle) {
        ++source_obstacles;
      } else {
        ++source_clearances;
      }
      const double lx = (static_cast<double>(mx) + 0.5) * grid->info.resolution;
      const double ly = (static_cast<double>(my) + 0.5) * grid->info.resolution;
      geometry_msgs::msg::Point point;
      point.x = grid->info.origin.position.x + c * lx - s * ly;
      point.y = grid->info.origin.position.y + s * lx + c * ly;
      point.z = 0.0;
      double wx = 0.0;
      double wy = 0.0;
      if (!transformPoint(point, transform, wx, wy)) {
        ++transform_failures;
        continue;
      }
      if (
        obstacle && limit_obstacle_range &&
        std::hypot(wx - robot_x, wy - robot_y) > obstacle_marking_max_range_) {
        ++range_filtered_obstacles;
        continue;
      }
      unsigned int costmap_x = 0U;
      unsigned int costmap_y = 0U;
      if (worldToMap(wx, wy, costmap_x, costmap_y)) {
        const size_t costmap_index = getIndex(costmap_x, costmap_y);
        if (obstacle) {
          setCost(costmap_x, costmap_y, nav2_costmap_2d::LETHAL_OBSTACLE);
          clearance_override_mask_[costmap_index] = 0U;
          ++marked_obstacles;
        } else if (getCost(costmap_x, costmap_y) != nav2_costmap_2d::LETHAL_OBSTACLE) {
          setCost(costmap_x, costmap_y, nav2_costmap_2d::FREE_SPACE);
          clearance_override_mask_[costmap_index] = 1U;
          ++marked_clearances;
        }
      } else {
        ++outside_costmap;
      }
    }
  }

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 5000,
    "ROG obstacle projection: sequence=%llu frame=%s->%s obstacles=%zu/%zu "
    "clearances=%zu/%zu range_filtered=%zu outside=%zu transform_failed=%zu",
    static_cast<unsigned long long>(sequence), grid->header.frame_id.c_str(), global_frame.c_str(),
    marked_obstacles, source_obstacles, marked_clearances, source_clearances,
    range_filtered_obstacles, outside_costmap, transform_failures);
  if (source_obstacles > range_filtered_obstacles && marked_obstacles == 0U) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "ROG grid contains %zu measured obstacle cells, but none project into the %s costmap "
      "(outside=%zu transform_failed=%zu)",
      source_obstacles, global_frame.c_str(), outside_costmap, transform_failures);
  }

  projected_grid_sequence_ = sequence;
  have_projected_grid_ = true;
  current_ = true;
  return marked_obstacles > 0U;
}

void RogDynamicObstacleLayer::clearRobotFootprint(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {
    return;
  }

  nav2_costmap_2d::transformFootprint(
    robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> layer_lock(*getMutex());
  for (const auto & point : transformed_footprint_) {
    touch(point.x, point.y, min_x, min_y, max_x, max_y);
  }
  setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
}

void RogDynamicObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  if (!enabled_) {
    current_ = true;
    return;
  }

  if (layered_costmap_->isRolling()) {
    const auto * master = layered_costmap_->getCostmap();
    updateOrigin(master->getOriginX(), master->getOriginY());
  }
  (void)rebuildFromLatestGrid(robot_x, robot_y);
  clearRobotFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

  // The layer is rebuilt each cycle. Cover the complete rolling window so the
  // master costmap cannot retain a cell from the previous ROG snapshot.
  const auto * master = layered_costmap_->getCostmap();
  *min_x = std::min(*min_x, master->getOriginX());
  *min_y = std::min(*min_y, master->getOriginY());
  *max_x = std::max(*max_x, master->getOriginX() + master->getSizeInMetersX());
  *max_y = std::max(*max_y, master->getOriginY() + master->getSizeInMetersY());
}

void RogDynamicObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  (void)min_i;
  (void)min_j;
  (void)max_i;
  (void)max_j;
  if (!enabled_ || !have_projected_grid_) {
    return;
  }

  // Do not rely on the bounds calculated by another layer.  This layer is a
  // snapshot overlay and its complete rolling window must be merged every
  // cycle; a stale/narrow bounds window otherwise leaves ROG cells out of the
  // master grid and lets SMAC route through them.  This is the same max merge
  // semantics as CostmapLayer::updateWithMax, with explicit accounting for
  // diagnostics and with a stable lock shared with the visualizer.
  // Keep lock ordering identical to the visualizer (master -> layer).  The
  // layer rebuild itself only takes the layer lock, so this also prevents a
  // visualizer/updateCosts inversion while a snapshot is committed.
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> master_lock(*master_grid.getMutex());
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> layer_lock(*getMutex());
  const unsigned int size_x = std::min(getSizeInCellsX(), master_grid.getSizeInCellsX());
  const unsigned int size_y = std::min(getSizeInCellsY(), master_grid.getSizeInCellsY());
  size_t source_lethal = 0U;
  size_t merged_lethal = 0U;
  size_t cleared_static = 0U;
  size_t master_lethal_after = 0U;
  for (unsigned int y = 0U; y < size_y; ++y) {
    for (unsigned int x = 0U; x < size_x; ++x) {
      const unsigned char layer_cost = getCost(x, y);
      const size_t cell_index = getIndex(x, y);
      if (
        cell_index < clearance_override_mask_.size() &&
        clearance_override_mask_[cell_index] != 0U &&
        layer_cost != nav2_costmap_2d::LETHAL_OBSTACLE) {
        if (master_grid.getCost(x, y) != nav2_costmap_2d::FREE_SPACE) {
          ++cleared_static;
        }
        master_grid.setCost(x, y, nav2_costmap_2d::FREE_SPACE);
        continue;
      }
      if (layer_cost == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }
      if (layer_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        ++source_lethal;
      }
      const unsigned char master_cost = master_grid.getCost(x, y);
      if (master_cost == nav2_costmap_2d::NO_INFORMATION || layer_cost > master_cost) {
        master_grid.setCost(x, y, layer_cost);
        if (layer_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
          ++merged_lethal;
        }
      }
      if (
        layer_cost == nav2_costmap_2d::LETHAL_OBSTACLE &&
        master_grid.getCost(x, y) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        ++master_lethal_after;
      }
    }
  }

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000,
    "ROG master merge: layer_lethal=%zu merged_lethal=%zu cleared_static=%zu "
    "master_lethal_after=%zu window=%ux%u bounds_ignored=true",
    source_lethal, merged_lethal, cleared_static, master_lethal_after, size_x, size_y);
}

}  // namespace pb_nav2_costmap_2d

PLUGINLIB_EXPORT_CLASS(pb_nav2_costmap_2d::RogDynamicObstacleLayer, nav2_costmap_2d::Layer)
