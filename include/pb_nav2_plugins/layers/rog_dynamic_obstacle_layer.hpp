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

#ifndef PB_NAV2_PLUGINS__LAYERS__ROG_DYNAMIC_OBSTACLE_LAYER_HPP_
#define PB_NAV2_PLUGINS__LAYERS__ROG_DYNAMIC_OBSTACLE_LAYER_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb_nav2_costmap_2d
{

/**
 * @brief Rolling costmap overlay fed by ROG's evidence-only obstacle grid.
 *
 * The layer owns a separate Costmap2D.  It is rebuilt from the newest ROG
 * snapshot on every costmap cycle, so an obstacle which leaves the sensor
 * window cannot remain in the master costmap indefinitely.  The layer is
 * intentionally placed before Nav2's inflation layer.
 */
class RogDynamicObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  RogDynamicObstacleLayer() { costmap_ = nullptr; }

  ~RogDynamicObstacleLayer() override = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;

  void matchSize() override;
  void reset() override;
  bool isClearable() override { return true; }

private:
  void onGrid(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  bool rebuildFromLatestGrid(double robot_x, double robot_y);
  void clearRobotFootprint(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y);
  bool transformPoint(
    const geometry_msgs::msg::Point & input, const geometry_msgs::msg::TransformStamped & transform,
    double & x, double & y) const;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Clock::SharedPtr clock_;

  mutable std::mutex grid_mutex_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_grid_;
  std::uint64_t latest_grid_sequence_{0U};
  std::uint64_t projected_grid_sequence_{0U};
  std::chrono::steady_clock::time_point latest_grid_rx_{};

  std::string topic_{"/rog_map/dynamic_obstacles"};
  int obstacle_threshold_{65};
  double stale_timeout_{0.75};
  double tf_timeout_{0.05};
  double obstacle_marking_max_range_{0.0};
  bool enabled_{true};
  bool footprint_clearing_enabled_{true};
  bool have_projected_grid_{false};
  bool warned_no_grid_{false};
  std::vector<uint8_t> clearance_override_mask_;
  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
};

}  // namespace pb_nav2_costmap_2d

#endif  // PB_NAV2_PLUGINS__LAYERS__ROG_DYNAMIC_OBSTACLE_LAYER_HPP_
