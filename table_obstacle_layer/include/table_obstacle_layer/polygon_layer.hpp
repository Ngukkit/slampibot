#ifndef TABLE_OBSTACLE_LAYER__POLYGON_LAYER_HPP_
#define TABLE_OBSTACLE_LAYER__POLYGON_LAYER_HPP_

#include <memory>
#include <mutex>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace table_obstacle_layer
{

class PolygonLayer : public nav2_costmap_2d::Layer
{
public:
  PolygonLayer() = default;
  virtual ~PolygonLayer() = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double &min_x, double &min_y, double &max_x, double &max_y) override;
  void updateCosts(costmap_2d::Costmap2D &master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  void activate() override;
  void deactivate() override;
  void reset() override;

private:
  void polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  bool pointInPolygon(double x, double y);

  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_;
  std::mutex mutex_;
  std::vector<geometry_msgs::msg::Point32> polygon_; // in map frame
  bool has_polygon_{false};
  double minx_, miny_, maxx_, maxy_;
  bool enabled_{true};
};

}  // namespace table_obstacle_layer

#endif  // TABLE_OBSTACLE_LAYER__POLYGON_LAYER_HPP_
