#include "table_obstacle_layer/polygon_layer.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

using std::placeholders::_1;

namespace table_obstacle_layer
{

void PolygonLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in PolygonLayer::onInitialize");
  }

  rclcpp::Logger logger = node->get_logger();
  RCLCPP_INFO(logger, "Initializing PolygonLayer");

  // parameters (topic name, enabled)
  declareParameter("topic", rclcpp::ParameterValue(std::string("/table_obstacle")));
  declareParameter("enabled", rclcpp::ParameterValue(true));

  std::string topic = node->get_parameter("topic").as_string();
  enabled_ = node->get_parameter("enabled").as_bool();

  sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    topic, rclcpp::SensorDataQoS(),
    std::bind(&PolygonLayer::polygonCallback, this, _1));
}

void PolygonLayer::activate()
{
  has_polygon_ = false;
}

void PolygonLayer::deactivate()
{
  // nothing special
}

void PolygonLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  polygon_.clear();
  has_polygon_ = false;
}

void PolygonLayer::polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Expect polygon in 'map' frame (costmap frame). If not, user must publish map-frame polygons.
  polygon_.clear();
  for (auto &p : msg->polygon.points) {
    polygon_.push_back(p);
  }
  has_polygon_ = !polygon_.empty();

  // compute bounding box
  minx_ = miny_ = std::numeric_limits<double>::infinity();
  maxx_ = maxy_ = -std::numeric_limits<double>::infinity();
  for (auto &pt : polygon_) {
    if (pt.x < minx_) minx_ = pt.x;
    if (pt.y < miny_) miny_ = pt.y;
    if (pt.x > maxx_) maxx_ = pt.x;
    if (pt.y > maxy_) maxy_ = pt.y;
  }
}

void PolygonLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                                double *min_x, double *min_y, double *max_x, double *max_y)
{
  if (!enabled_) return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_polygon_) return;

  // expand the costmap bounds to include polygon bounding box
  if (minx_ < *min_x) *min_x = minx_;
  if (miny_ < *min_y) *min_y = miny_;
  if (maxx_ > *max_x) *max_x = maxx_;
  if (maxy_ > *max_y) *max_y = maxy_;
}

bool PolygonLayer::pointInPolygon(double x, double y)
{
  // ray casting algorithm
  bool inside = false;
  size_t n = polygon_.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    double xi = polygon_[i].x, yi = polygon_[i].y;
    double xj = polygon_[j].x, yj = polygon_[j].y;
    bool intersect = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
}

void PolygonLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                               int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_) return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_polygon_) return;

  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();
  double resolution = master_grid.getResolution();
  double origin_x = master_grid.getOriginX();
  double origin_y = master_grid.getOriginY();

  // iterate over costmap cells within polygon bounding box to reduce work
  unsigned int mx_min, my_min, mx_max, my_max;
  if (!master_grid.worldToMap(minx_, miny_, mx_min, my_min)) {
    int mx_tmp, my_tmp;
    master_grid.worldToMapNoBounds(minx_, miny_, mx_tmp, my_tmp);
    mx_min = static_cast<unsigned int>(mx_tmp);
    my_min = static_cast<unsigned int>(my_tmp);
  }
  if (!master_grid.worldToMap(maxx_, maxy_, mx_max, my_max)) {
    int mx_tmp, my_tmp;
    master_grid.worldToMapNoBounds(maxx_, maxy_, mx_tmp, my_tmp);
    mx_max = static_cast<unsigned int>(mx_tmp);
    my_max = static_cast<unsigned int>(my_tmp);
  }

  // make sure indices within grid
  mx_min = std::max(0u, mx_min);
  my_min = std::max(0u, my_min);
  mx_max = std::min(size_x - 1, mx_max);
  my_max = std::min(size_y - 1, my_max);

  for (unsigned int mx = mx_min; mx <= mx_max; ++mx) {
    for (unsigned int my = my_min; my <= my_max; ++my) {
      double wx = origin_x + (mx + 0.5) * resolution;
      double wy = origin_y + (my + 0.5) * resolution;
      if (pointInPolygon(wx, wy)) {
        master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

}  // namespace table_obstacle_layer

// export plugin
PLUGINLIB_EXPORT_CLASS(table_obstacle_layer::PolygonLayer, nav2_costmap_2d::Layer)
