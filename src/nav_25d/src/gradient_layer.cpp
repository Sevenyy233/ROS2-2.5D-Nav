#include "nav_25d/gradient_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav_25d
{

GradientLayer::GradientLayer()
{
  costmap_ = NULL; 
}

GradientLayer::~GradientLayer()
{
}

void GradientLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic_name", rclcpp::ParameterValue("/velodyne_points"));
  declareParameter("max_slope_limit", rclcpp::ParameterValue(0.5)); 
  declareParameter("slope_cost_factor", rclcpp::ParameterValue(100.0));
  
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "topic_name", topic_name_);
  node->get_parameter(name_ + "." + "max_slope_limit", max_slope_limit_);
  node->get_parameter(name_ + "." + "slope_cost_factor", slope_cost_factor_);

  rolling_window_ = layered_costmap_->isRolling();

  point_cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_name_, rclcpp::SensorDataQoS(),
    std::bind(&GradientLayer::pointCloudCallback, this, std::placeholders::_1));

  need_recalculation_ = false;
  current_ = true;
  
  matchSize();
}

void GradientLayer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!enabled_) {
    return;
  }
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  last_cloud_ = msg;
  need_recalculation_ = true;
}

void GradientLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) return;

  // Check rolling window status
  rolling_window_ = layered_costmap_->isRolling();
  
  if (rolling_window_) {
    // Reset grid because origin moves
    std::fill(height_grid_.begin(), height_grid_.end(), -1000.0f);
    matchSize(); // Ensure size is correct (Costmap2D handles resize but we need to verify)
  }

  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (!last_cloud_) return;
      cloud = last_cloud_;
      // We process the cloud. If rolling, we process every update.
      // If static, we might only process new clouds?
      // But updateBounds is called frequently.
      // We should only re-process if need_recalculation_ is true OR rolling window (origin changed).
      if (!rolling_window_ && !need_recalculation_) return;
  }

  // Transform and update grid
  std::string target_frame = layered_costmap_->getGlobalFrameID();
  
  try {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(target_frame, cloud->header.frame_id, 
                                       tf2::TimePointZero, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(node_.lock()->get_logger(), "TF Error: %s", ex.what());
      return;
    }
    
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*cloud, pcl_cloud);
    
    Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform);
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(pcl_cloud, transformed_cloud, eigen_transform);
    
    // Update grid
    unsigned int mx, my;
    double cloud_min_x = 1e9, cloud_min_y = 1e9, cloud_max_x = -1e9, cloud_max_y = -1e9;
    
    for (const auto & pt : transformed_cloud.points) {
      if (worldToMap(pt.x, pt.y, mx, my)) {
        unsigned int index = getIndex(mx, my);
        if (height_grid_[index] < -900.0f) {
             height_grid_[index] = pt.z;
        } else {
             if (pt.z > height_grid_[index]) {
               height_grid_[index] = pt.z;
             }
        }
        
        if (pt.x < cloud_min_x) cloud_min_x = pt.x;
        if (pt.x > cloud_max_x) cloud_max_x = pt.x;
        if (pt.y < cloud_min_y) cloud_min_y = pt.y;
        if (pt.y > cloud_max_y) cloud_max_y = pt.y;
      }
    }
    
    if (cloud_min_x < cloud_max_x) {
        *min_x = std::min(*min_x, cloud_min_x);
        *min_y = std::min(*min_y, cloud_min_y);
        *max_x = std::max(*max_x, cloud_max_x);
        *max_y = std::max(*max_y, cloud_max_y);
    }
    
    need_recalculation_ = false;

  } catch (std::exception & e) {
    RCLCPP_ERROR(node_.lock()->get_logger(), "Exception in updateBounds: %s", e.what());
  }
}

void GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  unsigned char * master_array = master_grid.getCharMap();
  double resolution = master_grid.getResolution();
  unsigned int size_x = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      unsigned int index = getIndex(i, j);
      
      float h_center = height_grid_[index];
      if (h_center < -900.0f) continue; 
      
      float current_max_slope = 0.0f;
      
      int dx[] = {1, -1, 0, 0};
      int dy[] = {0, 0, 1, -1};
      
      for (int k=0; k<4; k++) {
        int nx = i + dx[k];
        int ny = j + dy[k];
        
        if (nx >= 0 && nx < (int)size_x && ny >= 0 && ny < (int)size_y_) { // Corrected size_x usage
          unsigned int n_index = getIndex(nx, ny);
          float h_neighbor = height_grid_[n_index];
          
          if (h_neighbor > -900.0f) {
            float diff = std::abs(h_center - h_neighbor);
            float dist = resolution; 
            float slope = std::atan2(diff, dist); 
            if (slope > current_max_slope) current_max_slope = slope;
          }
        }
      }
      
      if (current_max_slope > max_slope_limit_) {
        master_array[index] = LETHAL_OBSTACLE;
      } else {
        int cost = (int)(current_max_slope * slope_cost_factor_);
        if (cost > 252) cost = 252;
        
        if (master_array[index] < cost) {
          master_array[index] = (unsigned char)cost;
        }
      }
    }
  }
}

void GradientLayer::onFootprintChanged()
{
}

}  // namespace nav_25d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav_25d::GradientLayer, nav2_costmap_2d::Layer)
