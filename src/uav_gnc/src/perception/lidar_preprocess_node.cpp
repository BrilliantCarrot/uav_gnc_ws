#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class LidarPreprocessNode : public rclcpp::Node
{
public:
  LidarPreprocessNode() : Node("lidar_preprocess_node")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/lidar/points");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/lidar/points_filtered");

    voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.15);
    crop_z_min_ = this->declare_parameter<double>("crop_z_min", 0.0);
    crop_z_max_ = this->declare_parameter<double>("crop_z_max", 5.0);
    range_min_m_ = this->declare_parameter<double>("range_min_m", 0.5);
    range_max_m_ = this->declare_parameter<double>("range_max_m", 15.0);

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LidarPreprocessNode::cloudCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(),
      "[LiDAR Preprocess] input=%s output=%s leaf=%.2f z=[%.2f, %.2f] range=[%.2f, %.2f]",
      input_topic_.c_str(), output_topic_.c_str(), voxel_leaf_size_,
      crop_z_min_, crop_z_max_, range_min_m_, range_max_m_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud_raw);

    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*cloud_raw, *cloud_raw, valid_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud_raw);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(crop_z_min_, crop_z_max_);
    pass_z.filter(*cloud_z);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_range(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_range->reserve(cloud_z->size());

    for (const auto & pt : cloud_z->points) {
      const double r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if (r < range_min_m_ || r > range_max_m_) {
        continue;
      }
      cloud_range->points.push_back(pt);
    }

    cloud_range->width = static_cast<uint32_t>(cloud_range->points.size());
    cloud_range->height = 1;
    cloud_range->is_dense = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud_range);
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel.filter(*cloud_ds);

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*cloud_ds, out_msg);
    out_msg.header = msg->header;
    cloud_pub_->publish(out_msg);

    RCLCPP_DEBUG(this->get_logger(),
      "[LiDAR Preprocess] raw=%zu z_filtered=%zu range_filtered=%zu downsampled=%zu",
      cloud_raw->size(), cloud_z->size(), cloud_range->size(), cloud_ds->size());
  }

  std::string input_topic_;
  std::string output_topic_;

  double voxel_leaf_size_{0.15};
  double crop_z_min_{0.0};
  double crop_z_max_{5.0};
  double range_min_m_{0.5};
  double range_max_m_{15.0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPreprocessNode>());
  rclcpp::shutdown();
  return 0;
}
