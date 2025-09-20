#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

class Pc2Preprocessor : public rclcpp::Node {
public:
  Pc2Preprocessor();

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // 파라미터
  std::string input_topic_;
  std::string filtered_cloud_topic_;
  std::string grid_topic_;
  std::string map_frame_;
  double voxel_leaf_;                 // 다운샘플 크기(m)
  double z_min_, z_max_;              // Z 클립
  double ground_dist_thresh_;         // 바닥 분리 거리 임계(m)
  double grid_res_;                   // OccupancyGrid 해상도(m)
  double x_lim_, y_lim_;              // 투영 범위(+/- m)
  double inflate_radius_;             // grid 인플레이션(m)

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;

  // 내부 유틸
  nav_msgs::msg::OccupancyGrid makeGridMsg(const std_msgs::msg::Header &hdr,
                                           const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const;
};
