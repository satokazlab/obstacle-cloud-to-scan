#ifndef PCL_PROCESSING_FUNCTIONS_H
#define PCL_PROCESSING_FUNCTIONS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>

// ダウンサンプリング
pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double voxel_leaf_size,
    bool use_gpu,
    rclcpp::Logger logger);

// パススルーフィルタ
pcl::PointCloud<pcl::PointXYZ>::Ptr applyPassThroughFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<double> &robot_box_size,
    rclcpp::Logger logger);

// 法線推定
pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    rclcpp::Logger logger);

// 法線から障害物を割り出し
pcl::PointCloud<pcl::PointXYZ>::Ptr filterObstacles(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    double max_slope_angle,
    rclcpp::Logger logger);

// ロボット自身のポイントクラウドを除去
pcl::PointCloud<pcl::PointXYZ>::Ptr removeRobotBody(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<double> &box_min,
    const std::vector<double> &box_max,
    rclcpp::Logger logger);
#endif // PCL_PROCESSING_FUNCTIONS_H
