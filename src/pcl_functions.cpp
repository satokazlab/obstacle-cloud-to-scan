#include "obstacle_cloud_to_scan/pcl_functions.hpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double voxel_leaf_size,
    bool use_gpu,
    rclcpp::Logger logger)
{
    auto start_time = std::chrono::high_resolution_clock::now(); // 計測開始
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

    if (use_gpu)
    {
        // GPU processing can be implemented here
        RCLCPP_DEBUG(logger, "Using GPU for voxel grid filter (not implemented)");
    }
    else
    {
        RCLCPP_DEBUG(logger, "Using CPU for voxel grid filter");
        voxel_filter.filter(*downsampled_cloud);
    }

    auto end_time = std::chrono::high_resolution_clock::now(); // 計測終了
    std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
    //RCLCPP_INFO(logger, "downsamplePointCloud took %.2f ms", elapsed.count());

    return downsampled_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr applyPassThroughFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<double> &robot_box_size,
    rclcpp::Logger logger)
{
    auto start_time = std::chrono::high_resolution_clock::now(); // 計測開始

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.0, robot_box_size[2] + 0.3);
    pass.filter(*filtered_cloud);
    RCLCPP_DEBUG(logger, "Passthrough filter applied");

    auto end_time = std::chrono::high_resolution_clock::now(); // 計測終了
    std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
    //RCLCPP_INFO(logger, "applyPassThroughFilter took %.2f ms", elapsed.count());

    return filtered_cloud;
}

pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    rclcpp::Logger logger)
{
    auto start_time = std::chrono::high_resolution_clock::now(); // 計測開始

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(0.6);
    normal_estimation.compute(*normals);
    RCLCPP_DEBUG(logger, "Normal estimation completed");

    auto end_time = std::chrono::high_resolution_clock::now(); // 計測終了
    std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
    //RCLCPP_INFO(logger, "estimateNormals took %.2f ms", elapsed.count());

    return normals;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filterObstacles(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    double max_slope_angle,
    rclcpp::Logger logger)
{
    auto start_time = std::chrono::high_resolution_clock::now(); // 計測開始

    RCLCPP_DEBUG(logger, "Normal cloud size: %ld", cloud->points.size());
    RCLCPP_DEBUG(logger, "Max angle slope: %f", max_slope_angle);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        const auto &normal = normals->points[i];
        //double angle = std::acos(normal.normal_z) * 180.0 / M_PI;
        double angle = (90 - max_slope_angle) * M_PI / 180;
        double threshold_normal_z = std::sin(angle);
        //RCLCPP_DEBUG(logger, "angle: %f", angle);
        //RCLCPP_DEBUG(logger, "threshold_normal_z: %f", threshold_normal_z);
        if (normal.normal_z <= threshold_normal_z && normal.normal_z >= -threshold_normal_z)
        {
            filtered_cloud->points.push_back(cloud->points[i]);
        }
    }
    RCLCPP_DEBUG(logger, "Obstacle filtering completed");

    auto end_time = std::chrono::high_resolution_clock::now(); // 計測終了
    std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
//    RCLCPP_INFO(logger, "filterObstacles took %.2f ms", elapsed.count());

    return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr removeRobotBody(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<double> &box_position,
    const std::vector<double> &box_size,
    rclcpp::Logger logger)
{
    auto start_time = std::chrono::high_resolution_clock::now(); // 計測開始

    pcl::CropBox<pcl::PointXYZ> crop_box_filter;
    crop_box_filter.setInputCloud(cloud);

    // ボックスの範囲を設定
    Eigen::Vector4f min_point(-(box_size[0]/2)+box_position[0],
                              -(box_size[1]/2)+box_position[1], 
                              0.0+box_position[2], 1.0);
    Eigen::Vector4f max_point(box_size[0]/2+box_position[0], 
                              box_size[1]/2+box_position[1],
                              box_size[2]+box_position[2], 1.0);
    RCLCPP_DEBUG(logger, "min_point: %f, %f, %f", min_point[0], min_point[1], min_point[2]);
    RCLCPP_DEBUG(logger, "max_point: %f, %f, %f", max_point[0], max_point[1], max_point[2]);

    crop_box_filter.setMin(min_point);
    crop_box_filter.setMax(max_point);
    crop_box_filter.setNegative(true);  // ボックス内の点群を除去する

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    crop_box_filter.filter(*filtered_cloud);

    RCLCPP_DEBUG(logger, "Removed points within the robot body bounding box.");

    auto end_time = std::chrono::high_resolution_clock::now(); // 計測終了
    std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
    //RCLCPP_INFO(logger, "removeRobotBody took %.2f ms", elapsed.count());

    return filtered_cloud;
}
