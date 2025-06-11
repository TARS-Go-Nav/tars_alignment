#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int8.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Dense>

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>
#include <random>
#include <sstream>
#include <filesystem>
#include <fstream>

#include "../utils/command_launcher.hpp"

namespace tars_alignment {

struct AlignmentResult {
    Eigen::Matrix4f transform;
    double fitness_score;
    double inlier_ratio;
    double translation_error;
    double orientation_error;
    bool has_converged;
};

class InitPoseAlignment : public rclcpp::Node 
{
public:
    InitPoseAlignment(const rclcpp::NodeOptions& node_options);

private:
    void preprocessMapCloud();
    
    void prepareMultiResolutionMaps();
    
    void publishMapCloud();
    
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr processAccumulatedClouds();
    
    AlignmentResult searchBestOrientation(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const Eigen::Matrix4f& initial_guess);

    AlignmentResult performNDTAlignment(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
        const Eigen::Matrix4f& initial_guess,
        int max_iterations,
        double max_correspondence_distance);
    
    AlignmentResult performICPAlignment(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
        const Eigen::Matrix4f& initial_guess,
        int max_iterations,
        double max_correspondence_distance);

    AlignmentResult performGICPAlignment(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
        const Eigen::Matrix4f& initial_guess,
        int max_iterations,
        double max_correspondence_distance);

    AlignmentResult multipleAlignmentAttempts(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const Eigen::Matrix4f& initial_guess);

    double calculateInlierRatio(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
        const Eigen::Matrix4f& transform,
        double distance_threshold);

    double calculateTranslationError(
        const Eigen::Matrix4f& transform1,
        const Eigen::Matrix4f& transform2);

    double calculateOrientationError(
        const Eigen::Matrix4f& transform1,
        const Eigen::Matrix4f& transform2);

    void publishTransform(const Eigen::Matrix4f& transform);

    float getRandomGaussian(float mean, float stddev);

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;                     // Target point cloud (map PCD)
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;  // Accumulated source point clouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> multi_res_maps_;      // Multi-resolution maps

    Eigen::Matrix4f initial_guess_;
    Eigen::Matrix4f transform_result_;
    Eigen::Vector3f initial_position_;

    std::mt19937 rand_gen_;  // Random number generator

    int accumulate_time_;
    int orientation_search_steps_;
    int multi_align_attempts_;
    double min_inlier_ratio_;
    double max_distance_;
    double voxel_leaf_size_;
    double fitness_score_threshold_;
    double good_fitness_score_threshold_;

    bool use_preprocess_ = false;

    bool use_radius_filter_ = false;
    double filter_radius_;

    bool use_initial_pose_ = false;

    bool use_multi_resolution_;
    bool use_icp_;

    bool use_cartesian_grid_ = false; 
    double grid_size_;

    bool use_height_threshold_ = false;
    double min_height_;
    double max_height_;

    double ndt_leaf_size_;                    // 0.5
    int ndt_max_iterations_;                  // 30
    double ndt_max_correspondence_distance_;  // 2.0

    double icp_leaf_size_;                    // 0.25
    int icp_max_iterations_;                  // 50
    double icp_max_correspondence_distance_;  // 0.5

    double gicp_leaf_size_;                   // 0.1
    int gicp_max_iterations_;                 // 50
    double gicp_max_correspondence_distance_; // 0.1

    bool has_initial_pose_ = false;           // Whether an initial pose is available
    bool has_aligned_ = false;                // Whether alignment is completed
    bool is_first_time_ = true;               // Whether it's the first alignment

    std::string map_frame_id_, odom_frame_id_, robot_frame_id_;

    // ROS Publisher
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filter_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr get_pose_pub_;

    // ROS Subscription
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

    // TF2
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ROS Timer
    rclcpp::TimerBase::SharedPtr map_timer_;

    CommandLauncher launcher_;
};

}  // namespace tars_alignment

RCLCPP_COMPONENTS_REGISTER_NODE(tars_alignment::InitPoseAlignment)
