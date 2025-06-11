#include "init_pose_alignment.hpp"

namespace tars_alignment {

InitPoseAlignment::InitPoseAlignment(const rclcpp::NodeOptions& node_options) : Node("init_pose_alignment", node_options) {
    this->declare_parameter("target_pcd_file", "test.pcd");

    this->declare_parameter("map_frame_id", "map");
    this->declare_parameter("odom_frame_id", "odom_init");
    this->declare_parameter("robot_frame_id", "base_frame");

    this->declare_parameter("accumulate_time", 15);
    this->declare_parameter("orientation_search_steps", 18);
    this->declare_parameter("multi_align_attempts", 8);
    this->declare_parameter("max_distance", 30.0);
    this->declare_parameter("voxel_leaf_size", 0.1);
    this->declare_parameter("fitness_score_threshold", 5.0);
    this->declare_parameter("good_fitness_score_threshold", 0.5);

    this->declare_parameter("use_preprocess", false);
    this->declare_parameter("use_radius_filter", false);
    this->declare_parameter("filter_radius", 1.0);

    this->declare_parameter("use_initial_pose", true);
    this->declare_parameter("initial_pose.position.x", 0.0);
    this->declare_parameter("initial_pose.position.y", 0.0);
    this->declare_parameter("initial_pose.position.z", 0.0);
    this->declare_parameter("initial_pose.orientation.x", 0.0);
    this->declare_parameter("initial_pose.orientation.y", 0.0);
    this->declare_parameter("initial_pose.orientation.z", 0.0);
    this->declare_parameter("initial_pose.orientation.w", 1.0);

    this->declare_parameter("use_multi_resolution", true);
    this->declare_parameter("use_icp", false);

    this->declare_parameter("use_cartesian_grid", false);
    this->declare_parameter("grid_size", 0.5);

    this->declare_parameter("use_height_threshold", false);
    this->declare_parameter("min_height", -1.0);
    this->declare_parameter("max_height", 3.0);

    this->declare_parameter("ndt_leaf_size", 0.5);
    this->declare_parameter("ndt_max_iterations", 30);
    this->declare_parameter("ndt_max_correspondence_distance", 2.0);

    this->declare_parameter("icp_leaf_size", 0.25);
    this->declare_parameter("icp_max_iterations", 50);
    this->declare_parameter("icp_max_correspondence_distance", 0.5);

    this->declare_parameter("gicp_leaf_size", 0.1);
    this->declare_parameter("gicp_max_iterations", 100);
    this->declare_parameter("gicp_max_correspondence_distance", 0.1);

    std::string target_pcd_file = this->get_parameter("target_pcd_file").as_string();

    map_frame_id_ = this->get_parameter("map_frame_id").as_string();
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    robot_frame_id_ = this->get_parameter("robot_frame_id").as_string();

    accumulate_time_ = this->get_parameter("accumulate_time").as_int();
    orientation_search_steps_ = this->get_parameter("orientation_search_steps").as_int();
    multi_align_attempts_ = this->get_parameter("multi_align_attempts").as_int();
    max_distance_ = this->get_parameter("max_distance").as_double();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    fitness_score_threshold_ = this->get_parameter("fitness_score_threshold").as_double();
    good_fitness_score_threshold_ = this->get_parameter("good_fitness_score_threshold").as_double();

    use_preprocess_ = this->get_parameter("use_preprocess").as_bool();
    use_radius_filter_ = this->get_parameter("use_radius_filter").as_bool();
    filter_radius_ = this->get_parameter("filter_radius").as_double();

    use_initial_pose_ = this->get_parameter("use_initial_pose").as_bool();

    use_multi_resolution_ = this->get_parameter("use_multi_resolution").as_bool();
    use_icp_ = this->get_parameter("use_icp").as_bool();

    use_cartesian_grid_ = this->get_parameter("use_cartesian_grid").as_bool();
    grid_size_ = this->get_parameter("grid_size").as_double();

    use_height_threshold_ = this->get_parameter("use_height_threshold").as_bool();
    min_height_ = this->get_parameter("min_height").as_double();
    max_height_ = this->get_parameter("max_height").as_double();

    ndt_leaf_size_ = this->get_parameter("ndt_leaf_size").as_double();
    ndt_max_iterations_ = this->get_parameter("ndt_max_iterations").as_int();
    ndt_max_correspondence_distance_ = this->get_parameter("ndt_max_correspondence_distance").as_double();

    icp_leaf_size_ = this->get_parameter("icp_leaf_size").as_double();
    icp_max_iterations_ = this->get_parameter("icp_max_iterations").as_int();
    icp_max_correspondence_distance_ = this->get_parameter("icp_max_correspondence_distance").as_double();

    gicp_leaf_size_ = this->get_parameter("gicp_leaf_size").as_double();
    gicp_max_iterations_ = this->get_parameter("gicp_max_iterations").as_int();
    gicp_max_correspondence_distance_ = this->get_parameter("gicp_max_correspondence_distance").as_double();

    target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(target_pcd_file, *target_cloud_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map point cloud file: %s", target_pcd_file.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Map point cloud loaded successfully. Point count: %zu", target_cloud_->size());

    preprocessMapCloud();

    initial_guess_ = Eigen::Matrix4f::Identity();
    transform_result_ = Eigen::Matrix4f::Identity();

    std::random_device rd;
    rand_gen_.seed(rd());

    tf_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/tf_map_to_odom", 5);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/map", 10);
    filter_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filter_map", 10);
    get_pose_pub_ = this->create_publisher<std_msgs::msg::Int8>("/get_pose", 10);

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/pointcloud", rclcpp::SensorDataQoS(), 
        std::bind(&InitPoseAlignment::lidarCallback, this, std::placeholders::_1));

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, 
        std::bind(&InitPoseAlignment::initialPoseCallback, this, std::placeholders::_1));

    map_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), [this]() {
        publishMapCloud();
    });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    if (use_multi_resolution_) {
        prepareMultiResolutionMaps();
    }

    if (use_initial_pose_) {
        double x = this->get_parameter("initial_pose.position.x").as_double();
        double y = this->get_parameter("initial_pose.position.y").as_double();
        double z = this->get_parameter("initial_pose.position.z").as_double();

        double qx = this->get_parameter("initial_pose.orientation.x").as_double();
        double qy = this->get_parameter("initial_pose.orientation.y").as_double();
        double qz = this->get_parameter("initial_pose.orientation.z").as_double();
        double qw = this->get_parameter("initial_pose.orientation.w").as_double();

        Eigen::Quaternionf q(qw, qx, qy, qz);
        Eigen::Matrix3f rotation = q.toRotationMatrix();

        initial_guess_ = Eigen::Matrix4f::Identity();
        initial_guess_.block<3, 3>(0, 0) = rotation;
        initial_guess_(0, 3) = x;
        initial_guess_(1, 3) = y;
        initial_guess_(2, 3) = z;
        initial_position_ = Eigen::Vector3f(x, y, z);

        has_initial_pose_ = true;
    }

    RCLCPP_INFO(this->get_logger(), "Node initialization complete. Waiting for point cloud data...");
}


void InitPoseAlignment::preprocessMapCloud() {
    if (target_cloud_->empty()) {
        RCLCPP_WARN(this->get_logger(), "Map point cloud is empty, cannot preprocess.");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_grid.setInputCloud(target_cloud_);
    voxel_grid.filter(*processed_cloud);

    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud(processed_cloud);
    // sor.setMeanK(50);
    // sor.setStddevMulThresh(1.0);
    // sor.filter(*processed_cloud);

    if (use_height_threshold_) {
        pcl::CropBox<pcl::PointXYZ> crop_box;
        crop_box.setInputCloud(processed_cloud);
        crop_box.setMin(Eigen::Vector4f(-max_distance_, -max_distance_, min_height_, 1.0));
        crop_box.setMax(Eigen::Vector4f(max_distance_, max_distance_, max_height_, 1.0));
        crop_box.filter(*processed_cloud);
    }

    *target_cloud_ = *processed_cloud;
    RCLCPP_INFO(this->get_logger(), "Map point cloud preprocessing completed. Downsampled points: %zu", target_cloud_->size());
}


void InitPoseAlignment::prepareMultiResolutionMaps() {
    multi_res_maps_.clear();
    std::vector<double> leaf_sizes = {ndt_leaf_size_, icp_leaf_size_, gicp_leaf_size_};

    for (double leaf_size : leaf_sizes) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_map(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_grid.setInputCloud(target_cloud_);
        voxel_grid.filter(*downsampled_map);

        multi_res_maps_.push_back(downsampled_map);
        RCLCPP_INFO(this->get_logger(), "Multi-resolution map - leaf size: %.2f, points: %zu",
                    leaf_size, downsampled_map->size());
    }
}


void InitPoseAlignment::publishMapCloud() {
    if (has_aligned_) {
        return;
    }
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*target_cloud_, map_msg);
    map_msg.header.frame_id = map_frame_id_;
    map_msg.header.stamp = this->now();
    map_pub_->publish(map_msg);
}


void InitPoseAlignment::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received initial pose.");

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    Eigen::Quaternionf q(qw, qx, qy, qz);
    Eigen::Matrix3f rotation = q.toRotationMatrix();

    initial_guess_ = Eigen::Matrix4f::Identity();
    initial_guess_.block<3, 3>(0, 0) = rotation;
    initial_guess_(0, 3) = x;
    initial_guess_(1, 3) = y;
    initial_guess_(2, 3) = z;
    initial_position_ = Eigen::Vector3f(x, y, z);
    RCLCPP_INFO(this->get_logger(), "Initial pose: x=%.2f, y=%.2f, z=%.2f", x, y, z);

    has_initial_pose_ = true;
    has_aligned_ = false;
    is_first_time_ = true;
    accumulated_clouds_.clear();
}


void InitPoseAlignment::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (has_aligned_) {
        publishTransform(transform_result_);
        return;
    }
    std_msgs::msg::Int8 get_pose;

    // Process only if not aligned yet or a new initial pose is available
    if (!has_aligned_ || (has_initial_pose_ && is_first_time_)) {
        auto start_time = std::chrono::high_resolution_clock::now();

        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *source_cloud);
        get_pose.data = 0;
        get_pose_pub_->publish(get_pose);

        // Accumulate incoming point clouds
        if (accumulated_clouds_.size() < accumulate_time_) {
            accumulated_clouds_.push_back(source_cloud);
            RCLCPP_INFO(this->get_logger(), "Accumulating point clouds %zu/%d", accumulated_clouds_.size(), accumulate_time_);
            return;
        } else {
            accumulated_clouds_.erase(accumulated_clouds_.begin());
            accumulated_clouds_.push_back(source_cloud);
        }

        // Process accumulated clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud = processAccumulatedClouds();

        // Downsample
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_grid.setInputCloud(processed_cloud);
        voxel_grid.filter(*downsampled_cloud);

        std::vector<AlignmentResult> alignment_results;

        // Perform alignment
        if (has_initial_pose_ && is_first_time_) {
            RCLCPP_INFO(this->get_logger(), "Using initial pose. Performing orientation search and multi-trial alignment.");

            // ******************** Orientation Alignment ********************
            AlignmentResult orientation_result = searchBestOrientation(downsampled_cloud, initial_guess_);
            alignment_results.emplace_back(orientation_result);
            Eigen::Matrix4f current_transform = orientation_result.transform;

            if (orientation_result.fitness_score < good_fitness_score_threshold_) {
                RCLCPP_INFO(this->get_logger(), "Orientation search found a good match. Proceeding to next stage.");
            }

            // Perform multi-resolution alignment
            if (!multi_res_maps_.empty() && use_multi_resolution_) {
                // ******************** NDT Alignment ********************
                pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_downsampled_source(new pcl::PointCloud<pcl::PointXYZ>());
                voxel_grid.setLeafSize(ndt_leaf_size_, ndt_leaf_size_, ndt_leaf_size_);
                voxel_grid.setInputCloud(source_cloud);
                voxel_grid.filter(*ndt_downsampled_source);

                pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_pcd_map = multi_res_maps_[0];

                AlignmentResult ndt_result = 
                    performNDTAlignment(ndt_downsampled_source, ndt_pcd_map, current_transform, 
                                        ndt_max_iterations_, ndt_max_correspondence_distance_);
                
                if (ndt_result.has_converged) {
                    alignment_results.emplace_back(ndt_result);
                    current_transform = ndt_result.transform;
                    RCLCPP_INFO(this->get_logger(), "NDT stage alignment succeeded. Score: %.4f", ndt_result.fitness_score);
                } else {
                    RCLCPP_WARN(this->get_logger(), "NDT stage did not converge. Keeping previous result.");
                }

                // ******************** ICP Alignment ********************
                if (use_icp_) {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_downsampled_source(new pcl::PointCloud<pcl::PointXYZ>());
                    voxel_grid.setLeafSize(icp_leaf_size_, icp_leaf_size_, icp_leaf_size_);
                    voxel_grid.setInputCloud(source_cloud);
                    voxel_grid.filter(*icp_downsampled_source);

                    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_pcd_map = multi_res_maps_[1];

                    AlignmentResult icp_result = 
                        performICPAlignment(icp_downsampled_source, icp_pcd_map, current_transform, 
                                            icp_max_iterations_, icp_max_correspondence_distance_);
                    
                    if (icp_result.has_converged) {
                        alignment_results.emplace_back(icp_result);
                        current_transform = icp_result.transform;
                        RCLCPP_INFO(this->get_logger(), "ICP stage alignment succeeded. Score: %.4f", icp_result.fitness_score);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "ICP stage did not converge. Keeping previous result.");
                    }
                }

                // ******************** GICP Alignment ********************
                pcl::PointCloud<pcl::PointXYZ>::Ptr gicp_downsampled_source(new pcl::PointCloud<pcl::PointXYZ>());
                voxel_grid.setLeafSize(gicp_leaf_size_, gicp_leaf_size_, gicp_leaf_size_);
                voxel_grid.setInputCloud(source_cloud);
                voxel_grid.filter(*gicp_downsampled_source);

                pcl::PointCloud<pcl::PointXYZ>::Ptr gicp_pcd_map = multi_res_maps_[2];

                AlignmentResult gicp_result = 
                    performGICPAlignment(gicp_downsampled_source, gicp_pcd_map, current_transform, 
                                         gicp_max_iterations_, gicp_max_correspondence_distance_);
                
                if (gicp_result.has_converged) {
                    alignment_results.emplace_back(gicp_result);
                    current_transform = gicp_result.transform;
                    RCLCPP_INFO(this->get_logger(), "GICP stage alignment succeeded. Score: %.4f", gicp_result.fitness_score);
                } else {
                    RCLCPP_WARN(this->get_logger(), "GICP stage did not converge. Keeping previous result.");
                }

            } else {
                RCLCPP_WARN(this->get_logger(), "Multi-resolution maps not prepared or multi-resolution disabled. Using fallback alignment.");
                AlignmentResult normal_result = multipleAlignmentAttempts(downsampled_cloud, current_transform);
                alignment_results.emplace_back(normal_result);
            }

            is_first_time_ = false;

        } else {
            // No initial pose, start from identity matrix with multiple trials
            RCLCPP_INFO(this->get_logger(), "No initial pose. Performing multiple trials from identity transform.");
            Eigen::Matrix4f current_transform = Eigen::Matrix4f::Identity();

            if (!multi_res_maps_.empty() && use_multi_resolution_) {
                // ******************** NDT Alignment ********************
                pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_downsampled_source(new pcl::PointCloud<pcl::PointXYZ>());
                voxel_grid.setLeafSize(ndt_leaf_size_, ndt_leaf_size_, ndt_leaf_size_);
                voxel_grid.setInputCloud(source_cloud);
                voxel_grid.filter(*ndt_downsampled_source);

                pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_pcd_map = multi_res_maps_[0];

                AlignmentResult ndt_result = 
                    performNDTAlignment(ndt_downsampled_source, ndt_pcd_map, current_transform, 
                                        ndt_max_iterations_, ndt_max_correspondence_distance_);
                
                if (ndt_result.has_converged) {
                    alignment_results.emplace_back(ndt_result);
                    current_transform = ndt_result.transform;
                    RCLCPP_INFO(this->get_logger(), "NDT stage alignment succeeded. Score: %.4f", ndt_result.fitness_score);
                } else {
                    RCLCPP_WARN(this->get_logger(), "NDT stage did not converge. Keeping previous result.");
                }

                // ******************** ICP Alignment ********************
                if (use_icp_) {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_downsampled_source(new pcl::PointCloud<pcl::PointXYZ>());
                    voxel_grid.setLeafSize(icp_leaf_size_, icp_leaf_size_, icp_leaf_size_);
                    voxel_grid.setInputCloud(source_cloud);
                    voxel_grid.filter(*icp_downsampled_source);

                    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_pcd_map = multi_res_maps_[1];

                    AlignmentResult icp_result = 
                        performICPAlignment(icp_downsampled_source, icp_pcd_map, current_transform, 
                                            icp_max_iterations_, icp_max_correspondence_distance_);
                    
                    if (icp_result.has_converged) {
                        alignment_results.emplace_back(icp_result);
                        current_transform = icp_result.transform;
                        RCLCPP_INFO(this->get_logger(), "ICP stage alignment succeeded. Score: %.4f", icp_result.fitness_score);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "ICP stage did not converge. Keeping previous result.");
                    }
                }

                // ******************** GICP Alignment ********************
                pcl::PointCloud<pcl::PointXYZ>::Ptr gicp_downsampled_source(new pcl::PointCloud<pcl::PointXYZ>());
                voxel_grid.setLeafSize(gicp_leaf_size_, gicp_leaf_size_, gicp_leaf_size_);
                voxel_grid.setInputCloud(source_cloud);
                voxel_grid.filter(*gicp_downsampled_source);

                pcl::PointCloud<pcl::PointXYZ>::Ptr gicp_pcd_map = multi_res_maps_[2];

                AlignmentResult gicp_result = 
                    performGICPAlignment(gicp_downsampled_source, gicp_pcd_map, current_transform, 
                                         gicp_max_iterations_, gicp_max_correspondence_distance_);
                
                if (gicp_result.has_converged) {
                    alignment_results.emplace_back(gicp_result);
                    current_transform = gicp_result.transform;
                    RCLCPP_INFO(this->get_logger(), "GICP stage alignment succeeded. Score: %.4f", gicp_result.fitness_score);
                } else {
                    RCLCPP_WARN(this->get_logger(), "GICP stage did not converge. Keeping previous result.");
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Multi-resolution map not prepared or disabled. Using fallback alignment.");
                AlignmentResult normal_result = multipleAlignmentAttempts(downsampled_cloud, Eigen::Matrix4f::Identity());
                alignment_results.emplace_back(normal_result);
            }
        }

        // Select best result with lowest fitness score
        auto best_result = std::min_element(alignment_results.begin(), alignment_results.end(),
            [](const AlignmentResult& a, const AlignmentResult& b) {
                return a.fitness_score < b.fitness_score;
            });

        if (best_result->fitness_score < fitness_score_threshold_) {
            has_aligned_ = true;
            RCLCPP_INFO(this->get_logger(), "Alignment successful. Score: %.4f", best_result->fitness_score);
        }

        if (best_result->fitness_score < good_fitness_score_threshold_) {
            RCLCPP_INFO(this->get_logger(), "Alignment is very good.");
        }

        // Apply the best transformation result
        transform_result_ = best_result->transform;
        publishTransform(transform_result_);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        // std::cout << "Using time: " << ((double)duration.count()) << " ms" << std::endl;

        get_pose.data = 1;
        get_pose_pub_->publish(get_pose);

        // std::string point_lio_bash = "/home/muzs/point_lio.sh";
        // launcher_.executeExistedBashCommand(point_lio_bash);
    } else {
        // Already aligned, just publish the current transform
        publishTransform(transform_result_);
        get_pose.data = 1;
        get_pose_pub_->publish(get_pose);
    }
}


pcl::PointCloud<pcl::PointXYZ>::Ptr InitPoseAlignment::processAccumulatedClouds() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (use_cartesian_grid_) {
        std::map<std::tuple<int, int, int>, pcl::PointCloud<pcl::PointXYZ>::Ptr> cartesian_grid;
        cartesian_grid.clear();

        for (const auto& cloud : accumulated_clouds_) {
            for (const auto& point : *cloud) {
                double dist = std::sqrt(point.x * point.x + point.y * point.y);
                if (dist > max_distance_ || point.z < min_height_ || point.z > max_height_) {
                    continue;
                }

                int grid_x = static_cast<int>(std::floor(point.x / grid_size_));
                int grid_y = static_cast<int>(std::floor(point.y / grid_size_));
                int grid_z = static_cast<int>(std::floor(point.z / grid_size_));

                auto grid_key = std::make_tuple(grid_x, grid_y, grid_z);

                if (cartesian_grid.find(grid_key) == cartesian_grid.end()) {
                    cartesian_grid[grid_key] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
                }

                cartesian_grid[grid_key]->push_back(point);
            }
        }

        for (const auto& grid_item : cartesian_grid) {
            if (grid_item.second->size() >= 3) {
                pcl::PointXYZ centroid(0, 0, 0);
                for (const auto& point : *(grid_item.second)) {
                    centroid.x += point.x;
                    centroid.y += point.y;
                    centroid.z += point.z;
                }

                centroid.x /= grid_item.second->size();
                centroid.y /= grid_item.second->size();
                centroid.z /= grid_item.second->size();

                result_cloud->push_back(centroid);
            }
        }
        RCLCPP_INFO(this->get_logger(), "Used Cartesian grid. Number of extracted features: %zu", result_cloud->size());

    } else {
        for (const auto& cloud : accumulated_clouds_) {
            *result_cloud += *cloud;
        }
        RCLCPP_INFO(this->get_logger(), "Cartesian grid not used. Merged point cloud size: %zu", result_cloud->size());
    }

    if (use_radius_filter_) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_result(new pcl::PointCloud<pcl::PointXYZ>());
        filtered_result->reserve(result_cloud->size());

        for (const auto& point : *result_cloud) {
            double dist2 = point.x * point.x + point.y * point.y;
            if (dist2 >= 1.0 * 1.0) {
                filtered_result->push_back(point);
            }
        }
        return filtered_result;
    }

    return result_cloud;
}


AlignmentResult InitPoseAlignment::searchBestOrientation(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const Eigen::Matrix4f& initial_guess) {

    std::vector<AlignmentResult> orientation_results;

    Eigen::Vector3f position(initial_guess(0, 3), initial_guess(1, 3), initial_guess(2, 3));

    for (int i = 0; i < orientation_search_steps_; ++i) {
        float angle = i * (2.0f * M_PI / orientation_search_steps_);

        Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f rotation_matrix = rotation.toRotationMatrix();

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = rotation_matrix;
        transform.block<3, 1>(0, 3) = position;

        // Use ICP for quick alignment evaluation
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaximumIterations(20);  // Use fewer iterations for speed
        icp.setTransformationEpsilon(1e-6);
        icp.setMaxCorrespondenceDistance(1.0);
        icp.setInputTarget(target_cloud_);
        icp.setInputSource(source_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
        icp.align(*aligned, transform);

        AlignmentResult result;
        result.transform = icp.getFinalTransformation();
        result.fitness_score = icp.getFitnessScore();
        result.has_converged = icp.hasConverged();
        result.orientation_error = calculateOrientationError(result.transform, transform);

        orientation_results.push_back(result);

        RCLCPP_DEBUG(this->get_logger(), "Orientation search %d/%d, angle: %.1f deg, score: %.4f",
                    i+1, orientation_search_steps_, angle * 180.0f / M_PI, result.fitness_score);
    }

    // Select the best result with lowest fitness score
    auto best_result = std::min_element(orientation_results.begin(), orientation_results.end(),
        [](const AlignmentResult& a, const AlignmentResult& b) {
            return a.fitness_score < b.fitness_score;
        });

    int best_index = std::distance(orientation_results.begin(), best_result);
    float best_angle = best_index * (2.0f * M_PI / orientation_search_steps_);

    RCLCPP_INFO(this->get_logger(), "Best orientation found: angle %.1f deg, score: %.4f",
                best_angle * 180.0f / M_PI, best_result->fitness_score);

    return *best_result;
}


AlignmentResult InitPoseAlignment::performNDTAlignment(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    const Eigen::Matrix4f& initial_guess,
    int max_iterations,
    double max_correspondence_distance) {

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setMaximumIterations(max_iterations);
    ndt.setTransformationEpsilon(1e-8);
    ndt.setResolution(0.5);
    ndt.setStepSize(0.1);

    ndt.setInputTarget(target_cloud);
    ndt.setInputSource(source_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
    auto start_time = std::chrono::high_resolution_clock::now();
    ndt.align(*aligned, initial_guess);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    RCLCPP_DEBUG(this->get_logger(), "NDT alignment time: %ld ms", duration.count());

    AlignmentResult result;
    result.transform = ndt.getFinalTransformation();
    result.fitness_score = ndt.getFitnessScore();
    result.has_converged = ndt.hasConverged();
    result.orientation_error = calculateOrientationError(result.transform, initial_guess_);
    result.translation_error = calculateTranslationError(result.transform, initial_guess_);

    double inlier_ratio = calculateInlierRatio(source_cloud, target_cloud,
                                               result.transform, max_correspondence_distance);
    result.inlier_ratio = inlier_ratio;

    return result;
}


AlignmentResult InitPoseAlignment::performICPAlignment(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    const Eigen::Matrix4f& initial_guess,
    int max_iterations,
    double max_correspondence_distance) {

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(max_iterations);
    icp.setTransformationEpsilon(1e-8);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance);
    icp.setRANSACOutlierRejectionThreshold(0.05);
    icp.setEuclideanFitnessEpsilon(1e-6);

    icp.setInputTarget(target_cloud);
    icp.setInputSource(source_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
    auto start_time = std::chrono::high_resolution_clock::now();
    icp.align(*aligned, initial_guess);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    RCLCPP_DEBUG(this->get_logger(), "ICP alignment time: %ld ms", duration.count());

    double inlier_ratio = calculateInlierRatio(source_cloud, target_cloud,
                                               icp.getFinalTransformation(), max_correspondence_distance);

    AlignmentResult result;
    result.transform = icp.getFinalTransformation();
    result.fitness_score = icp.getFitnessScore();
    result.inlier_ratio = inlier_ratio;
    result.has_converged = icp.hasConverged();
    result.orientation_error = calculateOrientationError(result.transform, initial_guess_);
    result.translation_error = calculateTranslationError(result.transform, initial_guess_);

    return result;
}


AlignmentResult InitPoseAlignment::performGICPAlignment(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    const Eigen::Matrix4f& initial_guess,
    int max_iterations,
    double max_correspondence_distance) {

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaximumIterations(max_iterations);
    gicp.setTransformationEpsilon(1e-8);
    gicp.setMaxCorrespondenceDistance(max_correspondence_distance);
    gicp.setRANSACOutlierRejectionThreshold(0.05);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setCorrespondenceRandomness(20);
    gicp.setMaximumOptimizerIterations(20);
    gicp.setInputTarget(target_cloud);
    gicp.setInputSource(source_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
    auto start_time = std::chrono::high_resolution_clock::now();
    gicp.align(*aligned, initial_guess);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    RCLCPP_DEBUG(this->get_logger(), "GICP alignment time: %ld ms", duration.count());

    double inlier_ratio = calculateInlierRatio(source_cloud, target_cloud,
                                               gicp.getFinalTransformation(), max_correspondence_distance);

    AlignmentResult result;
    result.transform = gicp.getFinalTransformation();
    result.fitness_score = gicp.getFitnessScore();
    result.inlier_ratio = inlier_ratio;
    result.has_converged = gicp.hasConverged();
    result.orientation_error = calculateOrientationError(result.transform, initial_guess_);
    result.translation_error = calculateTranslationError(result.transform, initial_guess_);

    return result;
}


double InitPoseAlignment::calculateInlierRatio(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    const Eigen::Matrix4f& transform,
    double distance_threshold) {

    if (source_cloud->empty()) {
        return 0.0;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(target_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform);

    int inlier_count = 0;
    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);

    for (const auto& point : *transformed_cloud) {
        if (kdtree.nearestKSearch(point, 1, nn_indices, nn_dists) > 0) {
            if (nn_dists[0] <= distance_threshold * distance_threshold) {
                inlier_count++;
            }
        }
    }

    double ratio = static_cast<double>(inlier_count) / transformed_cloud->size();
    return ratio;
}


double InitPoseAlignment::calculateTranslationError(
    const Eigen::Matrix4f& transform1,
    const Eigen::Matrix4f& transform2) {

    Eigen::Vector3f translation1 = transform1.block<3, 1>(0, 3);
    Eigen::Vector3f translation2 = transform2.block<3, 1>(0, 3);

    Eigen::Vector3f translation_diff = translation1 - translation2;
    return translation_diff.norm();
}


double InitPoseAlignment::calculateOrientationError(
    const Eigen::Matrix4f& transform1,
    const Eigen::Matrix4f& transform2) {

    Eigen::Matrix3f rotation1 = transform1.block<3, 3>(0, 0);
    Eigen::Matrix3f rotation2 = transform2.block<3, 3>(0, 0);

    Eigen::Quaternionf q1(rotation1);
    Eigen::Quaternionf q2(rotation2);

    double dot_product = std::abs(q1.dot(q2));
    if (dot_product > 1.0) {
        dot_product = 1.0;
    }

    // Return orientation difference in radians
    return 2.0 * std::acos(dot_product);
}


AlignmentResult InitPoseAlignment::multipleAlignmentAttempts(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const Eigen::Matrix4f& initial_guess) {

    std::vector<AlignmentResult> alignment_results;

    Eigen::Vector3f initial_position(initial_guess(0, 3), initial_guess(1, 3), initial_guess(2, 3));
    Eigen::Matrix3f initial_rotation = initial_guess.block<3, 3>(0, 0);

    // Set range for random perturbations
    const float position_noise = 0.5f;  // Position perturbation range (meters)
    const float angle_noise = 0.1f;     // Angle perturbation range (radians)

    // Try multiple alignments with different initial guesses
    for (int i = 0; i < multi_align_attempts_; ++i) {
        Eigen::Matrix4f current_guess = initial_guess;

        if (i > 0) {
            Eigen::Vector3f position_perturbation(
                getRandomGaussian(0, position_noise),
                getRandomGaussian(0, position_noise),
                0
            );

            float angle_perturbation = getRandomGaussian(0, angle_noise);
            Eigen::AngleAxisf rotation_perturbation(angle_perturbation, Eigen::Vector3f::UnitZ());

            Eigen::Matrix3f perturbed_rotation = initial_rotation * rotation_perturbation.toRotationMatrix();
            Eigen::Vector3f perturbed_position = initial_position + position_perturbation;

            current_guess.block<3, 3>(0, 0) = perturbed_rotation;
            current_guess.block<3, 1>(0, 3) = perturbed_position;

            RCLCPP_DEBUG(this->get_logger(), "Attempt %d/%d: position perturbation=[%.2f, %.2f, %.2f], angle perturbation=%.2f deg",
                i+1, multi_align_attempts_,
                position_perturbation.x(), position_perturbation.y(), position_perturbation.z(),
                angle_perturbation * 180.0f / M_PI);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Attempt %d/%d: using original initial guess", i+1, multi_align_attempts_);
        }

        AlignmentResult result = performGICPAlignment(source_cloud, target_cloud_, current_guess,
                                                      gicp_max_iterations_, 1.0);

        alignment_results.push_back(result);

        RCLCPP_INFO(this->get_logger(), "Attempt %d/%d result: converged=%d, score=%.4f",
                    i+1, multi_align_attempts_, result.has_converged, result.fitness_score);
    }

    // Choose the best result (lowest score and has converged)
    auto best_result = std::min_element(alignment_results.begin(), alignment_results.end(),
        [this](const AlignmentResult& a, const AlignmentResult& b) {
            if (a.has_converged != b.has_converged) {
                return a.has_converged;
            }
            return a.fitness_score < b.fitness_score;
        });

    int best_index = std::distance(alignment_results.begin(), best_result);
    RCLCPP_INFO(this->get_logger(), "Best result from multiple attempts: score=%.4f", best_result->fitness_score);

    if (best_result->fitness_score < fitness_score_threshold_ && best_result->has_converged) {
        has_aligned_ = true;
        RCLCPP_INFO(this->get_logger(), "Good alignment found in multiple attempts.");
        return *best_result;
    }

    return *best_result;
}


void InitPoseAlignment::publishTransform(const Eigen::Matrix4f& transform) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = map_frame_id_;
    transform_stamped.child_frame_id = odom_frame_id_;
    
    transform_stamped.transform.translation.x = transform(0, 3);
    transform_stamped.transform.translation.y = transform(1, 3);
    transform_stamped.transform.translation.z = transform(2, 3);
    
    Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rotation);
    
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();
    
    tf_pub_->publish(transform_stamped);
}

float InitPoseAlignment::getRandomGaussian(float mean, float stddev) {
    std::normal_distribution<float> distribution(mean, stddev);
    return distribution(rand_gen_);
}

}  // namespace tars_alignment
