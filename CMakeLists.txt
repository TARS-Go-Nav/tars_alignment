cmake_minimum_required(VERSION 3.8)
project(alignment)
set(CMAKE_BUILD_TYPE "RELEASE") # GICP 配准需要 Release 编译

find_package (ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

# *************** Init Pose Alignment ***************
ament_auto_add_library(init_pose_alignment SHARED
    init_pose_alignment/init_pose_alignment.cpp
)

target_link_libraries(init_pose_alignment 
    ${PCL_LIBRARIES} Eigen3::Eigen
)

rclcpp_components_register_node(init_pose_alignment
    PLUGIN "tars_alignment::InitPoseAlignment"
    EXECUTABLE init_pose_alignment_node
)

# *************** Init TF Publisher ***************
ament_auto_add_library(init_tf_publisher SHARED
    init_tf_publisher/init_tf_publisher.cpp
)

rclcpp_components_register_node(init_tf_publisher
    PLUGIN "tars_alignment::InitTFPublisher"
    EXECUTABLE init_tf_publisher_node
)

ament_auto_package(
    INSTALL_TO_SHARE
    config launch
)
