#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace tars_alignment {

class InitTFPublisher : public rclcpp::Node
{
public:
    InitTFPublisher(const rclcpp::NodeOptions& node_options) : Node("init_tf_publisher", node_options)
    {
        this->declare_parameter<std::string>("odom_frame_id", "odom_init");
        this->declare_parameter<std::string>("map_frame_id", "map");

        this->get_parameter_or<std::string>("odom_frame_id", odom_frame_id_, "odom_init");
        this->get_parameter_or<std::string>("map_frame_id", map_frame_id_, "map");

        subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
                        "/tf_map_to_odom", 10, 
                        std::bind(&InitTFPublisher::callback, this, std::placeholders::_1));

        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

private:
    void callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform = *msg;
        transform.header.frame_id = map_frame_id_;
        transform.child_frame_id = odom_frame_id_;
        broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
    std::string odom_frame_id_, map_frame_id_;
};

} // namespace tars_alignment

RCLCPP_COMPONENTS_REGISTER_NODE(tars_alignment::InitTFPublisher)
