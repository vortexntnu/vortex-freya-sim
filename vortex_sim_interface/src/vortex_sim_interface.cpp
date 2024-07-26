#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <cmath>


class VortexSimInterface : public rclcpp::Node
{
public:
    VortexSimInterface() : Node("vortex_sim_interface")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/thrust/thruster_forces", 10,
            std::bind(&VortexSimInterface::thruster_callback, this, std::placeholders::_1));

        pub_right_front_ = this->create_publisher<std_msgs::msg::Float64>(
            "/wamv/thrusters/right_front/thrust", 10);
        pub_right_rear_ = this->create_publisher<std_msgs::msg::Float64>(
            "/wamv/thrusters/right_rear/thrust", 10);
        pub_left_rear_ = this->create_publisher<std_msgs::msg::Float64>(
            "/wamv/thrusters/left_rear/thrust", 10);
        pub_left_front_ = this->create_publisher<std_msgs::msg::Float64>(
            "/wamv/thrusters/left_front/thrust", 10);

        publishers_ = {pub_right_front_, pub_right_rear_, pub_left_rear_, pub_left_front_};

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/wamv/sensors/position/ground_truth_odometry", qos_sensor_data,
            std::bind(&VortexSimInterface::odometry_callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/seapath/odom/ned", qos_sensor_data);

        RCLCPP_INFO(this->get_logger(), "VortexSimInterface node initialized");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_front_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_rear_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_rear_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_front_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    void thruster_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 4) {
            RCLCPP_ERROR(this->get_logger(), "Received Float32MultiArray with incorrect size. Expected 4, got %zu", msg->data.size());
            return;
        }

        for (size_t i = 0; i < 4; ++i) {
            auto thrust_msg = std::make_unique<std_msgs::msg::Float64>();
            thrust_msg->data = static_cast<double>(msg->data[i]);
            publishers_[i]->publish(std::move(thrust_msg));
        }

        RCLCPP_DEBUG(this->get_logger(), "Published thrust values: %.2f, %.2f, %.2f, %.2f",
            msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto ned_msg = std::make_unique<nav_msgs::msg::Odometry>();

        ned_msg->header = msg->header;
        ned_msg->child_frame_id = "wamv/wamv/base_link";

        ned_msg->pose.pose.position.x = msg->pose.pose.position.y;
        ned_msg->pose.pose.position.y = msg->pose.pose.position.x;
        ned_msg->pose.pose.position.z = -msg->pose.pose.position.z;

        ned_msg->pose.pose.orientation = enu_to_ned_quaternion(msg->pose.pose.orientation);

        ned_msg->twist.twist.linear.x = msg->twist.twist.linear.y;
        ned_msg->twist.twist.linear.y = msg->twist.twist.linear.x;
        ned_msg->twist.twist.linear.z = -msg->twist.twist.linear.z;

        ned_msg->twist.twist.angular.x = msg->twist.twist.angular.y;
        ned_msg->twist.twist.angular.y = msg->twist.twist.angular.x;
        ned_msg->twist.twist.angular.z = -msg->twist.twist.angular.z;

        odom_pub_->publish(std::move(ned_msg));
    }

    geometry_msgs::msg::Quaternion enu_to_ned_quaternion(const geometry_msgs::msg::Quaternion& enu_quat)
    {
        // OBS: This is really hacky, but gets you "wamv/wamv/base_link" in NED when observed from the map frame
        tf2::Quaternion tf_enu_quat(enu_quat.x, enu_quat.y, enu_quat.z, enu_quat.w);
        tf2::Quaternion q_rot_z;
        q_rot_z.setRPY(0, 0, 3*M_PI/2);
        tf2::Quaternion tf_ned_quat = q_rot_z * tf_enu_quat;
        geometry_msgs::msg::Quaternion ned_quat;
        ned_quat.x = tf_ned_quat.x();
        ned_quat.y = tf_ned_quat.y();
        ned_quat.z = -tf_ned_quat.z();
        ned_quat.w = tf_ned_quat.w();
        return ned_quat;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VortexSimInterface>());
    rclcpp::shutdown();
    return 0;
}