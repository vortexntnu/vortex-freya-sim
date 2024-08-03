#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
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

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/seapath/odom/ned", qos_sensor_data);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ouster/points", qos_sensor_data);

        map_origin_lat_ = -33.722526870713736;
        map_origin_lon_ = 150.67413849891247;

        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/wamv/sensors/gps/gps/fix", qos_sensor_data,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                auto [x,y,z]= lla2flat(msg->latitude,msg->longitude,msg->altitude);
                map_origin_x = x;
                map_origin_y = y;
                geometry_msgs::msg::TransformStamped static_transform;
            static_transform.header.stamp = msg->header.stamp;
            static_transform.header.frame_id = "map";
            static_transform.child_frame_id = "odom";
            static_transform.transform.translation.x = y;
            static_transform.transform.translation.y = x;
            static_transform.transform.translation.z = 0.0;
            static_transform.transform.rotation.x = 0.0;
            static_transform.transform.rotation.y = 0.0;
            static_transform.transform.rotation.z = 0.0;
            static_transform.transform.rotation.w = 1.0;
            static_tf_broadcaster_->sendTransform(static_transform);

            geometry_msgs::msg::TransformStamped map_viz_tf;
            map_viz_tf.header.stamp = msg->header.stamp;
            map_viz_tf.header.frame_id = "map";
            map_viz_tf.child_frame_id = "map_viz";
            map_viz_tf.transform.translation.x = 0.0;
            map_viz_tf.transform.translation.y = 0.0;
            map_viz_tf.transform.translation.z = 0.0;
            map_viz_tf.transform.rotation.x = 0.0;
            map_viz_tf.transform.rotation.y = 1.0;
            map_viz_tf.transform.rotation.z = 0.0;
            map_viz_tf.transform.rotation.w = 0.0;
            static_tf_broadcaster_->sendTransform(map_viz_tf);
            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/wamv/sensors/position/ground_truth_odometry", qos_sensor_data,
            std::bind(&VortexSimInterface::odometry_callback, this, std::placeholders::_1));
            gps_sub_.reset();
            }

        );

        pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/wamv/sensors/lidars/lidar_wamv_sensor/points", qos_sensor_data,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                msg->header.frame_id = "os_sensor";
                pcl_pub_->publish(*msg);
            });

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
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

    bool map_tf_set_ = false;
    bool first_odom_received_ = false;
    geometry_msgs::msg::Point initial_position_;
    double map_origin_lat_;
    double map_origin_lon_;
    double map_origin_x;
    double map_origin_y;
    

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
        if (!first_odom_received_) {
            initial_position_ = msg->pose.pose.position;
            first_odom_received_ = true;
        }
        auto ned_msg = std::make_unique<nav_msgs::msg::Odometry>();

        ned_msg->header = msg->header;
        ned_msg->child_frame_id = "wamv/wamv/base_link";

        ned_msg->pose.pose.position.x = msg->pose.pose.position.y - initial_position_.y;
        ned_msg->pose.pose.position.y = msg->pose.pose.position.x - initial_position_.x;
        ned_msg->pose.pose.position.z = -msg->pose.pose.position.z - initial_position_.z;

        ned_msg->pose.pose.orientation = enu_to_ned_quaternion(msg->pose.pose.orientation);

        ned_msg->twist.twist.linear.x = msg->twist.twist.linear.x;
        ned_msg->twist.twist.linear.y = msg->twist.twist.linear.y;
        ned_msg->twist.twist.linear.z = -msg->twist.twist.linear.z;

        ned_msg->twist.twist.angular.x = msg->twist.twist.angular.y;
        ned_msg->twist.twist.angular.y = msg->twist.twist.angular.x;
        ned_msg->twist.twist.angular.z = -msg->twist.twist.angular.z;

         geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "seapath";

        transform.transform.translation.x = ned_msg->pose.pose.position.x;
        transform.transform.translation.y = ned_msg->pose.pose.position.y;
        transform.transform.translation.z = ned_msg->pose.pose.position.z;

        transform.transform.rotation = ned_msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform);

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

    std::array<double, 3> lla2flat(double lat, double lon, double alt) const
    {

        const double R = 6378137.0; // WGS-84 Earth semimajor axis (meters)
        const double f = 1.0 / 298.257223563; // Flattening of the earth
        const double psi_rad = 0.0; // Angular direction of the flat Earth x-axis, specified as a scalar. 
            // The angular direction is the degrees clockwise from north, 
            // which is the angle in degrees used for converting flat Earth x and y coordinates to the north and east coordinates

        // Convert angles from degrees to radians
        const double lat_rad = deg2rad(lat);
        const double lon_rad = deg2rad(lon);
        const double origin_lat_rad = deg2rad(map_origin_lat_);
        const double origin_lon_rad = deg2rad(map_origin_lon_);

        // Calculate delta latitude and delta longitude in radians
        const double dlat = lat_rad - origin_lat_rad;
        const double dlon = lon_rad - origin_lon_rad;

        // Radius of curvature in the vertical prime (RN)
        const double RN = R / sqrt(1.0 - (2.0 * f - f * f) * pow(sin(origin_lat_rad), 2));
        
        // Radius of curvature in the meridian (RM)
        const double RM = RN * (1.0 - (2.0 * f - f * f)) / (1.0 - (2.0 * f - f * f) * pow(sin(origin_lat_rad), 2));

        // Changes in the north (dN) and east (dE) positions
        const double dN = RM * dlat;
        const double dE = RN * cos(origin_lat_rad) * dlon;

        // Transformation from North-East to flat x-y coordinates
        const double px = cos(psi_rad) * dN - sin(psi_rad) * dE;
        const double py = sin(psi_rad) * dN + cos(psi_rad) * dE;
        const double pz = 0; // Flat Earth z-axis value (downwards positive, NED)

        return {px, py, pz};
    }

     constexpr double deg2rad(double degrees) const
    {
        return degrees * (M_PI / 180.0);
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VortexSimInterface>());
    rclcpp::shutdown();
    return 0;
}