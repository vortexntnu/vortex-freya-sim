#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>

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

        // Initialize the publishers vector
        publishers_ = {pub_right_front_, pub_right_rear_, pub_left_rear_, pub_left_front_};

        RCLCPP_INFO(this->get_logger(), "VortexSimInterface node initialized");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_front_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_rear_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_rear_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_front_;
    
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers_;

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
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VortexSimInterface>());
    rclcpp::shutdown();
    return 0;
}