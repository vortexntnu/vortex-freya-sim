#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class OtterSimInterface(Node):
    def __init__(self):
        super().__init__('otter_sim_interface')

        self.get_logger().info('Otter Sim Interface Node Started')
        
        pub_right_front = self.create_publisher(Float64, '/otter/thrusters/right_front/thrust', 10)
        pub_right_rear = self.create_publisher(Float64, '/otter/thrusters/right_rear/thrust', 10)
        pub_left_front = self.create_publisher(Float64, '/otter/thrusters/left_front/thrust', 10)
        pub_left_rear = self.create_publisher(Float64, '/otter/thrusters/left_rear/thrust', 10)

        self.publishers_ = [pub_right_front, pub_right_rear, pub_left_front, pub_left_rear]

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.publish_thrust(80.0)

    def publish_thrust(self, thrust_force: float):
        msg = Float64()
        for publisher in self.publishers_:
            msg.data = thrust_force
            publisher.publish(msg)


if __name__ == '__main__':
    rclpy.init()
    otter_sim_interface = OtterSimInterface()
    rclpy.spin(otter_sim_interface)
    otter_sim_interface.destroy_node()
    rclpy.shutdown()