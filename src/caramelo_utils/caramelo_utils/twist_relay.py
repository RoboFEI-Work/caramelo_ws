#!/usr/bin/env python3
"""Relay Twist -> TwistStamped: assina um topico Twist (saida do twist_mux ou
/cmd_vel do Nav2) e republica como TwistStamped na referencia do
mecanum_controller da Pi, carimbando hora e frame_id.
Quem usa: teleop.launch.py (no' twist_relay) e caramelo_navigation/
navigation.launch.py (no' nav2_twist_relay).
Como roda: ros2 run caramelo_utils twist_relay --ros-args -p input_twist_topic:=/cmd_vel
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay')

        # Parametros
        self.declare_parameter('input_twist_topic', 'robot_controller/cmd_vel_unstamped')
        self.declare_parameter('output_twist_stamped_topic', '/mecanum_controller/reference')
        self.declare_parameter('frame_id', 'base_footprint')

        input_topic = self.get_parameter('input_twist_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_twist_stamped_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Assinatura do Twist sem carimbo (twist_mux ou Nav2)
        self.twist_subscriber = self.create_subscription(
            Twist,
            input_topic,
            self.twist_callback,
            10,
        )

        # Publicacao do TwistStamped no topico do controlador mecanum
        self.twist_stamped_publisher = self.create_publisher(
            TwistStamped,
            output_topic,
            10,
        )

    def twist_callback(self, msg: Twist):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = self.frame_id
        twist_stamped.twist = msg
        self.twist_stamped_publisher.publish(twist_stamped)


def main():
    rclpy.init()
    node = TwistRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
