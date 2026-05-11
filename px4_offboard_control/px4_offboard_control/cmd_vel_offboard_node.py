#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class CmdVelOffboardNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_offboard_node')

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.counter = 0
        self.current_twist = Twist()
        self.altitude = -2.0

        self.get_logger().info('cmd_vel offboard node started')

    def cmd_vel_callback(self, msg):
        self.current_twist = msg

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_velocity_setpoint()

        if self.counter == 20:
            self.arm()
            self.set_offboard_mode()

        self.counter += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def publish_velocity_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.position = [float('nan'), float('nan'), self.altitude]
        msg.velocity = [
            float(self.current_twist.linear.x),
            float(self.current_twist.linear.y),
            float(self.current_twist.linear.z)
        ]
        msg.yaw = float('nan')
        msg.yawspeed = float(self.current_twist.angular.z)

        self.setpoint_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )
        self.get_logger().info('Arm command sent')

    def set_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0
        )
        self.get_logger().info('Offboard mode command sent')

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelOffboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
