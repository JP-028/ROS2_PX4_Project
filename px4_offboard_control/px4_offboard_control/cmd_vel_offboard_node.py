#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleAttitude,
)


class CmdVelOffboardNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_offboard_node')

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
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

        # PX4 NED: negative z means height above ground
        self.altitude = -0.8

        # current yaw angle of UAV in radians
        self.current_yaw = 0.0
        self.has_attitude = False

        self.get_logger().info('cmd_vel offboard node started')
        self.get_logger().info('Mode: /cmd_vel linear.x/y interpreted as BODY frame')
        self.get_logger().info('linear.x = forward relative to UAV')
        self.get_logger().info('linear.y = left/right relative to UAV')
        self.get_logger().info('angular.z = yaw speed')

    def cmd_vel_callback(self, msg):
        self.current_twist = msg

    def attitude_callback(self, msg):
        q = msg.q

        # PX4 quaternion order: [w, x, y, z]
        w = q[0]
        x = q[1]
        y = q[2]
        z = q[3]

        # yaw from quaternion
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_attitude = True

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

        # Body-frame command from /cmd_vel
        vx_body = float(self.current_twist.linear.x)
        vy_body = float(self.current_twist.linear.y)
        vz_body = float(self.current_twist.linear.z)

        yaw = self.current_yaw

        # Convert body-frame velocity to local-frame velocity
        vx_local = math.cos(yaw) * vx_body - math.sin(yaw) * vy_body
        vy_local = math.sin(yaw) * vx_body + math.cos(yaw) * vy_body

        msg.position = [float('nan'), float('nan'), self.altitude]
        msg.velocity = [
            float(vx_local),
            float(vy_local),
            float('nan')
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

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('cmd_vel offboard node stopped manually')
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
