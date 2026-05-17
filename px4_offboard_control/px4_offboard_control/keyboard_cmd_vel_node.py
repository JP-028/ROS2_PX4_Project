#!/usr/bin/env python3

import sys
import termios
import tty
import select
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


LINEAR_SPEED = 2.0
VERTICAL_SPEED = 0.5
YAW_SPEED = 1.0
DEADMAN_TIMEOUT = 0.5


class KeyboardCmdVelNode(Node):
    def __init__(self):
        super().__init__("keyboard_cmd_vel_node")

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.last_key_time = time.time()
        self.current_twist = Twist()
        self.stopped_by_deadman = False

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.print_instructions()

    def print_instructions(self):
        print("")
        print("======================================")
        print(" PX4 OFFBOARD KEYBOARD CONTROL")
        print("======================================")
        print("")
        print("Bewegung:")
        print("  w / Pfeil hoch      -> vorwärts")
        print("  s / Pfeil runter    -> rückwärts")
        print("  a / Pfeil links     -> links")
        print("  d / Pfeil rechts    -> rechts")
        print("")
        print("Drehen:")
        print("  j                   -> yaw links")
        print("  l                   -> yaw rechts")
        print("")
        print("Höhe:")
        print("  i                   -> hoch")
        print("  k                   -> runter / stop bei Bedarf")
        print("")
        print("Sicherheit:")
        print("  Leertaste           -> sofort Stop")
        print("  keine Taste 0.5 s   -> Auto-Stop")
        print("")
        print("Beenden:")
        print("  q")
        print("")
        print("======================================")
        print("")

    def stop(self):
        self.current_twist = Twist()
        self.publisher.publish(self.current_twist)

    def set_twist(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.z = angular_z

        self.current_twist = msg
        self.publisher.publish(msg)

        self.last_key_time = time.time()
        self.stopped_by_deadman = False

    def handle_key(self, key):
        if key in ["w", "\x1b[A"]:
            self.set_twist(linear_x=LINEAR_SPEED)
            print("vorwärts")

        elif key in ["s", "\x1b[B"]:
            self.set_twist(linear_x=-LINEAR_SPEED)
            print("rückwärts")

        elif key in ["a", "\x1b[D"]:
            self.set_twist(linear_y=LINEAR_SPEED)
            print("links")

        elif key in ["d", "\x1b[C"]:
            self.set_twist(linear_y=-LINEAR_SPEED)
            print("rechts")

        elif key == "i":
            self.set_twist(linear_z=VERTICAL_SPEED)
            print("hoch")

        elif key == "k":
            self.set_twist(linear_z=-VERTICAL_SPEED)
            print("runter")

        elif key == "j":
            self.set_twist(angular_z=YAW_SPEED)
            print("drehen links")

        elif key == "l":
            self.set_twist(angular_z=-YAW_SPEED)
            print("drehen rechts")

        elif key == " ":
            self.stop()
            self.last_key_time = time.time()
            print("STOP")

        elif key == "q":
            print("Beende Keyboard Control.")
            self.stop()
            raise KeyboardInterrupt

        else:
            print(f"Unbekannte Taste: {repr(key)}")

    def timer_callback(self):
        now = time.time()

        if now - self.last_key_time > DEADMAN_TIMEOUT:
            if not self.stopped_by_deadman:
                self.stop()
                self.stopped_by_deadman = True
                print("AUTO-STOP: keine Taste erkannt")


def read_key():
    if select.select([sys.stdin], [], [], 0.05)[0]:
        first = sys.stdin.read(1)

        if first == "\x1b":
            second = sys.stdin.read(1)
            third = sys.stdin.read(1)
            return first + second + third

        return first

    return None


def main(args=None):
    rclpy.init(args=args)

    node = KeyboardCmdVelNode()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)

        while rclpy.ok():
            key = read_key()

            if key is not None:
                node.handle_key(key)

            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
