# Copyright 2024, Evan Palmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from abc import ABC, abstractmethod
from typing import Any

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_default


class MobileToMaritime(Node, ABC):
    def __init__(self, message_type: Any, node_name: str) -> None:
        super().__init__(node_name)

        self.declare_parameters("", [("in_topic", ""), ("out_topic", "")])

        in_topic = self.get_parameter("in_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("out_topic").get_parameter_value().string_value

        self.in_sub = self.create_subscription(
            message_type, in_topic, self.in_callback, qos_profile_default
        )
        self.out_pub = self.create_publisher(
            message_type, out_topic, qos_profile_default
        )

    @abstractmethod
    def in_callback(self, msg: Any) -> None: ...


class MaritimeToMobileStamped(MobileToMaritime):
    def __init__(self, message_type: Any, node_name: str) -> None:
        super().__init__(message_type, node_name)

        self.declare_parameters("", [("in_frame", ""), ("out_frame", "")])

        self.in_frame = (
            self.get_parameter("in_frame").get_parameter_value().string_value
        )
        self.out_frame = (
            self.get_parameter("out_frame").get_parameter_value().string_value
        )


class MobileToMaritimeTwist(MobileToMaritime):
    def __init__(self) -> None:
        super().__init__(Twist, "mobile_to_maritime_twist")

    def in_callback(self, msg: Twist) -> None:
        maritime_twist = Twist()

        maritime_twist.linear.x = msg.linear.x
        maritime_twist.linear.y = -msg.linear.y
        maritime_twist.linear.z = -msg.linear.z

        maritime_twist.angular.x = msg.angular.x
        maritime_twist.angular.y = -msg.angular.y
        maritime_twist.angular.z = -msg.angular.z

        self.out_pub.publish(maritime_twist)


class MobileToMaritimeTwistStamped(MaritimeToMobileStamped):
    def __init__(self) -> None:
        super().__init__(TwistStamped, "mobile_to_maritime_twist_stamped")

    def in_callback(self, msg: TwistStamped) -> None:
        maritime_twist = TwistStamped()

        maritime_twist.header = msg.header
        maritime_twist.header.frame_id = self.out_frame

        maritime_twist.twist.linear.x = msg.twist.linear.x
        maritime_twist.twist.linear.y = -msg.twist.linear.y
        maritime_twist.twist.linear.z = -msg.twist.linear.z

        maritime_twist.twist.angular.x = msg.twist.angular.x
        maritime_twist.twist.angular.y = -msg.twist.angular.y
        maritime_twist.twist.angular.z = -msg.twist.angular.z

        self.out_pub.publish(maritime_twist)


def main_mobile_to_maritime_twist(args: list[str] | None = None):
    rclpy.init(args=args)

    node = MobileToMaritimeTwist()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


def main_mobile_to_maritime_twist_stamped(args: list[str] | None = None):
    rclpy.init(args=args)

    node = MobileToMaritimeTwistStamped()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()