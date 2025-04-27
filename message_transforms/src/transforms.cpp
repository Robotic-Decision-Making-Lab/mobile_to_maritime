// Copyright 2024, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "message_transforms/transforms.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace m2m
{

auto transform_message(geometry_msgs::msg::Pose & m) -> void
{
  const KDL::Vector v(m.position.x, m.position.y, m.position.z);
  const KDL::Rotation r = KDL::Rotation::Quaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w);

  // The transformation is a rotation about the x-axis by 180 degrees
  const KDL::Frame transform(KDL::Rotation::Quaternion(1, 0, 0, 0), KDL::Vector(0, 0, 0));
  const KDL::Frame v_out = transform * KDL::Frame(r, v);

  m.position.x = v_out.p.x();
  m.position.y = v_out.p.y();
  m.position.z = v_out.p.z();
  v_out.M.GetQuaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w);
}

auto transform_message(geometry_msgs::msg::Twist & m) -> void
{
  m.linear.y *= -1;
  m.linear.z *= -1;
  m.angular.y *= -1;
  m.angular.z *= -1;
}

auto transform_message(geometry_msgs::msg::Wrench & m) -> void
{
  m.force.y *= -1;
  m.force.z *= -1;
  m.torque.y *= -1;
  m.torque.z *= -1;
}

auto transform_message(geometry_msgs::msg::PoseStamped & m, const std::string & frame_id) -> void
{
  m.header.frame_id = frame_id;
  transform_message(m.pose);
}

auto transform_message(geometry_msgs::msg::TwistStamped & m, const std::string & frame_id) -> void
{
  m.header.frame_id = frame_id;
  transform_message(m.twist);
}

auto transform_message(geometry_msgs::msg::WrenchStamped & m, const std::string & frame_id) -> void
{
  m.header.frame_id = frame_id;
  transform_message(m.wrench);
}

auto transform_message(nav_msgs::msg::Odometry & m, const std::string & frame_id, const std::string & child_frame_id)
  -> void
{
  m.header.frame_id = frame_id;
  m.child_frame_id = child_frame_id;
  transform_message(m.pose.pose);
  transform_message(m.twist.twist);
}

}  // namespace m2m
