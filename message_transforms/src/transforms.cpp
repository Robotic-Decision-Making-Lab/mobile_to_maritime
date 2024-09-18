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

namespace m2m::transforms
{

MessageTransforms::MessageTransforms(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("message_transforms", options)
{
}

auto MessageTransforms::on_configure(const rclcpp_lifecycle::State & /*state*/) -> CallbackReturn
{
  try {
    param_listener_ = std::make_shared<message_transforms::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to get MessageTransforms parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }

  for (const auto & topic : params_.incoming_topics) {
    std::vector<rclcpp::TopicEndpointInfo> info = get_publishers_info_by_topic(topic);

    // TODO(evan-palmer): figure out how to reason about multiple publishers
    for (const auto & endpoint : info) {
      if (endpoint.endpoint_type() == rclcpp::EndpointType::Publisher) {
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(endpoint.qos_profile()));

        // TODO(evan-palmer): figure out how to get the message type from the endpoint
        publishers_.push_back(
          create_publisher<geometry_msgs::msg::Pose>(params_.outgoing_messages[topic].outgoing_topic, qos));

        auto sub = create_subscription<geometry_msgs::msg::Pose>(
          topic, qos, [this, topic](const std::shared_ptr<geometry_msgs::msg::Pose> message) {
            transform_message(message);
            publishers_.back()->publish(*message);
          };);

        subscriptions_.push_back(sub);
      }
    }
  }

  return CallbackReturn::SUCCESS;
}

auto MessageTransforms::on_activate(const rclcpp_lifecycle::State & /*state*/) -> CallbackReturn
{
  return CallbackReturn::SUCCESS;
}

auto MessageTransforms::transform_message(std::shared_ptr<geometry_msgs::msg::Pose> & message) -> void {}

auto MessageTransforms::transform_message(std::shared_ptr<geometry_msgs::msg::Twist> & message) -> void {}

auto MessageTransforms::transform_message(std::shared_ptr<geometry_msgs::msg::Wrench> & message) -> void {}

auto MessageTransforms::transform_message(std::shared_ptr<geometry_msgs::msg::PoseStamped> & message) -> void {}

auto MessageTransforms::transform_message(std::shared_ptr<geometry_msgs::msg::TwistStamped> & message) -> void {}

auto MessageTransforms::transform_message(std::shared_ptr<geometry_msgs::msg::WrenchStamped> & message) -> void {}

auto MessageTransforms::transform_message(std::shared_ptr<nav_msgs::msg::Odometry> & message) -> void {}

}  // namespace m2m::transforms

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto node = std::make_shared<m2m::transforms::MessageTransforms>();
  executor.add_node(node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
