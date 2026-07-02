/*
 * Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
 * Advanced Remanufacturing and Technology Centre
 * A*STAR Research Entities (Co. Registration No. 199702110H)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vda5050_interfaces/msg/error.hpp"
#include "vda5050_master_ros2/assignment_result_publisher.hpp"
#include "vda5050_master_ros2/msg/assignment_result.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

template <typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return pred();
}

class AssignmentResultPublisherTest : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("assignment_result_pub_test_node");
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    running_ = true;
    spin_thread_ = std::thread([this] {
      while (running_ && rclcpp::ok())
      {
        executor_->spin_some(std::chrono::milliseconds(10));
      }
    });
  }

  void TearDown() override
  {
    running_ = false;
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.reset();
    node_.reset();
  }
};

}  // namespace

TEST_F(AssignmentResultPublisherTest, TopicNameFollowsConvention)
{
  AssignmentResultPublisher pub(node_);
  EXPECT_EQ(pub.topic_name(), "/vda5050_master/assignment_results");
}

TEST_F(AssignmentResultPublisherTest, CustomNamespaceApplies)
{
  AssignmentResultPublisher pub(node_, "my_master");
  EXPECT_EQ(pub.topic_name(), "/my_master/assignment_results");
}

TEST_F(AssignmentResultPublisherTest, AcceptedResultEchoesAllFields)
{
  AssignmentResultPublisher pub(node_);

  std::atomic<int> count{0};
  vda5050_master_ros2::msg::AssignmentResult received;
  auto sub =
    node_->create_subscription<vda5050_master_ros2::msg::AssignmentResult>(
      pub.topic_name(), rclcpp::QoS(AssignmentResultPublisher::kQosDepth),
      [&](const vda5050_master_ros2::msg::AssignmentResult::SharedPtr msg) {
        received = *msg;
        count.fetch_add(1);
      });

  pub.publish_result(
    "uuid-1234", "ORD-A", 7,
    vda5050_master_ros2::msg::AssignmentResult::ACCEPTED, {});

  ASSERT_TRUE(
    wait_for([&] { return count.load() >= 1; }, std::chrono::milliseconds(500)))
    << "AssignmentResult never reached subscriber";
  EXPECT_EQ(received.assignment_id, "uuid-1234");
  EXPECT_EQ(received.order_id, "ORD-A");
  EXPECT_EQ(received.order_update_id, 7u);
  EXPECT_EQ(
    received.decision, vda5050_master_ros2::msg::AssignmentResult::ACCEPTED);
  EXPECT_TRUE(received.errors.empty());
  EXPECT_GT(received.stamp.sec, 0);
}

TEST_F(AssignmentResultPublisherTest, RejectedResultCarriesErrors)
{
  AssignmentResultPublisher pub(node_);

  std::atomic<int> count{0};
  vda5050_master_ros2::msg::AssignmentResult received;
  auto sub =
    node_->create_subscription<vda5050_master_ros2::msg::AssignmentResult>(
      pub.topic_name(), rclcpp::QoS(AssignmentResultPublisher::kQosDepth),
      [&](const vda5050_master_ros2::msg::AssignmentResult::SharedPtr msg) {
        received = *msg;
        count.fetch_add(1);
      });

  vda5050_interfaces::msg::Error err;
  err.error_type = "AgvNotReady";
  err.error_level = vda5050_interfaces::msg::Error::ERROR_LEVEL_FATAL;
  err.error_description.push_back("AGV is in ERROR state");

  pub.publish_result(
    "uuid-9", "ORD-B", 0,
    vda5050_master_ros2::msg::AssignmentResult::REJECTED_PREFLIGHT, {err});

  ASSERT_TRUE(
    wait_for([&] { return count.load() >= 1; }, std::chrono::milliseconds(500)))
    << "AssignmentResult never reached subscriber";
  EXPECT_EQ(
    received.decision,
    vda5050_master_ros2::msg::AssignmentResult::REJECTED_PREFLIGHT);
  ASSERT_EQ(received.errors.size(), 1u);
  EXPECT_EQ(received.errors[0].error_type, "AgvNotReady");
}

TEST_F(AssignmentResultPublisherTest, MultiplePublishesPreserveOrder)
{
  AssignmentResultPublisher pub(node_);

  std::atomic<int> count{0};
  std::vector<std::uint8_t> decisions;
  std::mutex decisions_mu;
  auto sub =
    node_->create_subscription<vda5050_master_ros2::msg::AssignmentResult>(
      pub.topic_name(), rclcpp::QoS(AssignmentResultPublisher::kQosDepth),
      [&](const vda5050_master_ros2::msg::AssignmentResult::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(decisions_mu);
        decisions.push_back(msg->decision);
        count.fetch_add(1);
      });

  pub.publish_result(
    "uuid-1", "ORD-X", 0, vda5050_master_ros2::msg::AssignmentResult::ACCEPTED,
    {});
  pub.publish_result(
    "uuid-1", "ORD-X", 0,
    vda5050_master_ros2::msg::AssignmentResult::REJECTED_POSTFLIGHT, {});

  ASSERT_TRUE(
    wait_for([&] { return count.load() >= 2; }, std::chrono::milliseconds(500)))
    << "expected two messages";
  std::lock_guard<std::mutex> lock(decisions_mu);
  ASSERT_EQ(decisions.size(), 2u);
  EXPECT_EQ(decisions[0], vda5050_master_ros2::msg::AssignmentResult::ACCEPTED);
  EXPECT_EQ(
    decisions[1],
    vda5050_master_ros2::msg::AssignmentResult::REJECTED_POSTFLIGHT);
}

}  // namespace test
}  // namespace vda5050_master_ros2
