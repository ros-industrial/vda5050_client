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
#include <string>
#include <thread>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/msg/order_status.hpp"
#include "vda5050_master_ros2/order_status_publisher.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

constexpr const char* kMfg = "ACME";
constexpr const char* kSerial = "AGV01";

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

class OrderStatusPublisherTest : public ::testing::Test
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
    node_ = rclcpp::Node::make_shared("order_status_publisher_test_node");
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

TEST_F(OrderStatusPublisherTest, TopicNameFollowsConvention)
{
  OrderStatusPublisher pub(node_);
  EXPECT_EQ(
    pub.order_status_topic(kMfg, kSerial),
    "/vda5050_master/ACME/AGV01/order_status");
}

TEST_F(OrderStatusPublisherTest, CustomNamespaceAppliesToTopic)
{
  OrderStatusPublisher pub(node_, "my_master");
  EXPECT_EQ(
    pub.order_status_topic(kMfg, kSerial),
    "/my_master/ACME/AGV01/order_status");
}

TEST_F(OrderStatusPublisherTest, LazyCreatesPublisherAndIsolatesPerAgv)
{
  OrderStatusPublisher pub(node_);
  EXPECT_FALSE(pub.has_publisher_for(kMfg, kSerial));
  EXPECT_FALSE(pub.has_publisher_for("OTHER", "AGV02"));

  vda5050_master_ros2::msg::OrderStatus msg_a;
  msg_a.manufacturer = kMfg;
  msg_a.serial_number = kSerial;
  pub.publish_order_status(kMfg, kSerial, msg_a);
  EXPECT_TRUE(pub.has_publisher_for(kMfg, kSerial));
  EXPECT_FALSE(pub.has_publisher_for("OTHER", "AGV02"));

  vda5050_master_ros2::msg::OrderStatus msg_b;
  msg_b.manufacturer = "OTHER";
  msg_b.serial_number = "AGV02";
  pub.publish_order_status("OTHER", "AGV02", msg_b);
  EXPECT_TRUE(pub.has_publisher_for("OTHER", "AGV02"));

  pub.remove_agv(kMfg, kSerial);
  EXPECT_FALSE(pub.has_publisher_for(kMfg, kSerial));
  EXPECT_TRUE(pub.has_publisher_for("OTHER", "AGV02"));
}

TEST_F(OrderStatusPublisherTest, PublishOrderStatusReachesSubscriber)
{
  OrderStatusPublisher pub(node_);

  std::atomic<int> count{0};
  auto sub = node_->create_subscription<vda5050_master_ros2::msg::OrderStatus>(
    pub.order_status_topic(kMfg, kSerial),
    rclcpp::QoS(OrderStatusPublisher::kQosDepth),
    [&](const vda5050_master_ros2::msg::OrderStatus::SharedPtr) {
      count.fetch_add(1);
    });

  vda5050_master_ros2::msg::OrderStatus msg;
  msg.manufacturer = kMfg;
  msg.serial_number = kSerial;
  msg.phase = vda5050_master_ros2::msg::OrderStatus::PHASE_RUNNING;
  pub.publish_order_status(kMfg, kSerial, msg);

  EXPECT_TRUE(
    wait_for([&] { return count.load() >= 1; }, std::chrono::milliseconds(500)))
    << "order_status message never reached subscriber";
}

}  // namespace test
}  // namespace vda5050_master_ros2
