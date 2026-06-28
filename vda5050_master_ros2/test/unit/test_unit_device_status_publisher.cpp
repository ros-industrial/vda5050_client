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
#include "vda5050_core/master/agv.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/factsheet.hpp"
#include "vda5050_core/types/state.hpp"
#include "vda5050_interfaces/msg/connection.hpp"
#include "vda5050_interfaces/msg/factsheet.hpp"
#include "vda5050_interfaces/msg/state.hpp"
#include "vda5050_master_ros2/device_status_publisher.hpp"
#include "vda5050_master_ros2/msg/device_status.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

constexpr const char* kMfg = "ACME";
constexpr const char* kSerial = "AGV01";

// Poll a predicate until true or timeout. Returns true if predicate passed.
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

class DeviceStatusPublisherTest : public ::testing::Test
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
    node_ = rclcpp::Node::make_shared("device_status_publisher_test_node");
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

// =============================================================================
// Topic naming + lazy creation
// =============================================================================

TEST_F(DeviceStatusPublisherTest, TopicNamesFollowDefaultConvention)
{
  DeviceStatusPublisher pub(node_);
  EXPECT_EQ(pub.state_topic(kMfg, kSerial), "/vda5050_master/ACME/AGV01/state");
  EXPECT_EQ(
    pub.connection_topic(kMfg, kSerial),
    "/vda5050_master/ACME/AGV01/connection");
  EXPECT_EQ(
    pub.factsheet_topic(kMfg, kSerial), "/vda5050_master/ACME/AGV01/factsheet");
}

TEST_F(DeviceStatusPublisherTest, CustomNamespaceAppliesToAllTopics)
{
  DeviceStatusPublisher pub(node_, "my_master");
  EXPECT_EQ(pub.state_topic(kMfg, kSerial), "/my_master/ACME/AGV01/state");
  EXPECT_EQ(
    pub.connection_topic(kMfg, kSerial), "/my_master/ACME/AGV01/connection");
  EXPECT_EQ(
    pub.factsheet_topic(kMfg, kSerial), "/my_master/ACME/AGV01/factsheet");
}

TEST_F(DeviceStatusPublisherTest, LazyCreatesPublishersOnFirstCall)
{
  DeviceStatusPublisher pub(node_);
  EXPECT_FALSE(pub.has_publishers_for(kMfg, kSerial));

  vda5050_core::types::State state;
  state.header.version = "2.0.0";
  state.header.manufacturer = kMfg;
  state.header.serial_number = kSerial;
  state.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;
  pub.publish_state(kMfg, kSerial, state);

  EXPECT_TRUE(pub.has_publishers_for(kMfg, kSerial));
}

// =============================================================================
// Round-trip publish — verify message reaches a subscriber
// =============================================================================

TEST_F(DeviceStatusPublisherTest, PublishStateReachesSubscriber)
{
  DeviceStatusPublisher pub(node_);

  std::atomic<int> count{0};
  auto sub = node_->create_subscription<vda5050_interfaces::msg::State>(
    pub.state_topic(kMfg, kSerial),
    rclcpp::QoS(DeviceStatusPublisher::kQosDepth),
    [&](const vda5050_interfaces::msg::State::SharedPtr) {
      count.fetch_add(1);
    });

  vda5050_core::types::State state;
  state.header.version = "2.0.0";
  state.header.manufacturer = kMfg;
  state.header.serial_number = kSerial;
  state.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;
  pub.publish_state(kMfg, kSerial, state);

  EXPECT_TRUE(
    wait_for([&] { return count.load() >= 1; }, std::chrono::milliseconds(500)))
    << "state message never reached subscriber";
}

TEST_F(DeviceStatusPublisherTest, PublishConnectionReachesSubscriber)
{
  DeviceStatusPublisher pub(node_);

  std::atomic<int> count{0};
  auto sub = node_->create_subscription<vda5050_interfaces::msg::Connection>(
    pub.connection_topic(kMfg, kSerial),
    rclcpp::QoS(DeviceStatusPublisher::kQosDepth),
    [&](const vda5050_interfaces::msg::Connection::SharedPtr) {
      count.fetch_add(1);
    });

  vda5050_core::types::Connection conn;
  conn.header.version = "2.0.0";
  conn.header.manufacturer = kMfg;
  conn.header.serial_number = kSerial;
  conn.connection_state = vda5050_core::types::ConnectionState::ONLINE;
  pub.publish_connection(kMfg, kSerial, conn);

  EXPECT_TRUE(
    wait_for([&] { return count.load() >= 1; }, std::chrono::milliseconds(500)))
    << "connection message never reached subscriber";
}

// =============================================================================
// Lifecycle
// =============================================================================

TEST_F(DeviceStatusPublisherTest, RemoveAgvDropsPublishers)
{
  DeviceStatusPublisher pub(node_);

  vda5050_core::types::State state;
  state.header.version = "2.0.0";
  state.header.manufacturer = kMfg;
  state.header.serial_number = kSerial;
  state.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;
  pub.publish_state(kMfg, kSerial, state);
  ASSERT_TRUE(pub.has_publishers_for(kMfg, kSerial));

  pub.remove_agv(kMfg, kSerial);
  EXPECT_FALSE(pub.has_publishers_for(kMfg, kSerial));
}

TEST_F(DeviceStatusPublisherTest, MultipleAGVs_HavePublishersIndependently)
{
  DeviceStatusPublisher pub(node_);

  vda5050_core::types::State state;
  state.header.version = "2.0.0";
  state.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;

  state.header.manufacturer = "ACME";
  state.header.serial_number = "AGV01";
  pub.publish_state("ACME", "AGV01", state);

  state.header.manufacturer = "OTHER";
  state.header.serial_number = "AGV02";
  pub.publish_state("OTHER", "AGV02", state);

  EXPECT_TRUE(pub.has_publishers_for("ACME", "AGV01"));
  EXPECT_TRUE(pub.has_publishers_for("OTHER", "AGV02"));

  pub.remove_agv("ACME", "AGV01");
  EXPECT_FALSE(pub.has_publishers_for("ACME", "AGV01"));
  EXPECT_TRUE(pub.has_publishers_for("OTHER", "AGV02"));
}

// =============================================================================
// Combined DeviceStatus topic
// =============================================================================
//
// The combined topic is RELIABLE + TRANSIENT_LOCAL (latched). Subscribers
// must match that QoS to receive messages.

namespace {

using vda5050_core::master::AGV;

rclcpp::QoS latched_qos()
{
  rclcpp::QoS qos(DeviceStatusPublisher::kQosDepthCombined);
  qos.reliable().transient_local();
  return qos;
}

vda5050_core::types::Connection make_connection()
{
  vda5050_core::types::Connection c;
  c.header.version = "2.0.0";
  c.header.manufacturer = kMfg;
  c.header.serial_number = kSerial;
  c.connection_state = vda5050_core::types::ConnectionState::ONLINE;
  return c;
}

vda5050_core::types::Factsheet make_factsheet()
{
  vda5050_core::types::Factsheet f;
  f.header.version = "2.0.0";
  f.header.manufacturer = kMfg;
  f.header.serial_number = kSerial;
  return f;
}

vda5050_core::types::State make_state()
{
  vda5050_core::types::State s;
  s.header.version = "2.0.0";
  s.header.manufacturer = kMfg;
  s.header.serial_number = kSerial;
  s.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;
  return s;
}

}  // namespace

TEST_F(DeviceStatusPublisherTest, CombinedTopicNameFollowsConvention)
{
  DeviceStatusPublisher pub(node_);
  EXPECT_EQ(
    pub.device_status_topic(kMfg, kSerial),
    "/vda5050_master/ACME/AGV01/device_status");
}

TEST_F(DeviceStatusPublisherTest, CombinedCustomNamespaceApplies)
{
  DeviceStatusPublisher pub(node_, "my_master");
  EXPECT_EQ(
    pub.device_status_topic(kMfg, kSerial),
    "/my_master/ACME/AGV01/device_status");
}

TEST_F(DeviceStatusPublisherTest, CombinedFullSnapshotPopulatesAllArrays)
{
  DeviceStatusPublisher pub(node_);

  std::atomic<int> count{0};
  vda5050_master_ros2::msg::DeviceStatus received;
  auto sub = node_->create_subscription<vda5050_master_ros2::msg::DeviceStatus>(
    pub.device_status_topic(kMfg, kSerial), latched_qos(),
    [&](const vda5050_master_ros2::msg::DeviceStatus::SharedPtr msg) {
      received = *msg;
      count.fetch_add(1);
    });

  AGV::StatusSnapshot snap;
  snap.state = make_state();
  snap.connection = make_connection();
  snap.factsheet = make_factsheet();
  snap.state_received_at = AGV::Clock::now();
  snap.connection_received_at = AGV::Clock::now();
  snap.factsheet_received_at = AGV::Clock::now();
  pub.publish_device_status(kMfg, kSerial, snap);

  ASSERT_TRUE(
    wait_for([&] { return count.load() >= 1; }, std::chrono::milliseconds(500)))
    << "combined snapshot never reached subscriber";

  EXPECT_EQ(received.manufacturer, kMfg);
  EXPECT_EQ(received.serial_number, kSerial);
  ASSERT_EQ(received.state.size(), 1u);
  ASSERT_EQ(received.connection.size(), 1u);
  ASSERT_EQ(received.factsheet.size(), 1u);
  EXPECT_EQ(received.state[0].header.manufacturer, kMfg);
  EXPECT_EQ(received.connection[0].header.serial_number, kSerial);
  EXPECT_EQ(received.factsheet[0].header.serial_number, kSerial);
  ASSERT_EQ(received.state_received_at.size(), 1u);
  ASSERT_EQ(received.connection_received_at.size(), 1u);
  ASSERT_EQ(received.factsheet_received_at.size(), 1u);
  EXPECT_GT(received.state_received_at[0].sec, 0);
}

TEST_F(DeviceStatusPublisherTest, CombinedEmptySnapshotProducesEmptyArrays)
{
  DeviceStatusPublisher pub(node_);

  std::atomic<int> count{0};
  vda5050_master_ros2::msg::DeviceStatus received;
  auto sub = node_->create_subscription<vda5050_master_ros2::msg::DeviceStatus>(
    pub.device_status_topic(kMfg, kSerial), latched_qos(),
    [&](const vda5050_master_ros2::msg::DeviceStatus::SharedPtr msg) {
      received = *msg;
      count.fetch_add(1);
    });

  AGV::StatusSnapshot snap;  // all optionals nullopt
  pub.publish_device_status(kMfg, kSerial, snap);

  ASSERT_TRUE(
    wait_for([&] { return count.load() >= 1; }, std::chrono::milliseconds(500)))
    << "combined snapshot never reached subscriber";

  EXPECT_EQ(received.manufacturer, kMfg);
  EXPECT_EQ(received.serial_number, kSerial);
  EXPECT_TRUE(received.state.empty());
  EXPECT_TRUE(received.connection.empty());
  EXPECT_TRUE(received.factsheet.empty());
  EXPECT_TRUE(received.state_received_at.empty());
  EXPECT_TRUE(received.connection_received_at.empty());
  EXPECT_TRUE(received.factsheet_received_at.empty());
}

TEST_F(DeviceStatusPublisherTest, CombinedPartialSnapshotOnlyPopulatesPresent)
{
  DeviceStatusPublisher pub(node_);

  std::atomic<int> count{0};
  vda5050_master_ros2::msg::DeviceStatus received;
  auto sub = node_->create_subscription<vda5050_master_ros2::msg::DeviceStatus>(
    pub.device_status_topic(kMfg, kSerial), latched_qos(),
    [&](const vda5050_master_ros2::msg::DeviceStatus::SharedPtr msg) {
      received = *msg;
      count.fetch_add(1);
    });

  AGV::StatusSnapshot snap;
  snap.state = make_state();
  snap.state_received_at = AGV::Clock::now();
  // connection + factsheet remain nullopt.
  pub.publish_device_status(kMfg, kSerial, snap);

  ASSERT_TRUE(
    wait_for([&] { return count.load() >= 1; }, std::chrono::milliseconds(500)))
    << "combined snapshot never reached subscriber";

  ASSERT_EQ(received.state.size(), 1u);
  EXPECT_TRUE(received.connection.empty());
  EXPECT_TRUE(received.factsheet.empty());
  ASSERT_EQ(received.state_received_at.size(), 1u);
  EXPECT_TRUE(received.connection_received_at.empty());
  EXPECT_TRUE(received.factsheet_received_at.empty());
}

TEST_F(DeviceStatusPublisherTest, CombinedLatchedDeliveryReachesLateSubscriber)
{
  DeviceStatusPublisher pub(node_);

  AGV::StatusSnapshot snap;
  snap.state = make_state();
  snap.state_received_at = AGV::Clock::now();
  pub.publish_device_status(kMfg, kSerial, snap);

  // Subscribe AFTER the publish. TRANSIENT_LOCAL must deliver the
  // latched message to this late joiner.
  std::atomic<int> count{0};
  auto sub = node_->create_subscription<vda5050_master_ros2::msg::DeviceStatus>(
    pub.device_status_topic(kMfg, kSerial), latched_qos(),
    [&](const vda5050_master_ros2::msg::DeviceStatus::SharedPtr) {
      count.fetch_add(1);
    });

  EXPECT_TRUE(wait_for(
    [&] { return count.load() >= 1; }, std::chrono::milliseconds(1000)))
    << "late subscriber did not receive latched message";
}

}  // namespace test
}  // namespace vda5050_master_ros2
