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
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/assignment_result.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_interfaces/msg/order.hpp"
#include "vda5050_master_ros2/assign_order_request_subscriber.hpp"
#include "vda5050_master_ros2/assignment_result_publisher.hpp"
#include "vda5050_master_ros2/msg/assign_order_request.hpp"
#include "vda5050_master_ros2/msg/assignment_result.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

constexpr const char* kMfg = "ACME";
constexpr const char* kSerial = "AGV01";

using AssignOrderRequest = vda5050_master_ros2::msg::AssignOrderRequest;
using AssignmentResult = vda5050_master_ros2::msg::AssignmentResult;

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

vda5050_interfaces::msg::Order make_order_msg(
  const std::string& order_id, std::uint32_t order_update_id)
{
  vda5050_interfaces::msg::Order msg;
  msg.header.header_id = 1;
  msg.header.version = "2.0.0";
  msg.header.manufacturer = kMfg;
  msg.header.serial_number = kSerial;
  msg.order_id = order_id;
  msg.order_update_id = order_update_id;
  vda5050_interfaces::msg::Node n;
  n.node_id = "N0";
  n.sequence_id = 0;
  n.released = true;
  msg.nodes.push_back(n);
  return msg;
}

// Test plumbing: a sender + recorder pair backed by shared state so
// tests can both observe what the subscriber dispatched and dictate the
// AssignmentResult it gets back.
struct DispatchStub
{
  std::shared_ptr<std::atomic<int>> sender_calls{
    std::make_shared<std::atomic<int>>(0)};
  std::shared_ptr<std::atomic<int>> recorder_calls{
    std::make_shared<std::atomic<int>>(0)};
  std::shared_ptr<std::string> last_assignment_id{
    std::make_shared<std::string>()};
  std::shared_ptr<vda5050_core::master::AssignmentResult> canned{
    std::make_shared<vda5050_core::master::AssignmentResult>()};

  AssignOrderRequestSubscriber::AssignmentSender sender()
  {
    auto sc = sender_calls;
    auto cn = canned;
    return [sc, cn](
             const std::string&, const std::string&,
             const vda5050_core::types::Order&) {
      sc->fetch_add(1);
      return *cn;
    };
  }

  AssignOrderRequestSubscriber::AssignmentRecorder recorder()
  {
    auto rc = recorder_calls;
    auto la = last_assignment_id;
    return
      [rc, la](
        const std::string&, const std::string&,
        const std::string& assignment_id, const std::string&, std::uint32_t) {
        rc->fetch_add(1);
        *la = assignment_id;
      };
  }
};

class AssignOrderRequestSubscriberTest : public ::testing::Test
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
    node_ = rclcpp::Node::make_shared("assign_order_request_sub_test_node");
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

// Plumbing: result-topic subscriber for assertions.
struct ResultCollector
{
  std::mutex mu;
  std::vector<AssignmentResult> received;
  rclcpp::Subscription<AssignmentResult>::SharedPtr sub;

  void attach(rclcpp::Node::SharedPtr node, const std::string& topic)
  {
    sub = node->create_subscription<AssignmentResult>(
      topic, rclcpp::QoS(AssignmentResultPublisher::kQosDepth),
      [this](AssignmentResult::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mu);
        received.push_back(*msg);
      });
  }

  std::size_t count()
  {
    std::lock_guard<std::mutex> lock(mu);
    return received.size();
  }

  AssignmentResult get(std::size_t i)
  {
    std::lock_guard<std::mutex> lock(mu);
    return received.at(i);
  }
};

}  // namespace

TEST_F(AssignOrderRequestSubscriberTest, TopicNameFollowsConvention)
{
  auto result_pub = std::make_shared<AssignmentResultPublisher>(node_);
  AssignOrderRequestSubscriber sub(
    node_,
    [](auto, auto, auto) { return vda5050_core::master::AssignmentResult{}; },
    nullptr, result_pub);
  EXPECT_EQ(sub.topic_name(), "/vda5050_master/assign_order_request");
}

TEST_F(AssignOrderRequestSubscriberTest, EmptyAssignmentIdRejected)
{
  DispatchStub stub;
  auto result_pub = std::make_shared<AssignmentResultPublisher>(node_);
  AssignOrderRequestSubscriber sub_obj(
    node_, stub.sender(), stub.recorder(), result_pub);

  ResultCollector collector;
  collector.attach(node_, result_pub->topic_name());

  auto pub = node_->create_publisher<AssignOrderRequest>(
    sub_obj.topic_name(), rclcpp::QoS(AssignOrderRequestSubscriber::kQosDepth));
  // Allow time for pub/sub to discover each other on the same node.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  AssignOrderRequest req;
  req.assignment_id = "";  // empty -> reject
  req.manufacturer = kMfg;
  req.serial_number = kSerial;
  req.order = make_order_msg("ORD-A", 0);
  pub->publish(req);

  ASSERT_TRUE(wait_for(
    [&] { return collector.count() >= 1; }, std::chrono::milliseconds(500)));
  EXPECT_EQ(collector.count(), 1u);
  auto r = collector.get(0);
  EXPECT_EQ(r.decision, AssignmentResult::REJECTED_PREFLIGHT);
  EXPECT_EQ(stub.sender_calls->load(), 0);
  EXPECT_EQ(stub.recorder_calls->load(), 0);
  ASSERT_EQ(r.errors.size(), 1u);
  EXPECT_EQ(r.errors[0].error_type, "InvalidRequest");
}

TEST_F(AssignOrderRequestSubscriberTest, EmptyManufacturerRejected)
{
  DispatchStub stub;
  auto result_pub = std::make_shared<AssignmentResultPublisher>(node_);
  AssignOrderRequestSubscriber sub_obj(
    node_, stub.sender(), stub.recorder(), result_pub);

  ResultCollector collector;
  collector.attach(node_, result_pub->topic_name());

  auto pub = node_->create_publisher<AssignOrderRequest>(
    sub_obj.topic_name(), rclcpp::QoS(AssignOrderRequestSubscriber::kQosDepth));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  AssignOrderRequest req;
  req.assignment_id = "uuid-1";
  req.manufacturer = "";
  req.serial_number = kSerial;
  req.order = make_order_msg("ORD-A", 0);
  pub->publish(req);

  ASSERT_TRUE(wait_for(
    [&] { return collector.count() >= 1; }, std::chrono::milliseconds(500)));
  EXPECT_EQ(collector.count(), 1u);
  EXPECT_EQ(collector.get(0).decision, AssignmentResult::REJECTED_PREFLIGHT);
  EXPECT_EQ(stub.sender_calls->load(), 0);
}

TEST_F(AssignOrderRequestSubscriberTest, AcceptedDecisionRecordsAssignment)
{
  DispatchStub stub;
  stub.canned->decision = vda5050_core::master::AssignmentDecision::ASSIGNED;

  auto result_pub = std::make_shared<AssignmentResultPublisher>(node_);
  AssignOrderRequestSubscriber sub_obj(
    node_, stub.sender(), stub.recorder(), result_pub);

  ResultCollector collector;
  collector.attach(node_, result_pub->topic_name());

  auto pub = node_->create_publisher<AssignOrderRequest>(
    sub_obj.topic_name(), rclcpp::QoS(AssignOrderRequestSubscriber::kQosDepth));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  AssignOrderRequest req;
  req.assignment_id = "uuid-accepted";
  req.manufacturer = kMfg;
  req.serial_number = kSerial;
  req.order = make_order_msg("ORD-X", 3);
  pub->publish(req);

  ASSERT_TRUE(wait_for(
    [&] { return collector.count() >= 1; }, std::chrono::milliseconds(500)));
  EXPECT_EQ(collector.count(), 1u);
  auto r = collector.get(0);
  EXPECT_EQ(r.decision, AssignmentResult::ACCEPTED);
  EXPECT_EQ(r.assignment_id, "uuid-accepted");
  EXPECT_EQ(r.order_id, "ORD-X");
  EXPECT_EQ(r.order_update_id, 3u);
  EXPECT_EQ(stub.sender_calls->load(), 1);
  EXPECT_EQ(stub.recorder_calls->load(), 1);
  EXPECT_EQ(*stub.last_assignment_id, "uuid-accepted");
}

TEST_F(AssignOrderRequestSubscriberTest, QueuedDecisionAlsoRecordsAssignment)
{
  DispatchStub stub;
  stub.canned->decision =
    vda5050_core::master::AssignmentDecision::STITCH_QUEUED;

  auto result_pub = std::make_shared<AssignmentResultPublisher>(node_);
  AssignOrderRequestSubscriber sub_obj(
    node_, stub.sender(), stub.recorder(), result_pub);

  ResultCollector collector;
  collector.attach(node_, result_pub->topic_name());

  auto pub = node_->create_publisher<AssignOrderRequest>(
    sub_obj.topic_name(), rclcpp::QoS(AssignOrderRequestSubscriber::kQosDepth));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  AssignOrderRequest req;
  req.assignment_id = "uuid-queued";
  req.manufacturer = kMfg;
  req.serial_number = kSerial;
  req.order = make_order_msg("ORD-Y", 0);
  pub->publish(req);

  ASSERT_TRUE(wait_for(
    [&] { return collector.count() >= 1; }, std::chrono::milliseconds(500)));
  EXPECT_EQ(collector.count(), 1u);
  EXPECT_EQ(collector.get(0).decision, AssignmentResult::QUEUED);
  EXPECT_EQ(stub.recorder_calls->load(), 1);
}

TEST_F(AssignOrderRequestSubscriberTest, RejectedDecisionCarriesErrors)
{
  DispatchStub stub;
  stub.canned->decision =
    vda5050_core::master::AssignmentDecision::AGV_NOT_READY;
  vda5050_core::types::Error err;
  err.error_type = "OperationalState";
  err.error_level = vda5050_core::types::ErrorLevel::FATAL;
  err.error_description = "agv not ready";
  stub.canned->errors.push_back(err);

  auto result_pub = std::make_shared<AssignmentResultPublisher>(node_);
  AssignOrderRequestSubscriber sub_obj(
    node_, stub.sender(), stub.recorder(), result_pub);

  ResultCollector collector;
  collector.attach(node_, result_pub->topic_name());

  auto pub = node_->create_publisher<AssignOrderRequest>(
    sub_obj.topic_name(), rclcpp::QoS(AssignOrderRequestSubscriber::kQosDepth));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  AssignOrderRequest req;
  req.assignment_id = "uuid-reject";
  req.manufacturer = kMfg;
  req.serial_number = kSerial;
  req.order = make_order_msg("ORD-R", 0);
  pub->publish(req);

  ASSERT_TRUE(wait_for(
    [&] { return collector.count() >= 1; }, std::chrono::milliseconds(500)));
  EXPECT_EQ(collector.count(), 1u);
  auto r = collector.get(0);
  EXPECT_EQ(r.decision, AssignmentResult::REJECTED_PREFLIGHT);
  EXPECT_EQ(stub.recorder_calls->load(), 0);
  ASSERT_EQ(r.errors.size(), 1u);
  EXPECT_EQ(r.errors[0].error_type, "OperationalState");
}

TEST_F(AssignOrderRequestSubscriberTest, NullRecorderDoesNotCrashOnAccepted)
{
  DispatchStub stub;
  stub.canned->decision = vda5050_core::master::AssignmentDecision::ASSIGNED;

  auto result_pub = std::make_shared<AssignmentResultPublisher>(node_);
  AssignOrderRequestSubscriber sub_obj(
    node_, stub.sender(),
    AssignOrderRequestSubscriber::AssignmentRecorder{nullptr}, result_pub);

  ResultCollector collector;
  collector.attach(node_, result_pub->topic_name());

  auto pub = node_->create_publisher<AssignOrderRequest>(
    sub_obj.topic_name(), rclcpp::QoS(AssignOrderRequestSubscriber::kQosDepth));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  AssignOrderRequest req;
  req.assignment_id = "uuid-no-recorder";
  req.manufacturer = kMfg;
  req.serial_number = kSerial;
  req.order = make_order_msg("ORD-N", 0);
  pub->publish(req);

  ASSERT_TRUE(wait_for(
    [&] { return collector.count() >= 1; }, std::chrono::milliseconds(500)));
  EXPECT_EQ(collector.count(), 1u);
  EXPECT_EQ(collector.get(0).decision, AssignmentResult::ACCEPTED);
}

}  // namespace test
}  // namespace vda5050_master_ros2
