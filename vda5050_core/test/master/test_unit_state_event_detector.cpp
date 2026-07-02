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

#include "vda5050_core/master/state/state_event_detector.hpp"

namespace vda5050_core::master::event::test {

namespace {
// Helpers for building synthetic State objects without ceremony.
vda5050_core::types::NodeState make_node(const std::string& id, bool released)
{
  vda5050_core::types::NodeState n;
  n.node_id = id;
  n.released = released;
  return n;
}

vda5050_core::types::Error make_error(
  const std::string& type, const std::string& description = "")
{
  vda5050_core::types::Error e;
  e.error_type = type;
  if (!description.empty()) e.error_description = description;
  return e;
}

vda5050_core::types::Load make_load(const std::string& id)
{
  vda5050_core::types::Load l;
  l.load_id = id;
  return l;
}
}  // namespace

// ============================================================================
// newly_reached_node
// ============================================================================

TEST(StateEventDetectorTest, NewlyReachedNodeOnLastNodeAdvance)
{
  vda5050_core::types::State prev;
  prev.last_node_id = "n0";
  prev.last_node_sequence_id = 0;
  vda5050_core::types::State curr;
  curr.last_node_id = "n1";
  curr.last_node_sequence_id = 2;

  auto r = newly_reached_node(prev, curr);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->node_id, "n1");
  EXPECT_EQ(r->sequence_id, 2u);
}

TEST(StateEventDetectorTest, NewlyReachedNodeOnSameIdNewSequence)
{
  vda5050_core::types::State prev;
  prev.last_node_id = "n1";
  prev.last_node_sequence_id = 2;
  vda5050_core::types::State curr;
  curr.last_node_id = "n1";  // same id revisited at a new sequence
  curr.last_node_sequence_id = 4;

  auto r = newly_reached_node(prev, curr);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->node_id, "n1");
  EXPECT_EQ(r->sequence_id, 4u);
}

TEST(StateEventDetectorTest, NewlyReachedNodeNulloptWhenUnchanged)
{
  vda5050_core::types::State prev;
  prev.last_node_id = "n1";
  prev.last_node_sequence_id = 2;
  vda5050_core::types::State curr;
  curr.last_node_id = "n1";
  curr.last_node_sequence_id = 2;

  EXPECT_FALSE(newly_reached_node(prev, curr).has_value());
}

// Regression: a released-flag flip (master releasing a horizon node) is an
// order update, NOT a traversal — must not report a reached node.
TEST(StateEventDetectorTest, NewlyReachedNodeNulloptOnReleasedFlipOnly)
{
  vda5050_core::types::State prev;
  prev.last_node_id = "n1";
  prev.last_node_sequence_id = 2;
  prev.node_states = {make_node("n2", false)};
  vda5050_core::types::State curr;
  curr.last_node_id = "n1";  // AGV has not advanced
  curr.last_node_sequence_id = 2;
  curr.node_states = {make_node("n2", true)};  // horizon -> base

  EXPECT_FALSE(newly_reached_node(prev, curr).has_value());
}

TEST(StateEventDetectorTest, NewlyReachedNodeOnFirstStateWithNode)
{
  vda5050_core::types::State prev;  // default: empty last_node_id, seq 0
  vda5050_core::types::State curr;
  curr.last_node_id = "n0";
  curr.last_node_sequence_id = 0;

  auto r = newly_reached_node(prev, curr);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->node_id, "n0");
}

TEST(StateEventDetectorTest, NewlyReachedNodeNulloptWhenNoLastNode)
{
  vda5050_core::types::State prev;
  vda5050_core::types::State curr;  // both empty last_node_id
  EXPECT_FALSE(newly_reached_node(prev, curr).has_value());
}

// ============================================================================
// errors_appeared / errors_resolved
// ============================================================================

TEST(StateEventDetectorTest, ErrorsAppearedReturnsNewOnly)
{
  vda5050_core::types::State prev;
  prev.errors = {make_error("E1", "old")};
  vda5050_core::types::State curr;
  curr.errors = {make_error("E1", "old"), make_error("E2", "new")};

  auto appeared = errors_appeared(prev, curr);
  ASSERT_EQ(appeared.size(), 1u);
  EXPECT_EQ(appeared[0].error_type, "E2");
}

TEST(StateEventDetectorTest, ErrorsDistinguishedByReferences)
{
  // Same type + description on different sources (error_references) must be
  // treated as distinct, not collapsed.
  auto err_on = [](const std::string& node) {
    vda5050_core::types::Error e;
    e.error_type = "blocked";
    e.error_description = "path blocked";
    vda5050_core::types::ErrorReference ref;
    ref.reference_key = "nodeId";
    ref.reference_value = node;
    e.error_references = std::vector<vda5050_core::types::ErrorReference>{ref};
    return e;
  };
  vda5050_core::types::State prev;
  prev.errors = {err_on("n1")};
  vda5050_core::types::State curr;
  curr.errors = {err_on("n1"), err_on("n2")};

  auto appeared = errors_appeared(prev, curr);
  ASSERT_EQ(appeared.size(), 1u);
  ASSERT_TRUE(appeared[0].error_references.has_value());
  EXPECT_EQ(appeared[0].error_references->front().reference_value, "n2");
}

TEST(StateEventDetectorTest, ErrorsAppearedEmptyWhenNoNewErrors)
{
  vda5050_core::types::State prev;
  prev.errors = {make_error("E1")};
  vda5050_core::types::State curr;
  curr.errors = {make_error("E1")};

  EXPECT_TRUE(errors_appeared(prev, curr).empty());
}

TEST(StateEventDetectorTest, ErrorsResolvedReturnsRemovedOnly)
{
  vda5050_core::types::State prev;
  prev.errors = {make_error("E1", "old"), make_error("E2", "vanishing")};
  vda5050_core::types::State curr;
  curr.errors = {make_error("E1", "old")};

  auto resolved = errors_resolved(prev, curr);
  ASSERT_EQ(resolved.size(), 1u);
  EXPECT_EQ(resolved[0].error_type, "E2");
}

TEST(StateEventDetectorTest, ErrorsResolvedEmptyWhenNothingRemoved)
{
  vda5050_core::types::State prev;
  prev.errors = {make_error("E1")};
  vda5050_core::types::State curr;
  curr.errors = {make_error("E1"), make_error("E2")};

  EXPECT_TRUE(errors_resolved(prev, curr).empty());
}

// ============================================================================
// new_base_requested
// ============================================================================

TEST(StateEventDetectorTest, NewBaseRequestedOnRisingEdge)
{
  vda5050_core::types::State prev;
  prev.new_base_request = false;
  vda5050_core::types::State curr;
  curr.new_base_request = true;

  EXPECT_TRUE(new_base_requested(prev, curr));
}

TEST(StateEventDetectorTest, NewBaseRequestedFalseWhenSustained)
{
  vda5050_core::types::State prev;
  prev.new_base_request = true;
  vda5050_core::types::State curr;
  curr.new_base_request = true;

  EXPECT_FALSE(new_base_requested(prev, curr));
}

TEST(StateEventDetectorTest, NewBaseRequestedHandlesAbsentPrev)
{
  vda5050_core::types::State prev;  // optional<bool> defaults to nullopt
  vda5050_core::types::State curr;
  curr.new_base_request = true;

  EXPECT_TRUE(new_base_requested(prev, curr));
}

// ============================================================================
// mode_changed
// ============================================================================

TEST(StateEventDetectorTest, ModeChangedOnTransition)
{
  vda5050_core::types::State prev;
  prev.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;
  vda5050_core::types::State curr;
  curr.operating_mode = vda5050_core::types::OperatingMode::MANUAL;

  EXPECT_TRUE(mode_changed(prev, curr));
}

TEST(StateEventDetectorTest, ModeChangedFalseWhenSame)
{
  vda5050_core::types::State prev;
  prev.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;
  vda5050_core::types::State curr;
  curr.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;

  EXPECT_FALSE(mode_changed(prev, curr));
}

// ============================================================================
// paused_changed
// ============================================================================

TEST(StateEventDetectorTest, PausedChangedOnRisingEdge)
{
  vda5050_core::types::State prev;
  prev.paused = false;
  vda5050_core::types::State curr;
  curr.paused = true;

  EXPECT_TRUE(paused_changed(prev, curr));
}

TEST(StateEventDetectorTest, PausedChangedOnFallingEdge)
{
  vda5050_core::types::State prev;
  prev.paused = true;
  vda5050_core::types::State curr;
  curr.paused = false;

  EXPECT_TRUE(paused_changed(prev, curr));
}

TEST(StateEventDetectorTest, PausedChangedFalseWhenSame)
{
  vda5050_core::types::State prev;
  prev.paused = true;
  vda5050_core::types::State curr;
  curr.paused = true;

  EXPECT_FALSE(paused_changed(prev, curr));
}

// ============================================================================
// driving_changed
// ============================================================================

TEST(StateEventDetectorTest, DrivingChangedOnRisingEdge)
{
  vda5050_core::types::State prev;
  prev.driving = false;
  vda5050_core::types::State curr;
  curr.driving = true;

  EXPECT_TRUE(driving_changed(prev, curr));
}

TEST(StateEventDetectorTest, DrivingChangedFalseWhenSame)
{
  vda5050_core::types::State prev;
  prev.driving = true;
  vda5050_core::types::State curr;
  curr.driving = true;

  EXPECT_FALSE(driving_changed(prev, curr));
}

// ============================================================================
// loads_changed
// ============================================================================

TEST(StateEventDetectorTest, LoadsChangedOnAddition)
{
  vda5050_core::types::State prev;
  prev.loads = std::vector<vda5050_core::types::Load>{};
  vda5050_core::types::State curr;
  curr.loads = std::vector<vda5050_core::types::Load>{make_load("L1")};

  EXPECT_TRUE(loads_changed(prev, curr));
}

TEST(StateEventDetectorTest, LoadsChangedOnRemoval)
{
  vda5050_core::types::State prev;
  prev.loads = std::vector<vda5050_core::types::Load>{make_load("L1")};
  vda5050_core::types::State curr;
  curr.loads = std::vector<vda5050_core::types::Load>{};

  EXPECT_TRUE(loads_changed(prev, curr));
}

TEST(StateEventDetectorTest, LoadsChangedFalseWhenSame)
{
  vda5050_core::types::State prev;
  prev.loads = std::vector<vda5050_core::types::Load>{make_load("L1")};
  vda5050_core::types::State curr;
  curr.loads = std::vector<vda5050_core::types::Load>{make_load("L1")};

  EXPECT_FALSE(loads_changed(prev, curr));
}

}  // namespace vda5050_core::master::event::test
