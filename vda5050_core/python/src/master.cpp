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

#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11_json/pybind11_json.hpp>

#include <nlohmann/json.hpp>

#include "vda5050_core/json_utils/serialization.hpp"
#include "vda5050_core/master/actions/instant_action_assignment_result.hpp"
#include "vda5050_core/master/agv.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_core/master/order/order_assignment_result.hpp"
#include "vda5050_core/types/connection_state.hpp"

#include "vda5050_core_py/master.hpp"

namespace vda5050_core_py {

using vda5050_core::master::AGV;
using vda5050_core::master::AGVState;
using vda5050_core::master::InstantActionAssignmentResult;
using vda5050_core::master::InstantActionDecision;
using vda5050_core::master::OrderAssignmentDecision;
using vda5050_core::master::OrderAssignmentResult;
using vda5050_core::master::VDA5050Master;

void bind_master(py::module_& m)
{
  auto m_master = m.def_submodule("master", "VDA5050 master fleet control API");

  // --- Enums ---

  py::enum_<AGVState>(m_master, "AGVState")
    .value("STATE_UNKNOWN", AGVState::STATE_UNKNOWN)
    .value("AVAILABLE", AGVState::AVAILABLE)
    .value("UNAVAILABLE", AGVState::UNAVAILABLE)
    .value("ERROR", AGVState::ERROR);

  py::enum_<OrderAssignmentDecision>(m_master, "OrderAssignmentDecision")
    .value("ASSIGNED", OrderAssignmentDecision::ASSIGNED)
    .value("AGV_NOT_ONBOARDED", OrderAssignmentDecision::AGV_NOT_ONBOARDED)
    .value("AGV_OFFLINE", OrderAssignmentDecision::AGV_OFFLINE)
    .value("AGV_NOT_READY", OrderAssignmentDecision::AGV_NOT_READY)
    .value("AGV_MODE_NOT_AUTO", OrderAssignmentDecision::AGV_MODE_NOT_AUTO)
    .value(
      "AGV_POSITION_NOT_INITIALIZED",
      OrderAssignmentDecision::AGV_POSITION_NOT_INITIALIZED)
    .value("AGV_NO_STATE_YET", OrderAssignmentDecision::AGV_NO_STATE_YET)
    .value("AGV_QUEUE_FULL", OrderAssignmentDecision::AGV_QUEUE_FULL)
    .value("STITCH_REJECTED", OrderAssignmentDecision::STITCH_REJECTED)
    .value("STITCH_QUEUED", OrderAssignmentDecision::STITCH_QUEUED)
    .value("DUPLICATE_IGNORED", OrderAssignmentDecision::DUPLICATE_IGNORED);

  py::enum_<InstantActionDecision>(m_master, "InstantActionDecision")
    .value("ASSIGNED", InstantActionDecision::ASSIGNED)
    .value("AGV_NOT_ONBOARDED", InstantActionDecision::AGV_NOT_ONBOARDED)
    .value("AGV_OFFLINE", InstantActionDecision::AGV_OFFLINE)
    .value("DUPLICATE_ACTION_ID", InstantActionDecision::DUPLICATE_ACTION_ID)
    .value("AGV_QUEUE_FULL", InstantActionDecision::AGV_QUEUE_FULL)
    .value("HARD_ACTION_BLOCKED", InstantActionDecision::HARD_ACTION_BLOCKED)
    .value(
      "ACTION_BLOCKED_BY_DRIVING",
      InstantActionDecision::ACTION_BLOCKED_BY_DRIVING)
    .value(
      "AGV_MODE_NOT_AUTO_FOR_ACTION",
      InstantActionDecision::AGV_MODE_NOT_AUTO_FOR_ACTION)
    .value("INVALID_CONTENT", InstantActionDecision::INVALID_CONTENT)
    .value(
      "AGV_CANNOT_PERFORM_ACTION",
      InstantActionDecision::AGV_CANNOT_PERFORM_ACTION)
    .value(
      "EXCEEDS_PROTOCOL_LIMITS",
      InstantActionDecision::EXCEEDS_PROTOCOL_LIMITS);

  // --- Result structs ---

  py::class_<OrderAssignmentResult>(m_master, "OrderAssignmentResult")
    .def_readonly("decision", &OrderAssignmentResult::decision)
    .def_readonly("errors", &OrderAssignmentResult::errors)
    .def("__bool__", [](const OrderAssignmentResult& r) {
      return static_cast<bool>(r);
    });

  py::class_<InstantActionAssignmentResult>(
    m_master, "InstantActionAssignmentResult")
    .def_readonly("decision", &InstantActionAssignmentResult::decision)
    .def_readonly("errors", &InstantActionAssignmentResult::errors)
    .def("__bool__", [](const InstantActionAssignmentResult& r) {
      return static_cast<bool>(r);
    });

  // last_disconnect_at: pybind11/chrono.h + pybind11/stl.h convert
  // std::optional<system_clock::time_point> → datetime.datetime | None.
  py::class_<VDA5050Master::BrokerStatusSnapshot>(
    m_master, "BrokerStatusSnapshot")
    .def_readonly("connected", &VDA5050Master::BrokerStatusSnapshot::connected)
    .def_readonly(
      "last_disconnect_at",
      &VDA5050Master::BrokerStatusSnapshot::last_disconnect_at)
    .def_readonly(
      "reconnect_count", &VDA5050Master::BrokerStatusSnapshot::reconnect_count);

  py::class_<VDA5050Master::OnboardSpec>(m_master, "OnboardSpec")
    .def(py::init<>())
    .def(
      py::init([](
                 const std::string& mfg, const std::string& serial,
                 std::size_t max_queue, bool drop_oldest) {
        VDA5050Master::OnboardSpec s;
        s.manufacturer = mfg;
        s.serial_number = serial;
        s.max_queue_size = max_queue;
        s.drop_oldest = drop_oldest;
        return s;
      }),
      py::arg("manufacturer"), py::arg("serial_number"),
      py::arg("max_queue_size") = 10, py::arg("drop_oldest") = true)
    .def_readwrite("manufacturer", &VDA5050Master::OnboardSpec::manufacturer)
    .def_readwrite("serial_number", &VDA5050Master::OnboardSpec::serial_number)
    .def_readwrite(
      "max_queue_size", &VDA5050Master::OnboardSpec::max_queue_size)
    .def_readwrite("drop_oldest", &VDA5050Master::OnboardSpec::drop_oldest);

  py::class_<VDA5050Master::BatchOnboardResult>(m_master, "BatchOnboardResult")
    .def_readonly("onboarded", &VDA5050Master::BatchOnboardResult::onboarded)
    .def_readonly(
      "skipped_already_onboarded",
      &VDA5050Master::BatchOnboardResult::skipped_already_onboarded)
    .def_readonly("failed", &VDA5050Master::BatchOnboardResult::failed);

  // --- AGV (read-only fleet view) ---
  py::class_<AGV, std::shared_ptr<AGV>>(m_master, "AGV")
    .def("get_interface_name", &AGV::get_interface_name)
    .def("get_manufacturer", &AGV::get_manufacturer)
    .def("get_serial_number", &AGV::get_serial_number)
    .def("get_agv_id", &AGV::get_agv_id)
    .def("is_connected", &AGV::is_connected)
    .def("get_connection_status", &AGV::get_connection_status)
    .def("get_operational_state", &AGV::get_operational_state)
    .def("get_last_state", &AGV::get_last_state)
    .def("get_last_connection", &AGV::get_last_connection)
    .def("get_last_factsheet", &AGV::get_last_factsheet)
    .def("get_last_visualization", &AGV::get_last_visualization)
    .def("has_active_order", &AGV::has_active_order)
    // std::optional<std::string> / std::optional<uint32_t> → str | None, int | None
    .def("active_order_id", &AGV::active_order_id)
    .def("active_order_update_id", &AGV::active_order_update_id)
    .def("is_order_complete", &AGV::is_order_complete)
    .def("get_pending_order_count", &AGV::get_pending_order_count)
    .def(
      "get_pending_instant_actions_count",
      &AGV::get_pending_instant_actions_count);

  // --- VDA5050Master ---
  py::class_<VDA5050Master, std::shared_ptr<VDA5050Master>>(
    m_master, "VDA5050Master")
    .def_static("make", &VDA5050Master::make, py::arg("mqtt_client"))

    // Connection management
    .def(
      "connect", &VDA5050Master::connect,
      py::call_guard<py::gil_scoped_release>())
    .def(
      "disconnect", &VDA5050Master::disconnect,
      py::call_guard<py::gil_scoped_release>())
    .def("is_connected", &VDA5050Master::is_connected)
    .def(
      "get_broker_status", &VDA5050Master::get_broker_status,
      py::call_guard<py::gil_scoped_release>())

    // AGV onboarding / offboarding
    .def(
      "onboard_agv",
      py::overload_cast<
        const std::string&, const std::string&, std::size_t, bool>(
        &VDA5050Master::onboard_agv),
      py::arg("manufacturer"), py::arg("serial_number"),
      py::arg("max_queue_size") = 10, py::arg("drop_oldest") = true,
      py::call_guard<py::gil_scoped_release>())
    .def(
      "onboard_agv_with_interface",
      py::overload_cast<
        const std::string&, const std::string&, const std::string&, std::size_t,
        bool>(&VDA5050Master::onboard_agv),
      py::arg("interface_name"), py::arg("manufacturer"),
      py::arg("serial_number"), py::arg("max_queue_size") = 10,
      py::arg("drop_oldest") = true, py::call_guard<py::gil_scoped_release>())
    .def(
      "offboard_agv", &VDA5050Master::offboard_agv, py::arg("manufacturer"),
      py::arg("serial_number"), py::call_guard<py::gil_scoped_release>())
    .def(
      "is_agv_onboarded", &VDA5050Master::is_agv_onboarded,
      py::arg("manufacturer"), py::arg("serial_number"))
    .def(
      "get_agv", &VDA5050Master::get_agv, py::arg("manufacturer"),
      py::arg("serial_number"))
    .def(
      "get_onboarded_agvs", &VDA5050Master::get_onboarded_agvs,
      py::call_guard<py::gil_scoped_release>())
    .def(
      "cancel_pending_orders", &VDA5050Master::cancel_pending_orders,
      py::arg("manufacturer"), py::arg("serial_number"),
      py::call_guard<py::gil_scoped_release>())
    .def(
      "resume_mode_cancelled_queue",
      &VDA5050Master::resume_mode_cancelled_queue, py::arg("manufacturer"),
      py::arg("serial_number"), py::call_guard<py::gil_scoped_release>())
    .def(
      "discard_mode_cancelled_queue",
      &VDA5050Master::discard_mode_cancelled_queue, py::arg("manufacturer"),
      py::arg("serial_number"), py::call_guard<py::gil_scoped_release>())
    .def(
      "onboard_agv_batch", &VDA5050Master::onboard_agv_batch, py::arg("specs"),
      py::call_guard<py::gil_scoped_release>())
    .def(
      "offboard_agv_batch", &VDA5050Master::offboard_agv_batch, py::arg("keys"),
      py::call_guard<py::gil_scoped_release>())

    .def(
      "publish_order", &VDA5050Master::publish_order, py::arg("manufacturer"),
      py::arg("serial_number"), py::arg("order"),
      py::call_guard<py::gil_scoped_release>())
    .def(
      "assign_order", &VDA5050Master::assign_order, py::arg("manufacturer"),
      py::arg("serial_number"), py::arg("order"),
      py::call_guard<py::gil_scoped_release>())
    .def(
      "publish_instant_actions", &VDA5050Master::publish_instant_actions,
      py::arg("manufacturer"), py::arg("serial_number"), py::arg("actions"),
      py::call_guard<py::gil_scoped_release>())
    .def(
      "assign_instant_actions", &VDA5050Master::assign_instant_actions,
      py::arg("manufacturer"), py::arg("serial_number"), py::arg("actions"),
      py::call_guard<py::gil_scoped_release>())

    .def(
      "load_layout_from_config",
      [](VDA5050Master& self, const std::string& path) {
        vda5050_core::layout::LayoutLoadResult result;
        {
          py::gil_scoped_release release;
          result = self.load_layout_from_config(path);
        }
        py::list errors;
        for (const auto& e : result.errors)
        {
          errors.append(e.description);
        }
        py::dict d;
        d["success"] = static_cast<bool>(result);
        d["errors"] = errors;
        return d;
      },
      py::arg("path"))

    // Callbacks
    .def("on_state", &VDA5050Master::on_state)
    .def("on_connection", &VDA5050Master::on_connection)
    .def("on_factsheet", &VDA5050Master::on_factsheet)
    .def("on_visualization", &VDA5050Master::on_visualization)

    .def("on_node_reached", &VDA5050Master::on_node_reached)
    .def("on_order_complete", &VDA5050Master::on_order_complete)
    .def("on_order_rejected", &VDA5050Master::on_order_rejected)
    .def("on_errors_appeared", &VDA5050Master::on_errors_appeared)
    .def("on_errors_resolved", &VDA5050Master::on_errors_resolved)
    .def("on_new_base_requested", &VDA5050Master::on_new_base_requested)
    .def("on_mode_changed", &VDA5050Master::on_mode_changed)
    .def("on_paused", &VDA5050Master::on_paused)
    .def("on_driving", &VDA5050Master::on_driving)
    .def("on_loads_changed", &VDA5050Master::on_loads_changed)

    .def("on_connect", &VDA5050Master::on_connect)
    .def("on_offline", &VDA5050Master::on_offline)
    .def("on_connection_broken", &VDA5050Master::on_connection_broken)

    // State heartbeat callbacks
    .def("on_state_timeout", &VDA5050Master::on_state_timeout)
    .def("on_state_resumed", &VDA5050Master::on_state_resumed)

    // Broker connection callbacks
    .def("on_broker_disconnected", &VDA5050Master::on_broker_disconnected)
    .def("on_broker_reconnected", &VDA5050Master::on_broker_reconnected);
}

}  // namespace vda5050_core_py
