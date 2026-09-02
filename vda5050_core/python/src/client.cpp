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

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11_json/pybind11_json.hpp>

#include "vda5050_core/client/adapter/state_manager.hpp"
#include "vda5050_core/client/adapter/transformation.hpp"
#include "vda5050_core/types/action_state.hpp"
#include "vda5050_core/types/action_status.hpp"
#include "vda5050_core/types/agv_position.hpp"
#include "vda5050_core/types/battery_state.hpp"
#include "vda5050_core/types/e_stop.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/error_level.hpp"
#include "vda5050_core/types/error_reference.hpp"
#include "vda5050_core/types/info.hpp"
#include "vda5050_core/types/info_level.hpp"
#include "vda5050_core/types/info_reference.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/safety_state.hpp"
#include "vda5050_core/types/velocity.hpp"

#include "vda5050_core_py/client.hpp"

namespace vda5050_core_py {

using vda5050_core::client::adapter::Pose2D;
using vda5050_core::client::adapter::StateManager;
using vda5050_core::client::adapter::Transformation;

void bind_client(py::module_& m)
{
  auto m_client = m.def_submodule("client", "Native VDA5050 client API");

  py::class_<Pose2D>(m_client, "Pose2D")
    .def(py::init<>())
    .def_readwrite("x", &Pose2D::x)
    .def_readwrite("y", &Pose2D::y)
    .def_readwrite("theta", &Pose2D::theta);

  py::class_<Transformation>(m_client, "Transformation")
    .def_static("calibrate", &Transformation::calibrate)
    .def("to_world_pose", &Transformation::to_world_pose)
    .def("to_agv_pose", &Transformation::to_agv_pose);

  py::class_<StateManager, std::shared_ptr<StateManager>>(
    m_client, "StateManager")
    .def("set_position", &StateManager::set_position)
    .def("set_velocity", &StateManager::set_velocity)
    .def("set_driving", &StateManager::set_driving)
    .def("set_paused", &StateManager::set_paused)
    .def("set_new_base_request", &StateManager::set_new_base_request)
    .def(
      "set_distance_since_last_node",
      &StateManager::set_distance_since_last_node)
    .def("set_battery_state", &StateManager::set_battery_state)
    .def("set_operating_mode", &StateManager::set_operating_mode)
    .def("set_safety_state", &StateManager::set_safety_state)
    .def("add_action_state", &StateManager::add_action_state)
    .def("set_action_states", &StateManager::set_action_states)
    .def("clear_action_states", &StateManager::clear_action_states)
    .def("add_error", &StateManager::add_error)
    .def("set_errors", &StateManager::set_errors)
    .def("clear_errors", &StateManager::clear_errors)
    .def("add_information", &StateManager::add_information)
    .def("set_information", &StateManager::set_information)
    .def("remove_information", &StateManager::remove_information)
    .def("initialize_position", &StateManager::initialize_position)
    .def("set_transformation", &StateManager::set_transformation);
}

}  // namespace vda5050_core_py
