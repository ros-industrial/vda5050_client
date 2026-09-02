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
#include <memory>

#include "vda5050_core/transport/mqtt_client_interface.hpp"

#include "vda5050_core_py/transport.hpp"

namespace vda5050_core_py {

using vda5050_core::transport::create_default_client_shared;
using vda5050_core::transport::MqttClientInterface;

namespace {

class PyMqttClientInterface : public MqttClientInterface
{
public:
  using MqttClientInterface::MqttClientInterface;
  void connect() override
  {
    PYBIND11_OVERRIDE_PURE(void, MqttClientInterface, connect);
  }

  void disconnect() override
  {
    PYBIND11_OVERRIDE_PURE(void, MqttClientInterface, disconnect);
  }

  bool connected() override
  {
    PYBIND11_OVERRIDE_PURE(bool, MqttClientInterface, connected);
  }

  void publish(
    const std::string& topic, const std::string& message, int qos,
    bool retain) override
  {
    PYBIND11_OVERRIDE_PURE(
      void, MqttClientInterface, publish, topic, message, qos, retain);
  }

  void subscribe(
    const std::string& topic, MessageHandler handler, int qos) override
  {
    PYBIND11_OVERRIDE_PURE(
      void, MqttClientInterface, subscribe, topic, handler, qos);
  }

  void unsubscribe(const std::string& topic) override
  {
    PYBIND11_OVERRIDE_PURE(void, MqttClientInterface, unsubscribe, topic);
  }

  void set_will(
    const std::string& topic, const std::string& message, int qos,
    bool retain) override
  {
    PYBIND11_OVERRIDE_PURE(
      void, MqttClientInterface, set_will, topic, message, qos, retain);
  }

  void set_connection_lost_callback(ConnectionStateHandler handler) override
  {
    PYBIND11_OVERRIDE(
      void, MqttClientInterface, set_connection_lost_callback, handler);
  }

  void set_connected_callback(ConnectionStateHandler handler) override
  {
    PYBIND11_OVERRIDE(
      void, MqttClientInterface, set_connected_callback, handler);
  }
};

}  // namespace

void bind_transport(py::module_& m)
{
  auto m_transport =
    m.def_submodule("transport", "VDA5050 MQTT transport layer");

  py::class_<
    MqttClientInterface, PyMqttClientInterface,
    std::shared_ptr<MqttClientInterface> >(m_transport, "MqttClientInterface")
    .def(py::init<>())
    .def("connect", &MqttClientInterface::connect)
    .def("disconnect", &MqttClientInterface::disconnect)
    .def("connected", &MqttClientInterface::connected)
    .def(
      "publish", &MqttClientInterface::publish, py::arg("topic"),
      py::arg("message"), py::arg("qos"), py::arg("retain") = false)
    .def(
      "subscribe", &MqttClientInterface::subscribe, py::arg("topic"),
      py::arg("handler"), py::arg("qos"))
    .def("unsubscribe", &MqttClientInterface::unsubscribe, py::arg("topic"))
    .def(
      "set_will", &MqttClientInterface::set_will, py::arg("topic"),
      py::arg("message"), py::arg("qos"), py::arg("retain") = true)
    .def(
      "set_connection_lost_callback",
      &MqttClientInterface::set_connection_lost_callback, py::arg("handler"))
    .def(
      "set_connected_callback", &MqttClientInterface::set_connected_callback,
      py::arg("handler"));

  m_transport.def(
    "create_default_client_shared", &create_default_client_shared,
    py::arg("broker_address"), py::arg("client_id"),
    "Create a Paho-backed MQTT client shared_ptr.");
}

}  // namespace vda5050_core_py
