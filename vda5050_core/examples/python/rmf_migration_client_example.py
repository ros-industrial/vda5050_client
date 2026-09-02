# Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
# Advanced Remanufacturing and Technology Centre
# A*STAR Research Entities (Co. Registration No. 199702110H)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Run one simulated VDA5050 robot adapter against an MQTT broker."""

from __future__ import annotations

import logging
import os
import time

from vda5050_core.rmf_migration import (
    Adapter,
    FleetConfiguration,
    RobotCallbacks,
    RobotConfiguration,
    RobotState,
)

BROKER_URI = os.environ.get("MQTT_BROKER", "tcp://localhost:1883")
MQTT_CLIENT_ID = os.environ.get(
    "ROBOT_MQTT_CLIENT_ID", "example-robot-adapter-manufacturer-s001"
)
MANUFACTURER = os.environ.get("VDA5050_MANUFACTURER", "Manufacturer")
SERIAL_NUMBER = os.environ.get("VDA5050_SERIAL_NUMBER", "S001")
MAP_ID = os.environ.get("VDA5050_MAP_ID", "demo-map")
LOGGER = logging.getLogger(__name__)


def main() -> None:
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    adapter = Adapter.make()
    fleet_config = FleetConfiguration(
        fleet_name="demo",
        broker_uri=BROKER_URI,
        client_id_prefix=MQTT_CLIENT_ID,
    )
    fleet = adapter.add_vda5050_fleet(fleet_config)

    robot_config = RobotConfiguration(
        manufacturer=MANUFACTURER,
        serial_number=SERIAL_NUMBER,
        interface_name="uagv",
        version="2.0.0",
    )
    initial_state = RobotState(MAP_ID, [0.0, 0.0, 0.0], 1.0)

    robot_handle = None

    def navigate(destination, execution) -> None:
        LOGGER.info(
            "Navigate to %s at %s",
            destination.name,
            destination.position,
        )
        if robot_handle is not None:
            state = RobotState(MAP_ID, destination.position, 1.0)
            robot_handle.update(state, execution.identifier)
        execution.finished()

    def stop() -> None:
        LOGGER.info("Stop requested")

    def execute_action(action_type, action_id, execution) -> None:
        LOGGER.info("Action: %s (%s)", action_type, action_id)
        execution.finished()

    callbacks = RobotCallbacks(navigate, stop, execute_action)
    robot_handle = fleet.add_robot(
        "robot-1",
        initial_state,
        robot_config,
        callbacks,
    )

    adapter.start()
    LOGGER.info(
        "Robot adapter online: %s/%s via %s",
        MANUFACTURER,
        SERIAL_NUMBER,
        BROKER_URI,
    )
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        adapter.stop()


if __name__ == "__main__":
    main()
