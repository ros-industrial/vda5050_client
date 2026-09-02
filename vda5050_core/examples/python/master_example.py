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

"""Run a VDA5050 master that observes the paired robot adapter."""

import logging
import os
import time

from vda5050_core.master import VDA5050Master
from vda5050_core.transport import create_default_client_shared

BROKER_URI = os.environ.get("MQTT_BROKER", "tcp://localhost:1883")
MQTT_CLIENT_ID = os.environ.get("MASTER_MQTT_CLIENT_ID", "example-master")
MANUFACTURER = os.environ.get("VDA5050_MANUFACTURER", "Manufacturer")
SERIAL_NUMBER = os.environ.get("VDA5050_SERIAL_NUMBER", "S001")
LOGGER = logging.getLogger(__name__)


class MasterObserver:
    def on_connect(self, agv_id) -> None:
        LOGGER.info("AGV connected: %s", agv_id)

    def on_offline(self, agv_id) -> None:
        LOGGER.info("AGV offline: %s", agv_id)

    def on_connection_broken(self, agv_id) -> None:
        LOGGER.warning("AGV connection broken: %s", agv_id)

    def on_state(self, agv_id, state) -> None:
        position = state.agv_position
        pose = None if position is None else (position.x, position.y, position.theta)
        LOGGER.info(
            "State: %s pose=%s battery=%s",
            agv_id,
            pose,
            state.battery_state.battery_charge,
        )


def main() -> None:
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    mqtt_client = create_default_client_shared(BROKER_URI, MQTT_CLIENT_ID)
    master = VDA5050Master.make(mqtt_client)
    master_observer = MasterObserver()

    master.on_connect(master_observer.on_connect)
    master.on_offline(master_observer.on_offline)
    master.on_state(master_observer.on_state)
    master.on_connection_broken(master_observer.on_connection_broken)

    master.connect()
    master.onboard_agv(MANUFACTURER, SERIAL_NUMBER)
    LOGGER.info(
        "Master listening for %s/%s via %s",
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
        master.offboard_agv(MANUFACTURER, SERIAL_NUMBER)
        master.disconnect()


if __name__ == "__main__":
    main()
