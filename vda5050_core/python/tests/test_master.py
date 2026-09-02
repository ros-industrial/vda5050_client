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

from datetime import datetime

import pytest
from vda5050_core.master import AGV, VDA5050Master


def test_master_imports():
    assert AGV is not None
    assert VDA5050Master is not None


@pytest.fixture
def mock_mqtt_client(mock_mqtt_client):
    # Define connection lost callback behavior
    on_connection_lost = None

    def set_connection_lost_callback(func):
        nonlocal on_connection_lost
        on_connection_lost = func

    # Define connected callback behavior
    on_connected = None

    def set_connected_callback(func):
        nonlocal on_connected
        on_connected = func

    connected = False

    def connect():
        nonlocal connected
        connected = True
        if on_connected:
            on_connected("connected!")

    def disconnect():
        nonlocal connected
        connected = False
        if on_connection_lost:
            on_connection_lost("disconnected")

    mock_mqtt_client.connected.side_effect = lambda: connected
    mock_mqtt_client.connect.side_effect = connect
    mock_mqtt_client.disconnect.side_effect = disconnect
    mock_mqtt_client.set_connection_lost_callback.side_effect = (
        set_connection_lost_callback
    )
    mock_mqtt_client.set_connected_callback.side_effect = set_connected_callback

    return mock_mqtt_client


def test_master_mock_transport_connect(mock_mqtt_client):
    master = VDA5050Master.make(mock_mqtt_client)
    assert master.is_connected() is False
    assert mock_mqtt_client.connected.call_count == 1

    master.connect()
    assert mock_mqtt_client.connect.call_count == 1
    assert master.is_connected() is True

    master.disconnect()
    assert mock_mqtt_client.disconnect.call_count == 1
    assert master.is_connected() is False


def test_master_last_connected(mock_mqtt_client):
    master = VDA5050Master.make(mock_mqtt_client)

    # connect
    master.connect()
    broker_status = master.get_broker_status()
    assert broker_status.connected is True
    assert broker_status.last_disconnect_at is None

    # disconnect from master
    disconnected_at = datetime.now()  # noqa: DTZ005
    master.disconnect()
    broker_status = master.get_broker_status()
    assert broker_status.connected is False

    # check conversion to datetime object
    assert isinstance(broker_status.last_disconnect_at, datetime)
    assert broker_status.last_disconnect_at > disconnected_at
