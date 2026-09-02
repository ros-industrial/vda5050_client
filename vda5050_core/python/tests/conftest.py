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

from unittest.mock import create_autospec

import pytest
from vda5050_core.transport import MqttClientInterface


class MockMqttClientInterface(MqttClientInterface):
    def __init__(self):
        super().__init__()
        self.mock = create_autospec(spec=MqttClientInterface)

    def __getattribute__(self, name):
        _mock = object.__getattribute__(self, "mock")
        if name == "mock":
            return _mock
        return getattr(_mock, name)


@pytest.fixture
def mock_mqtt_client():
    return MockMqttClientInterface()
