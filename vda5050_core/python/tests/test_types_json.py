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

import random
import string

import pytest
from vda5050_core import types as t

NUM_RANDOM_TESTS = 10

SERIALIZABLE_TYPES = [
    t.Action,
    t.ActionParameter,
    t.ActionParameterFactsheet,
    t.ActionState,
    t.AGVAction,
    t.AGVGeometry,
    t.AGVPosition,
    t.BatteryState,
    t.BoundingBoxReference,
    t.Connection,
    t.ControlPoint,
    t.Edge,
    t.EdgeState,
    t.Envelope2d,
    t.Envelope3d,
    t.Error,
    t.ErrorReference,
    t.Factsheet,
    t.Header,
    t.Info,
    t.InfoReference,
    t.InstantActions,
    t.Load,
    t.LoadDimensions,
    t.LoadSet,
    t.LoadSpecification,
    t.MaxArrayLens,
    t.MaxStringLens,
    t.Node,
    t.NodePosition,
    t.NodeState,
    t.OptionalParameter,
    t.Order,
    t.PhysicalParameters,
    t.PolygonPoint,
    t.Position,
    t.ProtocolFeatures,
    t.ProtocolLimits,
    t.SafetyState,
    t.State,
    t.Timing,
    t.Trajectory,
    t.TypeSpecification,
    t.Velocity,
    t.Visualization,
    t.WheelDefinition,
]


class RandomDataGenerator:
    def _random_string(self, length: int = 8) -> str:
        return "".join(random.choices(string.ascii_letters + string.digits, k=length))

    def _random_float(self) -> float:
        return random.uniform(-100.0, 100.0)

    def _random_int(self) -> int:
        return random.randint(0, 1000)

    def _random_bool(self) -> bool:
        return random.choice([True, False])

    def _random_enum(self, enum_type):
        return random.choice(list(enum_type.__members__.values()))

    def generate(self, type_class):
        if hasattr(type_class, "__members__"):
            return self._random_enum(type_class)

        obj = type_class()
        for name in dir(obj):
            if name.startswith("_"):
                continue
            val = getattr(obj, name)
            if callable(val):
                continue
            if val is None:
                continue  # optional field, leave as None
            if isinstance(val, list):
                continue  # vector field, leave empty
            if isinstance(val, bool):
                setattr(obj, name, self._random_bool())
            elif isinstance(val, int):
                setattr(obj, name, self._random_int())
            elif isinstance(val, float):
                if name == "timestamp":
                    # ISO 8601 serialization has millisecond precision; keep positive
                    setattr(obj, name, float(random.randint(0, 10**9)))
                else:
                    setattr(obj, name, self._random_float())
            elif isinstance(val, str):
                setattr(obj, name, self._random_string())
            elif hasattr(type(val), "__members__"):
                setattr(obj, name, self._random_enum(type(val)))
            else:
                setattr(obj, name, self.generate(type(val)))

        return obj


_generator = RandomDataGenerator()


@pytest.mark.parametrize(
    "type_class",
    SERIALIZABLE_TYPES,
    ids=[c.__name__ for c in SERIALIZABLE_TYPES],
)
def test_json_roundtrip(type_class):
    for _ in range(NUM_RANDOM_TESTS):
        obj = _generator.generate(type_class)
        result = type_class.from_json(obj.json())
        assert result == obj
