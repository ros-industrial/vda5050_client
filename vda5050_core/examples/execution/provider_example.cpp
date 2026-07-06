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

#include <memory>

#include <vda5050_core/logger/logger.hpp>

#include "vda5050_core/execution/base.hpp"
#include "vda5050_core/execution/provider.hpp"

using UpdateBase = vda5050_core::execution::UpdateBase;
using Provider = vda5050_core::execution::Provider;

struct PositionData
: public vda5050_core::execution::Initialize<PositionData, UpdateBase>
{
  double x;
  double y;

  PositionData(double x, double y) : x(x), y(y)
  {
    // Nothing to do here ...
  }
};

struct DiagnosticData
: public vda5050_core::execution::Initialize<DiagnosticData, UpdateBase>
{
  std::string error_code;

  explicit DiagnosticData(const std::string& error_code)
  : error_code(std::move(error_code))
  {
    // Nothing to do here ...
  }
};

int main()
{
  auto provider = std::make_shared<Provider>();

  provider->on<PositionData>([](auto update) {
    VDA5050_INFO_STREAM(
      "Listener 1 (Navigation): Received Position(" << update->x << ", "
                                                    << update->y << ")");
  });

  provider->on<DiagnosticData>([](auto update) {
    VDA5050_INFO_STREAM(
      "Listener 2 (Safety): Received Error Code [" << update->error_code
                                                   << "]");
  });

  VDA5050_INFO("Starting Provider Simulation ...");

  provider->push<PositionData>(4.5, 9.3);
  provider->push<DiagnosticData>("E400_SERVER_UNAVAILABLE");

  auto stored_position = std::make_shared<PositionData>(10.7, 7.2);
  provider->push_shared(stored_position);

  return 0;
}
