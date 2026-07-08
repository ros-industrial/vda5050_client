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

#include "vda5050_core/client/adapter/localization_request.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

//=============================================================================
double LocalizationRequest::x() const
{
  return x_;
}

//=============================================================================
double LocalizationRequest::y() const
{
  return y_;
}

//=============================================================================
double LocalizationRequest::theta() const
{
  return theta_;
}

//=============================================================================
const std::string& LocalizationRequest::map_id() const
{
  return map_id_;
}

//=============================================================================
LocalizationRequest::LocalizationRequest(
  double x, double y, double theta, const std::string& map_id)
: x_(x), y_(y), theta_(theta), map_id_(map_id)
{
  // Nothing to do here ...
}

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core
