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

#ifndef VDA5050_CORE__ERRORS__ERROR_CODES_HPP_
#define VDA5050_CORE__ERRORS__ERROR_CODES_HPP_

#include <string>

namespace vda5050_core {

namespace errors {

// Error Codes for vda5050_types::Error::error_type field
inline const std::string GraphValidationError = "graphValidationError";
inline const std::string OrderUpdateError = "orderUpdateError";
inline const std::string ValidationError = "validationError";
inline const std::string SchemaValidationError = "schemaValidationError";
inline const std::string PreSendValidationError = "preSendValidationError";

// Reference key string for vda5050_types::ErrorReference::key
inline const std::string RefOrderId = "orderId";
inline const std::string RefOrderUpdateId = "orderUpdateId";
inline const std::string RefNodeId = "nodeId";
inline const std::string RefEdgeId = "edgeId";
inline const std::string RefSequenceId = "sequenceId";

}  // namespace errors
}  // namespace vda5050_core

#endif  // VDA5050_CORE__ERRORS__ERROR_CODES_HPP_
