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

#include "vda5050_core/layout/layout_loader.hpp"

#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/json_utils/layout_serialization.hpp"
#include "vda5050_core/layout/validate_layout.hpp"

namespace vda5050_core::layout {

namespace {

void add(
  std::vector<LayoutLoadError>& errs, LayoutLoadErrorType t, std::string desc)
{
  errs.push_back({t, std::move(desc)});
}

// Run `body` under per-element try/catch, mapping nlohmann exceptions to
// LayoutLoadErrorType and prepending `crumb` (e.g. "layouts[2].nodes[7]") to
// the error description. Returns true on success.
template <typename F>
bool try_with_breadcrumb(
  std::vector<LayoutLoadError>& errs, const std::string& crumb, F&& body)
{
  try
  {
    body();
    return true;
  }
  catch (const nlohmann::json::out_of_range& e)
  {
    add(
      errs, LayoutLoadErrorType::MISSING_REQUIRED_FIELD,
      crumb + ": " + e.what());
  }
  catch (const nlohmann::json::type_error& e)
  {
    add(
      errs, LayoutLoadErrorType::INVALID_FIELD_VALUE, crumb + ": " + e.what());
  }
  catch (const nlohmann::json::exception& e)
  {
    add(errs, LayoutLoadErrorType::JSON_PARSE_ERROR, crumb + ": " + e.what());
  }
  catch (const std::invalid_argument& e)
  {
    add(
      errs, LayoutLoadErrorType::INVALID_FIELD_VALUE, crumb + ": " + e.what());
  }
  return false;
}

template <typename T, typename F>
void parse_element(
  const nlohmann::json& j, const std::string& crumb,
  std::vector<LayoutLoadError>& errs, F&& on_success)
{
  T parsed{};
  if (try_with_breadcrumb(errs, crumb, [&]() { parsed = j.get<T>(); }))
  {
    on_success(std::move(parsed));
  }
}

}  // namespace

LayoutLoadResult load_from_file(const std::string& path)
{
  std::vector<LayoutLoadError> errors;

  std::ifstream in(path);
  if (!in.is_open())
  {
    add(
      errors, LayoutLoadErrorType::FILE_NOT_FOUND,
      "Cannot open LIF config file '" + path + "'.");
    return LayoutLoadResult{std::nullopt, std::move(errors)};
  }

  std::stringstream buf;
  buf << in.rdbuf();
  if (in.bad())
  {
    add(
      errors, LayoutLoadErrorType::FILE_READ_FAILED,
      "I/O error while reading LIF config file '" + path + "'.");
    return LayoutLoadResult{std::nullopt, std::move(errors)};
  }

  nlohmann::json parsed;
  try
  {
    parsed = nlohmann::json::parse(buf.str());
  }
  catch (const nlohmann::json::parse_error& e)
  {
    add(
      errors, LayoutLoadErrorType::JSON_PARSE_ERROR,
      std::string("JSON parse error in '") + path + "': " + e.what());
    return LayoutLoadResult{std::nullopt, std::move(errors)};
  }

  return load_from_json(parsed);
}

LayoutLoadResult load_from_json(const nlohmann::json& json)
{
  std::vector<LayoutLoadError> errors;

  if (!json.is_object())
  {
    add(
      errors, LayoutLoadErrorType::JSON_PARSE_ERROR,
      "Top-level JSON value is not an object.");
    return LayoutLoadResult{std::nullopt, std::move(errors)};
  }

  LIF lif;

  if (!json.contains("metaInformation"))
  {
    add(
      errors, LayoutLoadErrorType::MISSING_REQUIRED_FIELD,
      "metaInformation: missing required field");
  }
  else
  {
    parse_element<MetaInformation>(
      json.at("metaInformation"), "metaInformation", errors,
      [&](MetaInformation mi) { lif.meta_information = std::move(mi); });
  }

  if (!json.contains("layouts"))
  {
    add(
      errors, LayoutLoadErrorType::MISSING_REQUIRED_FIELD,
      "layouts: missing required field");
  }
  else if (!json.at("layouts").is_array())
  {
    add(
      errors, LayoutLoadErrorType::INVALID_FIELD_VALUE,
      "layouts: must be an array");
  }
  else
  {
    const auto& layouts_json = json.at("layouts");
    for (std::size_t li = 0; li < layouts_json.size(); ++li)
    {
      const auto& lj = layouts_json[li];
      const std::string lcrumb = "layouts[" + std::to_string(li) + "]";

      Layout layout;
      bool ok = try_with_breadcrumb(
        errors, lcrumb, [&]() { layout = lj.get<Layout>(); });

      if (!ok)
      {
        // Whole-Layout parse failed; walk element-by-element so a single bad
        // node/edge/station doesn't suppress error reporting on its siblings.
        if (lj.is_object())
        {
          if (lj.contains("nodes") && lj.at("nodes").is_array())
          {
            const auto& na = lj.at("nodes");
            for (std::size_t ni = 0; ni < na.size(); ++ni)
            {
              parse_element<Node>(
                na[ni], lcrumb + ".nodes[" + std::to_string(ni) + "]", errors,
                [](Node) {});
            }
          }
          if (lj.contains("edges") && lj.at("edges").is_array())
          {
            const auto& ea = lj.at("edges");
            for (std::size_t ei = 0; ei < ea.size(); ++ei)
            {
              parse_element<Edge>(
                ea[ei], lcrumb + ".edges[" + std::to_string(ei) + "]", errors,
                [](Edge) {});
            }
          }
          if (lj.contains("stations") && lj.at("stations").is_array())
          {
            const auto& sa = lj.at("stations");
            for (std::size_t si = 0; si < sa.size(); ++si)
            {
              parse_element<Station>(
                sa[si], lcrumb + ".stations[" + std::to_string(si) + "]",
                errors, [](Station) {});
            }
          }
        }
        continue;
      }

      lif.layouts.push_back(std::move(layout));
    }
  }

  if (!errors.empty())
  {
    return LayoutLoadResult{std::nullopt, std::move(errors)};
  }

  auto invariants = validate_layout(lif);
  if (!invariants.empty())
  {
    errors.insert(errors.end(), invariants.begin(), invariants.end());
    return LayoutLoadResult{std::nullopt, std::move(errors)};
  }

  return LayoutLoadResult{std::move(lif), {}};
}

}  // namespace vda5050_core::layout
