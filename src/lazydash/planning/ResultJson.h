#pragma once

#include "lazydash/planning/LazyDash.h"
#include "lazydash/scene/Scene.h"

#include <string>

namespace lazydash {

/// Write a run as CoMotion viewer `*_result.json` plus a `lazydash` block (success,
/// statistics, task plan, log). `urdf_path` is relative to the viewer's root;
/// `stride` keeps every stride-th timestep.
std::string resultJson(const Scene &scene, const LazyDashResult &result, const std::string &urdf_path,
                       int stride = 1);

/// Only the statistics, as one JSON object (for benchmark logs).
std::string statsJson(const LazyDashResult &result);

} // namespace lazydash
