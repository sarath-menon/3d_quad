#pragma once
#include "safety_checks.h"
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace paths {

// Yaml paths

const std::string folder_path = "quadcopter_sim/quadcopter_sim_app/parameters/";

const std::string quad_yaml = folder_path + "quad_properties.yaml";

const std::string sim_yaml = folder_path + "simulation_parameters.yaml";

const std::string initial_conditions_yaml =
    folder_path + "initial_conditions.yaml";

// Text file paths

const std::string event_log = "logs/event_logs/";
const std::string data_log = "logs/data_logs/";

} // namespace paths