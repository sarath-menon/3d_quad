#pragma once

#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>
#include <yaml-cpp/yaml.h>

using namespace mahi::gui;
using namespace mahi::util;

// Inherit from Application
class MyApp : public Application {
public:
  // 640x480 px window
  MyApp() : Application(1080, 720, "My App") {}
  // Override update (called once per frame)

  void update() override;
};

// Initialize shared plotting flags
namespace plot_flags {
// Load yaml file containing plotting properties
inline YAML::Node plot_yaml = YAML::LoadFile(
    "quadcopter_sim/quadcopter_sim_app/parameters/plot_parameters.yaml");

// Load properties from yaml file
const bool plot_enable = plot_yaml["plot_enable"].as<bool>();

const bool x_flag = plot_yaml["x_flag"].as<bool>();
const bool y_flag = plot_yaml["y_flag"].as<bool>();
const bool z_flag = plot_yaml["z_flag"].as<bool>();

const bool roll_angle_flag = plot_yaml["roll_angle_flag"].as<bool>();
const bool pitch_angle_flag = plot_yaml["pitch_angle_flag"].as<bool>();
const bool yaw_angle_flag = plot_yaml["yaw_angle_flag"].as<bool>();

const bool body_thrust_flag = plot_yaml["body_thrust_flag"].as<bool>();

const bool body_roll_torque_flag =
    plot_yaml["body_roll_torque_flag"].as<bool>();
const bool body_pitch_torque_flag =
    plot_yaml["body_pitch_torque_flag"].as<bool>();
const bool body_yaw_torque_flag = plot_yaml["body_yaw_torque_flag"].as<bool>();

} // namespace plot_flags

// Initialize shared plotting varibles
namespace plot_var {
constexpr static int euler_timesteps = 500;

// Variables to be plotted
inline float x[euler_timesteps];
inline float y[euler_timesteps];
inline float z[euler_timesteps];

inline float roll_angle[euler_timesteps];
inline float pitch_angle[euler_timesteps];
inline float yaw_angle[euler_timesteps];

inline float x_setpoint[euler_timesteps];
inline float y_setpoint[euler_timesteps];
inline float z_setpoint[euler_timesteps];

inline float roll_angle_setpoint[euler_timesteps];
inline float pitch_angle_setpoint[euler_timesteps];
inline float yaw_angle_setpoint[euler_timesteps];

inline float thrust[euler_timesteps];
inline float roll_torque[euler_timesteps];
inline float pitch_torque[euler_timesteps];
inline float yaw_torque[euler_timesteps];

inline float t[euler_timesteps];

// Plot axes limits
constexpr static int x_min = 0;
constexpr static int x_max = 5;
constexpr static int y_min = 0;
constexpr static int y_max = 20;

} // namespace plot_var