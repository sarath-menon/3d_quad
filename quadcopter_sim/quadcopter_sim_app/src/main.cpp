#include "motor_mixing.h"
#include "pid_cascaded.h"
#include "plot.h"
#include "quadcopter.h"
#include "simulator.h"
#include <iostream>
// Fastdds Headers
#include "mocap_quadcopterPubSubTypes.h"
#include "mocap_quadcopterPublisher.h"

int main() {

  Quadcopter quad;
  Simulator sim;
  PidCascadedController controller;

  sim.set_parameters("quadcopter_sim/quadcopter_sim_app/parameters/"
                     "simulation_parameters.yaml");

  quad.set_parameters();
  quad.set_initial_conditions();

  // // Fastdds publisher and message initialization
  mocap_quadcopterPublisher pose_pub;
  bool fastdds_flag = false;

  if (sim.pose_pub_flag()) {
    fastdds_flag = pose_pub.init();
  }

  // To be moved to external controller file
  // const float altitude_target = 2;
  // const float y_target = 2;

  const float x_target = 2;
  const float y_target = 2;
  const float z_target = 2;

  // Declare for now
  float motor_commands[4] = {0, 0, 0, 0};
  // float torque_commands[3] = {0, 0, 0};

  matrix::Vector3f body_thrust_command;
  matrix::Vector3f body_torque_command;
  matrix::Vector3f attitude_command;

  ///////////////////////////////////////////////////////////////////////////////////////////
  // Start Simulation
  ///////////////////////////////////////////////////////////////////////////////////////////

  for (int i = 0; i < sim.euler_steps(); i++) {
    // Print simulation timestep
    std::cout << "Timestep:" << i + 1 << '\n';

    // Get system state
    quad.sensor_read();

    // Altitude controller
    body_thrust_command(2) =
        controller.altitude_controller(quad, z_target, sim.dt());

    // Reduced attitude controller
    attitude_command(0) = controller.y_pos_controller(quad, y_target, sim.dt());
    attitude_command(1) = controller.x_pos_controller(quad, x_target, sim.dt());

    // Angular rate controllers
    body_torque_command(0) =
        controller.roll_angle_controller(quad, attitude_command(0), sim.dt());
    body_torque_command(1) =
        controller.pitch_angle_controller(quad, attitude_command(1), sim.dt());

    // Dynamics function that accepts bidy thrust, torque commands
    quad.dynamics_direct_thrust_torque(body_thrust_command, body_torque_command,
                                       sim.dt());

    // Simulate using explicit Euler integration
    quad.euler_step(sim.dt());

    // Plot variables for debugging
    //////////////////////////////////////////////////////////////////////////////////

    std::cout << "Euler angle:" << quad.frame.euler_orientation()(0) << '\t'
              << quad.frame.euler_orientation()(1) << '\t'
              << quad.frame.euler_orientation()(2) << '\n';

    std::cout << '\n';
    //////////////////////////////////////////////////////////////////////////////////

    if (plot_flags::plot_enable) {
      // Set variables for plotting
      plot_var::x[i] = quad.position()(0);
      plot_var::y[i] = quad.position()(1);
      plot_var::z[i] = quad.position()(2);

      plot_var::roll_angle[i] = quad.frame.euler_orientation()(0);
      plot_var::pitch_angle[i] = quad.frame.euler_orientation()(1);
      plot_var::yaw_angle[i] = quad.frame.euler_orientation()(2);

      plot_var::x_setpoint[i] = x_target;
      plot_var::y_setpoint[i] = y_target;
      plot_var::z_setpoint[i] = z_target;

      plot_var::roll_angle_setpoint[i] = attitude_command(0);

      plot_var::thrust[i] = body_thrust_command(2);

      plot_var::t[i] = i * sim.dt();
    }

    if (fastdds_flag) {
      // Publish mocap msg
      mocap_quadcopter msg;

      msg.index({(uint32_t)i + 1});
      msg.position({quad.position()(0) * 1000, quad.position()(1) * 1000,
                    quad.position()(2) * 1000});

      msg.orientation_quaternion({quad.orientation()(1), quad.orientation()(2),
                                  quad.orientation()(3),
                                  quad.orientation()(0)});

      pose_pub.run(msg);
      // std::this_thread::sleep_for(std::chrono::milliseconds(sim.sim_time()));
    }
  }

  if (plot_flags::plot_enable) {
    // Initialize visualizer
    MyApp app;
    app.run();
  }

  return 0;
}
