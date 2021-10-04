#include "motor_mixing.h"
#include "pid_cascaded.h"
#include "plot.h"
#include "quadcopter.h"
#include "simulator.h"
#include <iostream>

int main() {

  Quadcopter quad;
  Simulator sim;
  PidCascadedController controller;

  sim.set_parameters("quadcopter_sim/quadcopter_sim_app/parameters/"
                     "simulation_parameters.yaml");

  quad.set_parameters();
  quad.set_initial_conditions();

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

    // Outer loop
    body_thrust_command(2) = controller.altitude_controller(quad, 5, sim.dt());

    attitude_command(0) = controller.horizontal_controller(quad, 0, sim.dt());

    body_torque_command(0) =
        controller.roll_angle_controller(quad, attitude_command(0), sim.dt());

    // // Convert thrust, torque to motor speeds
    // motor_mixing(quad, motor_commands, body_thrust_command,
    // body_torque_command,
    //              quad.motor[0].k_f(), quad.frame.arm_length());

    // // Dynamics function that accepts motor commands i
    // quad.dynamics(motor_commands, sim.dt());

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
      plot_var::x_plot[i] = quad.position()(0);
      plot_var::y_plot[i] = quad.position()(1);
      plot_var::z_plot[i] = quad.position()(2);

      plot_var::roll_angle_plot[i] = quad.frame.euler_orientation()(0);
      plot_var::pitch_angle_plot[i] = quad.frame.euler_orientation()(1);
      plot_var::yaw_angle_plot[i] = quad.frame.euler_orientation()(2);

      plot_var::thrust_plot[i] = body_thrust_command(2);

      plot_var::roll_torque_plot[i] = body_torque_command(0);

      plot_var::t_plot[i] = i * sim.dt();
    }
  }

  if (plot_flags::plot_enable) {
    // Initialize visualizer
    MyApp app;
    app.run();
  }

  return 0;
}
