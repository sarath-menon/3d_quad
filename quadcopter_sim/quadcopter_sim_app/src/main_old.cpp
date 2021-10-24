#include "include_helper.h"

int main() {

  Quadcopter quad;
  Simulator sim;
  PidCascadedController controller;

  sim.set_parameters("quadcopter_sim/quadcopter_sim_app/parameters/"
                     "simulation_parameters.yaml");

  quad.set_parameters();
  quad.set_initial_conditions();

  // Create participant. Argument-> Domain id, QOS name
  DefaultParticipant dp(0, "quad_simulator_2d_qos");

  // Create mocap data publisher
  DDSPublisher mocap_pub(idl_msg::MocapPubSubType(), "mocap_pose",
                         dp.participant());
  mocap_pub.init();

  // Give time to match pub,sub. Important! Do not delete
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // To publish pose msg or not
  if (sim.pose_pub_flag()) {
    mocap_pub.init();
  }

  // To be moved to external controller file
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
    quad.dynamics_direct_thrust_torque(body_thrust_command,
                                       body_torque_command);

    std::cout << "Position:" << quad.position()(0) << '\t' << quad.position()(1)
              << '\t' << quad.position()(2) << '\n';

    // Simulate using explicit Euler integration
    quad.euler_step(sim.dt());

    // Not working !!!!!   (Using simulator module)
    //////////////////////////////////////////////////////////////////////////////////
    // // Simulate one timestep
    // sim.simulate_step(quad.position(), quad.velocity(), quad.acceleration(),
    //                   quad.orientation(), quad.angular_velocity(),
    //                   quad.angular_acceleration());

    // quad.set_state(sim.position(), sim.velocity(), sim.orientation(),
    //                sim.angular_velocity());

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

    if (sim.pose_pub_flag()) {
      // Publish mocap msg
      // Construct mocap message
      cpp_msg::Mocap mocap_msg;
      mocap_msg.header.id = "srl_quad_sim";
      mocap_msg.header.timestamp = i + 1;

      mocap_msg.pose.position.x = quad.position()(0);
      mocap_msg.pose.position.y = quad.position()(1);
      mocap_msg.pose.position.z = quad.position()(2);

      mocap_msg.pose.orientation_quat.w = quad.orientation()(0);
      mocap_msg.pose.orientation_quat.x = quad.orientation()(1);
      mocap_msg.pose.orientation_quat.y = quad.orientation()(2);
      mocap_msg.pose.orientation_quat.z = quad.orientation()(3);

      // mocap_msg.pose.orientation_euler.roll = quad.true_beta();
      // mocap_msg.pose.orientation_euler.pitch = 0;
      // mocap_msg.pose.orientation_euler.yaw = 0;

      // Send mocap message
      mocap_pub.publish(mocap_msg);

      // Insert delay for real time visualization
      std::this_thread::sleep_for(std::chrono::milliseconds(sim.sim_time()));
    }
  }

  if (plot_flags::plot_enable) {
    // Initialize visualizer
    MyApp app;
    app.run();
  }

  return 0;
}
