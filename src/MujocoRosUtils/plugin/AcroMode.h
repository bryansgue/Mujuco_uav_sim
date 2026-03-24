#pragma once

#include <quadrotor_msgs/msg/trpy_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

namespace MujocoRosUtils
{

class AcroMode
{
public:
  static void RegisterPlugin();
  static AcroMode * Create(const mjModel * m, mjData * d, int plugin_id);

public:
  AcroMode(AcroMode &&) = default;
  AcroMode(const AcroMode &) = delete;
  AcroMode & operator=(const AcroMode &) = delete;

  void reset(const mjModel * m, int plugin_id);
  void compute(const mjModel * m, mjData * d, int plugin_id);

protected:
  AcroMode(const mjModel * m,
           mjData * d,
           int id_fz,
           int id_tx,
           int id_ty,
           int id_tz,
           std::string topic_name,
           double control_hz,
           int body_id,
           std::string body_name,
           bool motor_model_enabled,
           double arm_length,
           double kf,
           double km,
           double motor_tau,
           double motor_omega_max,
           double drag_coeff);

  // Callbacks
  void trpy_callback(const quadrotor_msgs::msg::TRPYCommand::SharedPtr msg);
  void trpy_payload_callback(const quadrotor_msgs::msg::TRPYCommand::SharedPtr msg);

  // Service Callback
  void activate_payload_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                 std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  static Eigen::Matrix3d quatToRot(const Eigen::Vector4d & q_wxyz);

protected:
  // Actuator and Body IDs
  int id_fz_ = -1, id_tx_ = -1, id_ty_ = -1, id_tz_ = -1;
  int body_id_ = -1;
  std::string body_name_;

  // Controller State
  bool use_nmpc_payload_ = false;

  // Command Variables
  double commanded_thrust_ = 0.0;
  Eigen::Vector3d wd_ = Eigen::Vector3d::Zero(); // Desired angular velocity

  // Gains and Inertia
  double kw_x_{20.0};
  double kw_y_{35.0};
  double kw_z_{45.0};
  Eigen::Matrix3d kom_ = Eigen::Vector3d(kw_x_, kw_y_, kw_z_).asDiagonal();
  Eigen::Matrix3d J_ = Eigen::Vector3d(0.00345398, 0.00179687, 0.00179676).asDiagonal();

  // ── Motor model parameters ──
  bool motor_model_enabled_ = false;

  // Geometry: arm length from center to each motor [m]
  double arm_length_ = 0.1;

  // Propeller coefficients:  F_i = kf * omega_i^2,  tau_i = km * omega_i^2
  double kf_ = 1.91e-6;    // thrust coefficient [N/(rad/s)^2]
  double km_ = 2.6e-8;     // torque coefficient [N·m/(rad/s)^2]

  // Motor first-order dynamics time constant [s]
  //   omega_dot = (omega_des - omega) / tau_motor
  double motor_tau_ = 0.02;

  // Maximum motor angular velocity [rad/s]
  double motor_omega_max_ = 2500.0;

  // Aerodynamic drag coefficient (body-frame linear drag) [N/(m/s)]
  double drag_coeff_ = 0.1;

  // Motor states: current angular velocities [rad/s]
  Eigen::Vector4d motor_omega_ = Eigen::Vector4d::Zero();

  // Mixer: allocation matrix  [F, τx, τy, τz] → [ω1², ω2², ω3², ω4²]
  // and its inverse for converting desired wrench → desired motor speeds
  Eigen::Matrix4d mixer_inv_;  // wrench → omega_i^2

  // Simulation Timing
  double ctrl_dt_ = 0.0;
  double next_ctrl_time_ = 0.0;

  // ROS 2
  rclcpp::Node::SharedPtr nh_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Subscription<quadrotor_msgs::msg::TRPYCommand>::SharedPtr sub_trpy_cmd_;
  rclcpp::Subscription<quadrotor_msgs::msg::TRPYCommand>::SharedPtr sub_trpy_payload_cmd_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_activate_payload_;
};

} // namespace MujocoRosUtils
