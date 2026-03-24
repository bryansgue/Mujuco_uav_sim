#include "AcroMode.h"

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstring>

namespace MujocoRosUtils
{

// ---------- Helpers ----------
static inline double parse_double(const char * s, double fallback)
{
  if(!s || *s == '\0') return fallback;
  char * end = nullptr;
  const double v = std::strtod(s, &end);
  return (end == s) ? fallback : v;
}

static inline int find_actuator(const mjModel * m, const char * name)
{
  if(!name || std::strlen(name) == 0) return -1;
  return mj_name2id(m, mjOBJ_ACTUATOR, name);
}

// ---------- Registration ----------
void AcroMode::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);
  plugin.name = "MujocoRosUtils::AcroMode";
  plugin.capabilityflags |= mjPLUGIN_ACTUATOR;
  plugin.needstage = mjSTAGE_VEL;

  // NOTE: new attributes added for motor model configuration
  static const char * attributes[] = {
      "topic_name",  "actuator_fz",  "actuator_tx",  "actuator_ty",
      "actuator_tz", "control_hz",   "body_name",
      "motor_model", "arm_length",   "kf",           "km",
      "motor_tau",   "motor_omega_max", "drag_coeff"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, int) { return 0; };
  plugin.nsensordata = +[](const mjModel *, int, int) { return 0; };

  plugin.init = +[](const mjModel * m, mjData * d, int id)
  {
    auto * inst = AcroMode::Create(m, d, id);
    if(!inst) return -1;
    d->plugin_data[id] = reinterpret_cast<uintptr_t>(inst);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int id)
  {
    delete reinterpret_cast<AcroMode *>(d->plugin_data[id]);
    d->plugin_data[id] = 0;
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int id, int)
  { reinterpret_cast<AcroMode *>(d->plugin_data[id])->compute(m, d, id); };

  mjp_registerPlugin(&plugin);
}

// ---------- Factory ----------
AcroMode * AcroMode::Create(const mjModel * m, mjData * d, int plugin_id)
{
  int act_id = -1;
  for(int i = 0; i < m->nu; ++i)
  {
    if(m->actuator_plugin[i] == plugin_id)
    {
      act_id = i;
      break;
    }
  }

  std::string j_name = (act_id != -1) ? mj_id2name(m, mjOBJ_JOINT, m->actuator_trnid[act_id * 2]) : "unnamed";

  const char * t_cfg = mj_getPluginConfig(m, plugin_id, "topic_name");
  std::string topic = (t_cfg && strlen(t_cfg) > 0) ? t_cfg : "trpy_cmd";

  int id_fz = find_actuator(m, mj_getPluginConfig(m, plugin_id, "actuator_fz"));
  int id_tx = find_actuator(m, mj_getPluginConfig(m, plugin_id, "actuator_tx"));
  int id_ty = find_actuator(m, mj_getPluginConfig(m, plugin_id, "actuator_ty"));
  int id_tz = find_actuator(m, mj_getPluginConfig(m, plugin_id, "actuator_tz"));

  const char * b_cfg = mj_getPluginConfig(m, plugin_id, "body_name");
  int b_id = b_cfg ? mj_name2id(m, mjOBJ_BODY, b_cfg) : 0;
  std::string b_name = b_cfg ? b_cfg : mj_id2name(m, mjOBJ_BODY, b_id);

  double hz = parse_double(mj_getPluginConfig(m, plugin_id, "control_hz"), 500.0);

  // Motor model parameters (read from XML config, with sensible defaults)
  const char * mm_cfg = mj_getPluginConfig(m, plugin_id, "motor_model");
  bool motor_model = (mm_cfg && (std::string(mm_cfg) == "true" || std::string(mm_cfg) == "1"));

  double arm_length     = parse_double(mj_getPluginConfig(m, plugin_id, "arm_length"),      0.1);
  double kf             = parse_double(mj_getPluginConfig(m, plugin_id, "kf"),               1.91e-6);
  double km             = parse_double(mj_getPluginConfig(m, plugin_id, "km"),               2.6e-8);
  double motor_tau      = parse_double(mj_getPluginConfig(m, plugin_id, "motor_tau"),        0.02);
  double motor_omega_max = parse_double(mj_getPluginConfig(m, plugin_id, "motor_omega_max"), 2500.0);
  double drag_coeff     = parse_double(mj_getPluginConfig(m, plugin_id, "drag_coeff"),       0.0);

  return new AcroMode(m, d, id_fz, id_tx, id_ty, id_tz, topic, hz, b_id, b_name,
                      motor_model, arm_length, kf, km, motor_tau, motor_omega_max, drag_coeff);
}

// ---------- Constructor ----------
AcroMode::AcroMode(const mjModel * m,
                   mjData * d,
                   int id_fz,
                   int id_tx,
                   int id_ty,
                   int id_tz,
                   std::string topic,
                   double hz,
                   int bid,
                   std::string bname,
                   bool motor_model_enabled,
                   double arm_length,
                   double kf,
                   double km,
                   double motor_tau,
                   double motor_omega_max,
                   double drag_coeff)
: id_fz_(id_fz), id_tx_(id_tx), id_ty_(id_ty), id_tz_(id_tz), body_id_(bid), body_name_(bname),
  motor_model_enabled_(motor_model_enabled),
  arm_length_(arm_length), kf_(kf), km_(km),
  motor_tau_(motor_tau), motor_omega_max_(motor_omega_max), drag_coeff_(drag_coeff)
{
  ctrl_dt_ = 1.0 / hz;

  // ── Build the mixer matrix (X-configuration, Betaflight convention) ──
  //
  //  Motor layout (top view):
  //
  //       Front
  //    M1(+x,+y)  M2(+x,-y)
  //         \      /
  //          [BODY]
  //         /      \
  //    M4(-x,+y)  M3(-x,-y)
  //
  //  Betaflight motor numbering & spin:
  //    M1 = front-right  CW    (+x, -y) → negative yaw torque
  //    M2 = rear-right   CCW   (-x, -y) → positive yaw torque
  //    M3 = rear-left    CW    (-x, +y) → negative yaw torque
  //    M4 = front-left   CCW   (+x, +y) → positive yaw torque
  //
  //  Wrench from 4 motors:
  //    F_total = kf * (w1^2 + w2^2 + w3^2 + w4^2)
  //    tau_x   = kf * L/sqrt(2) * (-w1^2 - w2^2 + w3^2 + w4^2)  (X-config)
  //    tau_y   = kf * L/sqrt(2) * ( w1^2 - w2^2 - w3^2 + w4^2)
  //    tau_z   = km * (-w1^2 + w2^2 - w3^2 + w4^2)
  //
  //  In matrix form:  wrench = A * [w1^2, w2^2, w3^2, w4^2]^T
  //  We store A_inv to go from desired wrench → desired w_i^2.

  const double L = arm_length_;
  const double Ls = L / std::sqrt(2.0);   // effective arm for X-config

  Eigen::Matrix4d A;
  //              M1     M2     M3     M4
  A <<           kf_,    kf_,    kf_,    kf_,       // F_total
        -kf_*Ls, -kf_*Ls,  kf_*Ls,  kf_*Ls,       // tau_x (roll)
         kf_*Ls, -kf_*Ls, -kf_*Ls,  kf_*Ls,       // tau_y (pitch)
        -km_,     km_,    -km_,     km_;            // tau_z (yaw)

  mixer_inv_ = A.inverse();

  if(motor_model_enabled_)
  {
    // Log motor model parameters
    printf("[AcroMode] Motor model ENABLED for body '%s'\n", bname.c_str());
    printf("  arm_length=%.4f  kf=%.3e  km=%.3e\n", arm_length_, kf_, km_);
    printf("  motor_tau=%.4f  motor_omega_max=%.1f  drag_coeff=%.4f\n",
           motor_tau_, motor_omega_max_, drag_coeff_);
  }

  std::string ns = body_name_;
  if(ns.empty()) ns = "/";
  if(ns.front() != '/') ns = "/" + ns;

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=" + ns});

  if(!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  nh_ = rclcpp::Node::make_shared("acro_ctrl", options);

  // Standard command subscription
  std::string quadrotor_topic = "trpy_cmd";
  sub_trpy_cmd_ = nh_->create_subscription<quadrotor_msgs::msg::TRPYCommand>(
      quadrotor_topic, 10, std::bind(&AcroMode::trpy_callback, this, std::placeholders::_1));

  // Payload command subscription
  std::string payload_topic = "trpy_payload_cmd";
  sub_trpy_payload_cmd_ = nh_->create_subscription<quadrotor_msgs::msg::TRPYCommand>(
      payload_topic, 10, std::bind(&AcroMode::trpy_payload_callback, this, std::placeholders::_1));

  // Activation Service
  std::string service_name = "activate_payload_nmpc_controller";
  srv_activate_payload_ = nh_->create_service<std_srvs::srv::SetBool>(
      service_name,
      std::bind(&AcroMode::activate_payload_callback, this, std::placeholders::_1, std::placeholders::_2));

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(nh_);
}

// ---------- Callbacks ----------

void AcroMode::activate_payload_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                         std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  use_nmpc_payload_ = request->data;
  response->success = true;
  response->message = use_nmpc_payload_ ? "Payload NMPC active" : "Standard TRPY active";
  RCLCPP_INFO(nh_->get_logger(), "Switching controller mode: %s", response->message.c_str());
}

void AcroMode::trpy_callback(const quadrotor_msgs::msg::TRPYCommand::SharedPtr msg)
{
  if(!use_nmpc_payload_)
  {
    commanded_thrust_ = msg->thrust;
    wd_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "Using NMPC QUADROTOR");
  }
}

void AcroMode::trpy_payload_callback(const quadrotor_msgs::msg::TRPYCommand::SharedPtr msg)
{
  if(use_nmpc_payload_)
  {
    commanded_thrust_ = msg->thrust;
    wd_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "Using NMPC PAYLOAD");
  }
}

// ---------- Physics Compute ----------

void AcroMode::compute(const mjModel * m, mjData * d, int)
{
  if(executor_) executor_->spin_once(std::chrono::seconds(0));

  const double t = d->time;
  const double sim_dt = m->opt.timestep;

  while(t + 1e-12 >= next_ctrl_time_)
  {
    // ── Step 1: Read current body state from MuJoCo ──
    mjtNum vel[6];
    mj_objectVelocity(m, d, mjOBJ_XBODY, body_id_, vel, 0);

    Eigen::Vector4d q_curr;
    q_curr << d->xquat[4 * body_id_], d->xquat[4 * body_id_ + 1],
              d->xquat[4 * body_id_ + 2], d->xquat[4 * body_id_ + 3];
    Eigen::Matrix3d R = quatToRot(q_curr);

    Eigen::Vector3d w_world(vel[0], vel[1], vel[2]);
    Eigen::Vector3d w_body = R.transpose() * w_world;

    // ── Step 2: Rate controller (same as before) ──
    //   Computes desired wrench from angular velocity error
    Eigen::Vector3d e_omega = w_body - wd_;
    Eigen::Vector3d Iw = J_ * w_body;
    Eigen::Vector3d gyroscopic = w_body.cross(Iw);
    Eigen::Vector3d M_des = gyroscopic - J_ * (kom_ * e_omega);
    double F_des = commanded_thrust_;

    if(!motor_model_enabled_)
    {
      // ── Direct mode (original behavior): write wrench to MuJoCo ──
      if(id_fz_ != -1) d->ctrl[id_fz_] = F_des;
      if(id_tx_ != -1) d->ctrl[id_tx_] = M_des(0);
      if(id_ty_ != -1) d->ctrl[id_ty_] = M_des(1);
      if(id_tz_ != -1) d->ctrl[id_tz_] = M_des(2);
    }
    else
    {
      // ── Motor model mode: realistic motor dynamics ──

      // Step 3a: Desired wrench → desired motor speeds (via mixer)
      Eigen::Vector4d wrench_des(F_des, M_des(0), M_des(1), M_des(2));
      Eigen::Vector4d omega_sq_des = mixer_inv_ * wrench_des;

      // Clamp to valid range and take sqrt to get desired ωᵢ
      Eigen::Vector4d omega_des;
      const double omega_sq_max = motor_omega_max_ * motor_omega_max_;
      for(int i = 0; i < 4; ++i)
      {
        double clamped = std::max(0.0, std::min(omega_sq_des(i), omega_sq_max));
        omega_des(i) = std::sqrt(clamped);
      }

      // Step 3b: First-order motor dynamics
      //   ω̇ᵢ = (ωᵢ_des - ωᵢ) / τ_motor
      const double alpha = sim_dt / motor_tau_;  // discrete approximation
      for(int i = 0; i < 4; ++i)
      {
        motor_omega_(i) += alpha * (omega_des(i) - motor_omega_(i));
        motor_omega_(i) = std::max(0.0, std::min(motor_omega_(i), motor_omega_max_));
      }

      // Step 3c: Motor speeds → actual forces/torques
      Eigen::Vector4d omega_sq_actual;
      for(int i = 0; i < 4; ++i)
        omega_sq_actual(i) = motor_omega_(i) * motor_omega_(i);

      double F_actual  = kf_ * omega_sq_actual.sum();
      const double Ls = arm_length_ / std::sqrt(2.0);
      double tx_actual = kf_ * Ls * (-omega_sq_actual(0) - omega_sq_actual(1)
                                      + omega_sq_actual(2) + omega_sq_actual(3));
      double ty_actual = kf_ * Ls * ( omega_sq_actual(0) - omega_sq_actual(1)
                                      - omega_sq_actual(2) + omega_sq_actual(3));
      double tz_actual = km_ * (-omega_sq_actual(0) + omega_sq_actual(1)
                                - omega_sq_actual(2) + omega_sq_actual(3));

      // Step 3d: Aerodynamic drag (body-frame linear drag)
      if(drag_coeff_ > 0.0)
      {
        Eigen::Vector3d v_world(vel[3], vel[4], vel[5]);
        Eigen::Vector3d v_body = R.transpose() * v_world;
        Eigen::Vector3d F_drag = -drag_coeff_ * v_body;

        // Add drag to the control: only affects Fz in body frame
        // For a full model, tx/ty also get drag torques, but the primary
        // effect is the translational drag opposing body-Z thrust and body-XY.
        // We apply the Z-component as thrust reduction and X/Y as torques
        // via cross-product with moment arm (≈ 0 for CoG, so just direct force).
        // Simplified: add drag force components to the corresponding actuators.
        F_actual += F_drag(2);  // Z-body drag reduces effective thrust
        tx_actual += F_drag(1) * 0.01;  // small coupling (negligible at low speed)
        ty_actual += F_drag(0) * 0.01;
      }

      // Write to MuJoCo actuators
      if(id_fz_ != -1) d->ctrl[id_fz_] = std::max(0.0, F_actual);
      if(id_tx_ != -1) d->ctrl[id_tx_] = tx_actual;
      if(id_ty_ != -1) d->ctrl[id_ty_] = ty_actual;
      if(id_tz_ != -1) d->ctrl[id_tz_] = tz_actual;
    }

    next_ctrl_time_ += ctrl_dt_;
  }
}

void AcroMode::reset(const mjModel *, int)
{
  next_ctrl_time_ = 0.0;
  commanded_thrust_ = 0.0;
  wd_ = Eigen::Vector3d::Zero();
  motor_omega_ = Eigen::Vector4d::Zero();
}

Eigen::Matrix3d AcroMode::quatToRot(const Eigen::Vector4d & q)
{
  Eigen::Quaterniond eq(q(0), q(1), q(2), q(3));
  return eq.toRotationMatrix();
}

} // namespace MujocoRosUtils
