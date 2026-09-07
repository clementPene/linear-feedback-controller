#include "linear_feedback_controller/lf_controller.hpp"

#include <cmath>

#include "pinocchio/algorithm/joint-configuration.hpp"

namespace linear_feedback_controller {

namespace {
// Looks up a Contact by name. Returns nullptr if absent (e.g. the topic
// hasn't published yet, or this cycle's message doesn't carry it).
const linear_feedback_controller_msgs::Eigen::Contact* find_contact(
    const std::vector<linear_feedback_controller_msgs::Eigen::Contact>&
        contacts,
    const std::string& name) {
  for (const auto& contact : contacts) {
    if (contact.name == name) {
      return &contact;
    }
  }
  return nullptr;
}
}  // namespace

LFController::LFController() {}

LFController::~LFController() {}

void LFController::initialize(const RobotModelBuilder::SharedPtr& rmb,
                             int n_force_dirs) {
  if (!rmb) {
    throw std::invalid_argument("RobotModelBuilder pointer cannot be null.");
  }
  if (n_force_dirs < 0) {
    throw std::invalid_argument("n_force_dirs must be >= 0.");
  }
  rmb_ = rmb;
  n_force_dirs_ = n_force_dirs;
  const auto nq = rmb_->get_nq();
  const auto nv = rmb_->get_nv();
  const auto joint_nv = rmb_->get_joint_nv();

  desired_configuration_ = Eigen::VectorXd::Zero(nq);
  desired_velocity_ = Eigen::VectorXd::Zero(nv);
  measured_configuration_ = Eigen::VectorXd::Zero(nq);
  measured_velocity_ = Eigen::VectorXd::Zero(nv);
  control_ = Eigen::VectorXd::Zero(joint_nv);
  // [ dq(nv); dv(nv); df(n_force_dirs_) ]
  diff_state_ = Eigen::VectorXd::Zero(2 * nv + n_force_dirs_);

  u_fb_ = Eigen::VectorXd::Zero(joint_nv);
  u_fb_raw_ = Eigen::VectorXd::Zero(joint_nv);
  fb_lp_out_ = Eigen::VectorXd::Zero(joint_nv);
  fb_lp_x1_ = Eigen::VectorXd::Zero(joint_nv);
  fb_lp_x2_ = Eigen::VectorXd::Zero(joint_nv);
  fb_lp_y1_ = Eigen::VectorXd::Zero(joint_nv);
  fb_lp_y2_ = Eigen::VectorXd::Zero(joint_nv);
  fb_lp_primed_ = false;
}

void LFController::configure_feedback_lowpass(double cutoff_hz,
                                              double sample_rate_hz) {
  if (cutoff_hz <= 0.0 || sample_rate_hz <= 0.0 ||
      cutoff_hz >= 0.5 * sample_rate_hz) {
    fb_lp_enabled_ = false;
    return;
  }
  // 2nd-order Butterworth low-pass, bilinear transform with pre-warping.
  constexpr double kPi = 3.14159265358979323846;
  const double q = 1.0 / std::sqrt(2.0);
  const double k = std::tan(kPi * cutoff_hz / sample_rate_hz);
  const double norm = 1.0 / (1.0 + k / q + k * k);
  fb_lp_b0_ = k * k * norm;
  fb_lp_b1_ = 2.0 * fb_lp_b0_;
  fb_lp_b2_ = fb_lp_b0_;
  fb_lp_a1_ = 2.0 * (k * k - 1.0) * norm;
  fb_lp_a2_ = (1.0 - k / q + k * k) * norm;
  fb_lp_enabled_ = true;
  fb_lp_primed_ = false;
}

void LFController::configure_contact_force_feedback(
    const std::string& contact_name, const std::vector<int>& wrench_indices,
    double activation_time_constant, double sample_rate_hz) {
  force_contact_name_ = contact_name;
  force_wrench_indices_ = wrench_indices;
  if (activation_time_constant <= 0.0 || sample_rate_hz <= 0.0) {
    force_blend_alpha_ = 1.0;  // hard on/off
  } else {
    force_blend_alpha_ =
        1.0 - std::exp(-1.0 / (activation_time_constant * sample_rate_hz));
  }
}

const Eigen::VectorXd& LFController::compute_control(
    const linear_feedback_controller_msgs::Eigen::Sensor& sensor_msg,
    const linear_feedback_controller_msgs::Eigen::Control& control_msg,
    const linear_feedback_controller_msgs::Eigen::Sensor&
        measured_force_sensor_msg) {
  if (!rmb_) {
    throw std::runtime_error(
        "LFController is not initialized. Call initialize() before "
        "compute_control().");
  }
  const linear_feedback_controller_msgs::Eigen::Sensor& ctrl_init =
      control_msg.initial_state;

  // Reconstruct the state vector: x = [q, v]. The desired state is the OCP knot
  // (possibly interpolated across the cycle by the ROS layer for delay comp).
  rmb_->construct_robot_state(ctrl_init, desired_configuration_,
                              desired_velocity_);
  rmb_->construct_robot_state(sensor_msg, measured_configuration_,
                              measured_velocity_);

  // diff_state = [ q_des (-) q_meas ; v_des - v_meas ; df(n_force_dirs_) ].
  const auto nv = rmb_->get_model().nv;
  pinocchio::difference(rmb_->get_model(), measured_configuration_,
                        desired_configuration_, diff_state_.head(nv));
  diff_state_.segment(nv, nv) = desired_velocity_ - measured_velocity_;

  // Contact-force error tail: df = blend * (f0 - f_meas), gated on the
  // measured contact's .active by a first-order smoothed blend (see
  // configure_contact_force_feedback). f0 comes from this cycle's control
  // (the OCP's own setpoint); f_meas from the live force-sensor reading.
  if (n_force_dirs_ > 0) {
    const auto* f0_contact =
        find_contact(ctrl_init.contacts, force_contact_name_);
    const auto* meas_contact =
        find_contact(measured_force_sensor_msg.contacts, force_contact_name_);
    const bool active = meas_contact != nullptr && meas_contact->active;
    force_blend_ +=
        force_blend_alpha_ * ((active ? 1.0 : 0.0) - force_blend_);
    if (f0_contact != nullptr && meas_contact != nullptr &&
        static_cast<int>(force_wrench_indices_.size()) == n_force_dirs_) {
      for (int i = 0; i < n_force_dirs_; ++i) {
        diff_state_(2 * nv + i) =
            force_blend_ * (f0_contact->wrench(force_wrench_indices_[i]) -
                            meas_contact->wrench(force_wrench_indices_[i]));
      }
    } else {
      diff_state_.tail(n_force_dirs_).setZero();
    }
  }

  // Feedback torque, optionally low-pass filtered (see
  // configure_feedback_lowpass).
  u_fb_.noalias() = control_msg.feedback_gain * diff_state_;
  u_fb_raw_ = u_fb_;
  if (fb_lp_enabled_) {
    if (!fb_lp_primed_) {
      fb_lp_x1_ = fb_lp_x2_ = fb_lp_y1_ = fb_lp_y2_ = u_fb_;
      fb_lp_primed_ = true;
    }
    // y[n] = b0 x[n] + b1 x[n-1] + b2 x[n-2] - a1 y[n-1] - a2 y[n-2]
    fb_lp_out_.noalias() = fb_lp_b0_ * u_fb_ + fb_lp_b1_ * fb_lp_x1_ +
                           fb_lp_b2_ * fb_lp_x2_ - fb_lp_a1_ * fb_lp_y1_ -
                           fb_lp_a2_ * fb_lp_y2_;
    fb_lp_x2_ = fb_lp_x1_;
    fb_lp_x1_ = u_fb_;
    fb_lp_y2_ = fb_lp_y1_;
    fb_lp_y1_ = fb_lp_out_;
    u_fb_ = fb_lp_out_;
  }

  // Optional attenuation of the fast-loop gain (see set_feedback_gain_scale).
  // u_fb_raw_ above stays the unscaled K*(x - x_ref).
  if (fb_scale_ != 1.0) u_fb_ *= fb_scale_;

  control_.noalias() = control_msg.feedforward + u_fb_;
  return control_;
}

}  // namespace linear_feedback_controller
