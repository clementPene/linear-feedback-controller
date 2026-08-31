#include "linear_feedback_controller/lf_controller.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"

namespace linear_feedback_controller {

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
}

const Eigen::VectorXd& LFController::compute_control(
    const linear_feedback_controller_msgs::Eigen::Sensor& sensor_msg,
    const linear_feedback_controller_msgs::Eigen::Control& control_msg) {
  if (!rmb_) {
    throw std::runtime_error(
        "LFController is not initialized. Call initialize() before "
        "compute_control().");
  }
  // Shortcuts for easier code writing.
  const linear_feedback_controller_msgs::Eigen::JointState& sensor_js =
      sensor_msg.joint_state;
  const linear_feedback_controller_msgs::Eigen::JointState& ctrl_js =
      control_msg.initial_state.joint_state;
  const linear_feedback_controller_msgs::Eigen::Sensor& ctrl_init =
      control_msg.initial_state;

  // Reconstruct the state vector: x = [q, v]
  rmb_->construct_robot_state(ctrl_init, desired_configuration_,
                              desired_velocity_);
  rmb_->construct_robot_state(sensor_msg, measured_configuration_,
                              measured_velocity_);

  // Compute the linear feedback controller desired torque.
  const auto nv = rmb_->get_model().nv;
  pinocchio::difference(rmb_->get_model(), measured_configuration_,
                        desired_configuration_, diff_state_.head(nv));
  diff_state_.segment(nv, nv) = desired_velocity_ - measured_velocity_;
  // diff_state_.tail(n_force_dirs_) — the contact-force error — is filled by a
  // later step; it stays zero here, so the augmented feedback_gain columns
  // contribute nothing until then.
  control_.noalias() =
      control_msg.feedforward + control_msg.feedback_gain * diff_state_;

  return control_;
}

}  // namespace linear_feedback_controller
