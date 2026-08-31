#ifndef LINEAR_FEEDBACK_CONTROLLER_LFCONTROLLER_HPP
#define LINEAR_FEEDBACK_CONTROLLER_LFCONTROLLER_HPP

#include "Eigen/Core"
#include "linear_feedback_controller/robot_model_builder.hpp"
#include "linear_feedback_controller/visibility.hpp"
#include "linear_feedback_controller_msgs/eigen_conversions.hpp"

namespace linear_feedback_controller {

class LINEAR_FEEDBACK_CONTROLLER_PUBLIC LFController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  static constexpr int kNbFreeFlyerDof = 6;

  LFController();
  virtual ~LFController();

  /// @param n_force_dirs number of contact-force directions appended to the
  /// feedback state: diff_state_ becomes [ dq; dv; df ] of size
  /// 2*nv + n_force_dirs, and compute_control() expects a feedback_gain with
  /// that many columns. 0 keeps the plain [ dq; dv ] behaviour.
  void initialize(const RobotModelBuilder::SharedPtr& rmb,
                  int n_force_dirs = 0);

  const Eigen::VectorXd& compute_control(
      const linear_feedback_controller_msgs::Eigen::Sensor& sensor_msg,
      const linear_feedback_controller_msgs::Eigen::Control& control_msg);

 private:
  Eigen::VectorXd desired_configuration_;
  Eigen::VectorXd desired_velocity_;
  Eigen::VectorXd measured_configuration_;
  Eigen::VectorXd measured_velocity_;

  /**
   *  @brief Difference between the desired and the measured state,
   *  [ q_des \ominus q_meas ; v_des - v_meas ] (size 2*nv), optionally
   *  augmented with the contact-force error [ ... ; f0 - f_meas ] when
   *  n_force_dirs_ > 0.
   */
  Eigen::VectorXd diff_state_;

  /// @brief Number of contact-force directions in diff_state_'s tail.
  int n_force_dirs_ = 0;

  Eigen::VectorXd control_;
  RobotModelBuilder::SharedPtr rmb_;
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_LFCONTROLLER_HPP
