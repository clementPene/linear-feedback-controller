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

  void initialize(const RobotModelBuilder::SharedPtr& rmb);

  /// @brief Enable a per-joint 2nd-order Butterworth low-pass on the feedback
  /// torque u_fb = K*(x - x_ref). @param cutoff_hz 0 disables it (default).
  /// @param sample_rate_hz the rate compute_control() is called at.
  void configure_feedback_lowpass(double cutoff_hz, double sample_rate_hz);

  const Eigen::VectorXd& compute_control(
      const linear_feedback_controller_msgs::Eigen::Sensor& sensor_msg,
      const linear_feedback_controller_msgs::Eigen::Control& control_msg);

 private:
  Eigen::VectorXd desired_configuration_;
  Eigen::VectorXd desired_velocity_;
  Eigen::VectorXd measured_configuration_;
  Eigen::VectorXd measured_velocity_;

  /**
   *  @brief Difference between the measured and the desired state,
   *  \$f x = [ q^T, \dot{q}^T ]^T \$f
   *
   */
  Eigen::VectorXd diff_state_;

  Eigen::VectorXd control_;
  RobotModelBuilder::SharedPtr rmb_;

  /// @name Feedback-torque low-pass (see configure_feedback_lowpass)
  /// 2nd-order Butterworth, direct-form-II transposed, per joint.
  /// @{
  bool fb_lp_enabled_ = false;
  double fb_lp_b0_ = 1.0, fb_lp_b1_ = 0.0, fb_lp_b2_ = 0.0;
  double fb_lp_a1_ = 0.0, fb_lp_a2_ = 0.0;
  Eigen::VectorXd fb_lp_x1_, fb_lp_x2_, fb_lp_y1_, fb_lp_y2_;
  bool fb_lp_primed_ = false;  ///< seed the filter state on the first sample
  Eigen::VectorXd u_fb_;       ///< scratch: K*(x - x_ref) before filtering
  Eigen::VectorXd fb_lp_out_;  ///< scratch: filtered feedback torque
  /// @}
};

}  // namespace linear_feedback_controller

#endif  // LINEAR_FEEDBACK_CONTROLLER_LFCONTROLLER_HPP
