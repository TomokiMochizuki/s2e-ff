/**
 * @file relative_position_attitude_controller.cpp
 * @brief Relative Position and Attitude Controller
 */

#include "./relative_position_attitude_controller.hpp"

RelativePositionAttitudeController::RelativePositionAttitudeController(const int prescaler, ClockGenerator* clock_gen,
                                                                       ForceGenerator* force_generator, TorqueGenerator* torque_generator,
                                                                       RelativePositionAttitudeObserver* relative_position_attitude_observer)
    : Component(prescaler, clock_gen),
      force_generator_(force_generator),
      torque_generator_(torque_generator),
      relative_position_attitude_observer_(relative_position_attitude_observer) {
  Initialize();
}

void RelativePositionAttitudeController::MainRoutine(int count) { EstimateStates(count); }

std::string RelativePositionAttitudeController::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "relative_position_attitude_controller_";
  str_tmp += WriteVector(head + "estimated_position", "tb2rb", "m", 3);
  str_tmp += WriteVector(head + "estimated_velocity", "tb2rb", "m/s", 3);
  str_tmp += WriteVector(head + "estimated_eular_angle", "tb2rb", "rad", 3);
  str_tmp += WriteVector(head + "estimated_angular_velocity", "tb2rb", "rad/s", 3);
  return str_tmp;
}

std::string RelativePositionAttitudeController::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteVector(estimated_state_position_m_);
  str_tmp += WriteVector(estimated_state_velocity_m_s_);
  str_tmp += WriteVector(estimated_state_euler_angle_rad_);
  str_tmp += WriteVector(estimated_state_angular_velocity_rad_s_);
  return str_tmp;
}

// Functions
void RelativePositionAttitudeController::Initialize() {
  observer_gain_[0] = 1.0;
  observer_gain_[1] = 0.2;
  feedback_gain_[0] = 0.1;
  feedback_gain_[1] = 0.5;

  inertia_tensor_kgm2_[0][0] = 0.1;
  inertia_tensor_kgm2_[1][1] = 0.1;
  inertia_tensor_kgm2_[2][2] = 0.1;

  target_relatative_position_m_[0] = 9.0;
}

void RelativePositionAttitudeController::EstimateStates(int count) {
  libra::Vector<3> observed_relative_position_m = -relative_position_attitude_observer_->GetObservedRelativePosition_m();
  libra::Vector<3> observed_relative_euler_angle_rad = -relative_position_attitude_observer_->GetObservedRelativeEulerAngle_rad();
  libra::Vector<3> calculated_acceleration_m_s2{0.0};
  libra::Vector<3> calculated_acceleration_rad_s2{0.0};
  if (count > 500) {
    calculated_acceleration_m_s2 = -feedback_gain_[0] * estimated_state_position_m_ - feedback_gain_[1] * estimated_state_velocity_m_s_;

    libra::Vector<3> calculated_force_b_N = mass_kg_ * calculated_acceleration_m_s2;
    force_generator_->SetForce_b_N(calculated_force_b_N);

    // calculated_acceleration_rad_s2 =
    //     -feedback_gain_[0] * estimated_state_euler_angle_rad_ - feedback_gain_[1] * estimated_state_angular_velocity_rad_s_;

    // libra::Vector<3> calculated_torque_b_Nm = inertia_tensor_kgm2_ * calculated_acceleration_rad_s2;
    // torque_generator_->SetTorque_b_Nm(calculated_torque_b_Nm);
  }

  estimated_state_position_m_ += component_update_sec_ * estimated_state_position_dot_m_s_;
  estimated_state_velocity_m_s_ += component_update_sec_ * estimated_state_velocity_dot_m_s2_;

  estimated_state_position_dot_m_s_ =
      estimated_state_velocity_m_s_ +
      observer_gain_[0] * (observed_relative_position_m - target_relatative_position_m_ - estimated_state_position_m_);
  estimated_state_velocity_dot_m_s2_ =
      calculated_acceleration_m_s2 + observer_gain_[1] * (observed_relative_position_m - target_relatative_position_m_ - estimated_state_position_m_);

  estimated_state_euler_angle_rad_ += component_update_sec_ * estimated_state_euler_angle_dot_rad_s_;
  estimated_state_angular_velocity_rad_s_ += component_update_sec_ * estimated_state_angular_velocity_dot_rad_s2_;

  estimated_state_euler_angle_dot_rad_s_ =
      estimated_state_angular_velocity_rad_s_ + observer_gain_[0] * (observed_relative_euler_angle_rad - estimated_state_euler_angle_rad_);
  estimated_state_angular_velocity_dot_rad_s2_ =
      calculated_acceleration_rad_s2 + observer_gain_[1] * (observed_relative_euler_angle_rad - estimated_state_euler_angle_rad_);
}
