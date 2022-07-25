#include "ForceGenerator.hpp"

#include <cfloat>

// Constructor
ForceGenerator::ForceGenerator(const int prescaler, ClockGenerator* clock_gen, const double magnitude_error_standard_deviation_N,
                               const double direction_error_standard_deviation_rad, const Dynamics* dynamics)
    : ComponentBase(prescaler, clock_gen),
      magnitude_noise_(0.0, magnitude_error_standard_deviation_N),
      direction_noise_(0.0, direction_error_standard_deviation_rad),
      dynamics_(dynamics) {}

ForceGenerator::~ForceGenerator() {}

void ForceGenerator::MainRoutine(int count) {
  UNUSED(count);

  generated_force_b_N_ = ordered_force_b_N_;
  // TODO: Add noise

  libra::Quaternion q_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
  libra::Quaternion q_i2rtn = dynamics_->GetOrbit().CalcQuaternionI2LVLH();
  generated_force_i_N_ = q_i2b.frame_conv_inv(generated_force_b_N_);
  generated_force_rtn_N_ = q_i2rtn.frame_conv(generated_force_i_N_);
}

void ForceGenerator::PowerOffRoutine() {
  generated_force_b_N_ *= 0.0;
  generated_force_i_N_ *= 0.0;
  generated_force_rtn_N_ *= 0.0;
}

void ForceGenerator::SetForce_i_N(const libra::Vector<3> force_i_N) {
  libra::Quaternion q_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
  ordered_force_b_N_ = q_i2b.frame_conv(force_i_N);
}

void ForceGenerator::SetForce_rtn_N(const libra::Vector<3> force_rtn_N) {
  libra::Quaternion q_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
  libra::Quaternion q_i2rtn = dynamics_->GetOrbit().CalcQuaternionI2LVLH();

  libra::Vector<3> force_i_N = q_i2rtn.frame_conv_inv(force_rtn_N);
  ordered_force_b_N_ = q_i2b.frame_conv(force_i_N);
}

std::string ForceGenerator::GetLogHeader() const {
  std::string str_tmp = "";

  std::string head = "IdealForceGenerator";
  str_tmp += WriteVector(head + "generated_force", "b", "N", 3);

  return str_tmp;
}

std::string ForceGenerator::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(generated_force_b_N_);

  return str_tmp;
}

libra::Quaternion GenerateDirectionNoiseQuaternion(libra::Vector<3> true_direction, const double error_standard_deviation_rad) {
  libra::NormalRand normal_rand;

  libra::Vector<3> random_direction;
  random_direction[0] = normal_rand;
  random_direction[1] = normal_rand;
  random_direction[2] = normal_rand;
  random_direction = normalize(random_direction);

  libra::Vector<3> rotation_axis;
  rotation_axis = outer_product(true_direction, random_direction);
  double norm_rotation_axis = norm(rotation_axis);
  if (norm_rotation_axis < 0.0 + DBL_EPSILON) {
    // TODO: add error handling
  }

  double error_angle_rad = normal_rand * error_standard_deviation_rad;
  libra::Quaternion error_quaternion(rotation_axis, error_angle_rad);
  return error_quaternion;
}
