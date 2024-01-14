/**
 * @file relative_position_attitude_observer.cpp
 * @brief Relative Position and Attitude Observer
 */

#include "./relative_position_attitude_observer.hpp"

RelativePositionAttitudeObserver::RelativePositionAttitudeObserver(const int prescaler, ClockGenerator* clock_gen,
                                                                   std::vector<LaserDistanceMeter*>& laser_distance_meters,
                                                                   RelativePositionSensor& relative_position_sensor,
                                                                   RelativeAttitudeSensor& relative_attitude_sensor,
                                                                   FfInterSpacecraftCommunication& inter_spacecraft_communication)
    : Component(prescaler, clock_gen),
      laser_distance_meters_(laser_distance_meters),
      relative_position_sensor_(relative_position_sensor),
      relative_attitude_sensor_(relative_attitude_sensor),
      inter_spacecraft_communication_(inter_spacecraft_communication) {}

void RelativePositionAttitudeObserver::MainRoutine(int count) {
  UNUSED(count);
  ObserveRelativePositionAttitude();
}

std::string RelativePositionAttitudeObserver::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "relative_position_attitude_observer_";
  str_tmp += WriteVector(head + "position", "tb2rb", "m", 3);
  str_tmp += WriteVector(head + "euler_angle", "tb2rb", "rad", 3);
  return str_tmp;
}

std::string RelativePositionAttitudeObserver::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteVector(observed_relative_position_m_);
  str_tmp += WriteVector(observed_relative_euler_angle_rad_);
  return str_tmp;
}

// Functions
void RelativePositionAttitudeObserver::ObserveRelativePositionAttitude() {
  libra::Vector<kLaserDistanceMetersNumber> line_of_sight_distance_m{0.0};

  bool is_laser_received_by_laser_distance_meter = true;
  for (size_t laser_id = 0; laser_id < kLaserDistanceMetersNumber; ++laser_id) {
    is_laser_received_by_laser_distance_meter = laser_distance_meters_[laser_id]->GetIsReflected();
    if (!is_laser_received_by_laser_distance_meter) break;
    line_of_sight_distance_m[laser_id] = -laser_distance_meters_[laser_id]->GetObservedDistance_m();
  }

  if (!is_laser_received_by_laser_distance_meter) {
    observed_relative_position_m_[0] = relative_position_sensor_.GetMeasuredTargetPosition_b_m()[0];
    observed_relative_euler_angle_rad_[1] = relative_attitude_sensor_.GetMeasuredEulerAngle_rb2tb_rad()[1];
    observed_relative_euler_angle_rad_[2] = relative_attitude_sensor_.GetMeasuredEulerAngle_rb2tb_rad()[2];
  } else {
    observed_relative_position_m_[0] = (line_of_sight_distance_m[0] + line_of_sight_distance_m[2]) / 2.0;
    observed_relative_euler_angle_rad_[1] = (line_of_sight_distance_m[1] - line_of_sight_distance_m[2]) / 2.0 / component_position_z_axis_m_;
    observed_relative_euler_angle_rad_[2] = (line_of_sight_distance_m[0] - line_of_sight_distance_m[1]) / 2.0 / component_position_y_axis_m_;
  }

  inter_spacecraft_communication_.SetLineOfSightDistance_m(observed_relative_position_m_[0]);
  std::vector<bool> is_laser_received_by_qpd_sensors = inter_spacecraft_communication_.GetIsLaserReceivedByQpdSensor();

  for (size_t qpd_id = 0; qpd_id < is_laser_received_by_qpd_sensors.size(); ++qpd_id) {
    if (!is_laser_received_by_qpd_sensors[qpd_id]) {
      observed_relative_position_m_[1] = relative_position_sensor_.GetMeasuredTargetPosition_b_m()[1];
      observed_relative_position_m_[2] = relative_position_sensor_.GetMeasuredTargetPosition_b_m()[2];
      observed_relative_euler_angle_rad_[0] = relative_attitude_sensor_.GetMeasuredEulerAngle_rb2tb_rad()[0];
      return;
    }
  }

  libra::Vector<kQpdPositioningSensorsNumber> displacement_y_axis_m = -inter_spacecraft_communication_.GetYAxisDisplacementCalcedByQpdSensor_m();
  libra::Vector<kQpdPositioningSensorsNumber> displacement_z_axis_m = -inter_spacecraft_communication_.GetZAxisDisplacementCalcedByQpdSensor_m();

  observed_relative_position_m_[1] = (displacement_y_axis_m[0] + displacement_y_axis_m[1]) / 2.0;
  observed_relative_position_m_[2] = (displacement_z_axis_m[0] + displacement_z_axis_m[1]) / 2.0;
  observed_relative_euler_angle_rad_[0] = (displacement_y_axis_m[1] - displacement_y_axis_m[0]) / 2.0 / component_position_z_axis_m_;
}
