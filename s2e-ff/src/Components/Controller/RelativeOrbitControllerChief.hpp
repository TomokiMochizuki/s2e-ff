#ifndef RELATIVE_ORBIT_CONTROLLER_H_
#define RELATIVE_ORBIT_CONTROLLER_H_

#include <Component/Abstract/ComponentBase.h>
#include <Interface/LogOutput/ILoggable.h>

#include "../../Library/RelativeOrbit/QuasiNonsingularRelativeOrbitalElements.hpp"
#include "../../Simulation/Spacecraft/FfComponents.hpp"

class FfComponents;

class RelativeOrbitControllerChief : public ComponentBase, public ILoggable {
 public:
  RelativeOrbitControllerChief(const int prescaler, ClockGenerator* clock_gen, FfComponents& components);
  ~RelativeOrbitControllerChief();
  // ComponentBase
  void MainRoutine(int count) override;

  // ILoggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 protected:
  FfComponents& components_;

  // Internal variables
  double a_m_ = 6928000.0;
  double mass_kg_ = 40.0;
  double impulse_output_duration_sec_ = 10.0;
  double component_update_sec_ = 0.1;

  QuasiNonsingularRelativeOrbitalElements target_qns_roe_;     //!< Target QNS Relative OE with semi-major axis
  QuasiNonsingularRelativeOrbitalElements estimated_qns_roe_;  //!< Estimated QNS Relative OE

  double estimated_relative_distance_m_ = 0.0;  //!< Estimated relative distance
  double estimated_inc_vec_angle_ = 0.0;        //!< Estimated angle of inclination vector

  libra::Vector<3> dv_rtn_m_s_{0.0};
  double dv_timing_rad_ = 0.0;
  bool first_thrust_done_ = false;
  bool second_thrust_done_ = false;

  // Constants, parameters
  double mu_m3_s2_;

  // Functions
  void EstimateStates();
  libra::Vector<3> InPlaneSingleImpulse(double& maneuver_timing_rad, const QuasiNonsingularRelativeOrbitalElements diff_qns_roe);
  libra::Vector<3> OutPlaneSingleImpulse(double& maneuver_timing_rad, const QuasiNonsingularRelativeOrbitalElements diff_qns_roe);

  libra::Vector<3> DoubleImpulse_seirios(double& first_maneuver_s, double& delta_maneuver_s, QuasiNonsingularRelativeOrbitalElements diff_qns_roe);
};

#endif
