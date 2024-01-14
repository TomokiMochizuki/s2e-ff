/**
 * @file relative_position_attitude_controller.hpp
 * @brief Relative Position and Attitude Controller
 */

#ifndef S2E_COMPONENTS_RELATIVE_POSITION_ATTITUDE_CONTROLLER_HPP_
#define S2E_COMPONENTS_RELATIVE_POSITION_ATTITUDE_CONTROLLER_HPP_

#include <components/base/component.hpp>
#include <components/ideal/force_generator.hpp>
#include <library/logger/logger.hpp>
#include <library/math/vector.hpp>

#include "../observer/relative_position_attitude_observer.hpp"

/**
 * @class RelativePositionAttitudeController
 * @brief Relative Position and Attitude Controller
 */
class RelativePositionAttitudeController : public Component, public ILoggable {
 public:
  /**
   * @fn RelativePositionAttitudeController
   * @brief Constructor
   */
  RelativePositionAttitudeController(const int prescaler, ClockGenerator* clock_gen, ForceGenerator* force_generator,
                                     RelativePositionAttitudeObserver* relative_position_attitude_observer);
  /**
   * @fn ~RelativePositionAttitudeController
   * @brief Destructor
   */
  ~RelativePositionAttitudeController() {}

  // ComponentBase override function
  /**
   * @fn MainRoutine
   * @brief Main routine
   */
  void MainRoutine(int count) override;

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const;

 protected:
  ForceGenerator* force_generator_;
  RelativePositionAttitudeObserver* relative_position_attitude_observer_;

  double component_update_sec_ = 0.1;
  double mass_kg_ = 14;

  libra::Vector<3> target_relatative_position_m_{0.0};

  libra::Vector<3> estimated_state_position_m_{0.0};
  libra::Vector<3> estimated_state_velocity_m_s_{0.0};
  libra::Vector<3> estimated_state_position_dot_m_s_{0.0};
  libra::Vector<3> estimated_state_velocity_dot_m_s2_{0.0};

  libra::Vector<2> observer_gain_{0.0};
  libra::Vector<2> feedback_gain_{0.0};
  // Funbstions
  void Initialize();
  void EstimateStates();
};

#endif  // S2E_COMPONENTS_RELATIVE_POSITION_ATTITUDE_CONTROLLER_HPP_
