//
// Created by Milan Jelisavcic on 28/03/16.
//

#ifndef TOL_PLUGIN_RLPOWER_SPLINES_H_
#define TOL_PLUGIN_RLPOWER_SPLINES_H_

#include <string>
#include <vector>

#include <gazebo/gazebo.hh>

#include <revolve/msgs/neural_net.pb.h>

#include "revolve/gazebo/brain/Brain.h"

#include "Evaluator.h"
#include "brain/SimpleSplitBrain.h"
#include "brain/controller/PolicyController.h"
#include "brain/learner/RLPowerLearner.h"

namespace tol {

class RLPower_Splines
        : public revolve::gazebo::Brain
        , private revolve::brain::ConverterSplitBrain<revolve::brain::PolicyPtr,
                                                      revolve::brain::PolicyPtr>
{
  public:

   /// \brief Constructor
   /// \param model_name: name of a robot
   /// \param brain: configuration file
   /// \param evaluator: pointer to fitness evaluatior
   /// \param n_actuators: number of actuators
   /// \param n_sensors: number of sensors
   /// \return pointer to the RLPower class object
  RLPower_Splines(std::string model_name,
                  sdf::ElementPtr brain,
                  tol::EvaluatorPtr evaluator,
                  std::vector<revolve::gazebo::MotorPtr> &actuators,
                  std::vector<revolve::gazebo::SensorPtr> &sensors);

  /// \brief Destructor
  virtual ~RLPower_Splines();

  using revolve::brain::ConverterSplitBrain<revolve::brain::PolicyPtr,
                                            revolve::brain::PolicyPtr>::update;
  /// \brief Update sensors reading, actuators position, and `brain` state
  /// \param[inout] actuators List of actuators
  /// \param[inout] sensors List of sensors
  /// \param[in] t Time value
  /// \param[in] step Time step
  virtual void update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                      const std::vector<revolve::gazebo::SensorPtr> &sensors,
                      double t,
                      double step);

  /// \brief
  static revolve::brain::RLPowerLearner::Config parseSDF(sdf::ElementPtr brain);
};

} /* namespace tol */

#endif // TOL_PLUGIN_RLPOWER_SPLINES_H_
