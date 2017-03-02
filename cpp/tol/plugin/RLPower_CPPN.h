//
// Created by Milan Jelisavcic on 28/03/16.
//

#ifndef TOL_PLUGIN_RLPOWER_CPPN_H_
#define TOL_PLUGIN_RLPOWER_CPPN_H_

#include <string>
#include <vector>

#include <gazebo/gazebo.hh>

#include <revolve/msgs/neural_net.pb.h>
#include "revolve/gazebo/brain/Brain.h"

#include "Evaluator.h"
#include "brain/ConvertingSplitBrain.h"
#include "brain/learner/RLPowerLearner.h"

namespace tol
{

class RLPowerNet
        : public revolve::gazebo::Brain
          , private revolve::brain::ConvSplitBrain<std::vector<double>, revolve::brain::PolicyPtr>
{

public:
    /**
     * The RLPower constructor reads out configuration file, deretmines which algorithm type to apply and
     * initialises new policy.
     * @param modelName: name of a robot
     * @param brain: configuration file
     * @param evaluator: pointer to fitness evaluatior
     * @param n_actuators: number of actuators
     * @param n_sensors: number of sensors
     * @return pointer to the RLPower class object
     */
    RLPowerNet(std::string modelName,
               sdf::ElementPtr brain,
               tol::EvaluatorPtr evaluator,
               std::vector<revolve::gazebo::MotorPtr> &actuators,
               std::vector<revolve::gazebo::SensorPtr> &sensors);

    virtual ~RLPowerNet();

    /**
     * Method for updating sensors readings, actuators positions, ranked list of policies and generating new policy
     * @param actuators: vector list of robot's actuators
     * @param sensors: vector list of robot's sensors
     * @param t:
     * @param step:
     */
    virtual void
    update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
           const std::vector<revolve::gazebo::SensorPtr> &sensors,
           double t,
           double step);

    static revolve::brain::RLPowerLearner::Config
    parseSDF(sdf::ElementPtr brain);
};

} /* namespace tol */

#endif // TOL_PLUGIN_RLPOWER_CPPN_H_

