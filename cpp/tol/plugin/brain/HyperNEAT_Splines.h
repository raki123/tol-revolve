#ifndef TOL_PLUGIN_HYPERNEAT_SPLINE_H_
#define TOL_PLUGIN_HYPERNEAT_SPLINE_H_

#include <string>
#include <vector>

#include <gazebo/gazebo.hh>

#include <revolve/msgs/neural_net.pb.h>
#include "revolve/gazebo/brain/Brain.h"

#include "Evaluator.h"
#include "brain/ConvertingSplitBrain.h"
#include "brain/controller/PolicyController.h"
#include "brain/learner/NEATLearner.h"
#include "brain/learner/RLPowerLearner.h"

namespace tol {

class HyperSplines
        : public revolve::gazebo::Brain
          , private revolve::brain::ConvSplitBrain<revolve::brain::PolicyPtr, CPPNEAT::GeneticEncodingPtr>
{

public:
    /**
* Constructor for a neural network including neurons that are of a different type than the usual ones.
* @param modelName: name of the model
* @param evaluator: pointer to the evaluator that is used
* @param node: the sdf file containing the necessary information to build the network
* @param actuators: vector list of robot's actuators
* @param sensors: vector list of robot's sensors
* @return pointer to the neural network
*/
    HyperSplines(std::string modelName,
                 sdf::ElementPtr brain,
                 tol::EvaluatorPtr evaluator,
                 const std::vector<revolve::gazebo::MotorPtr> &actuators,
                 const std::vector<revolve::gazebo::SensorPtr> &sensors);

    virtual ~HyperSplines();

    /**
     * Method for updating sensors readings, actuators positions, ranked list of policies and generating
     * new policy
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

    static CPPNEAT::Learner::LearningConfiguration
    parseLearningSDF(sdf::ElementPtr brain);
};


} /* namespace tol */

#endif // TOL_PLUGIN_HYPERNEAT_SPLINE_H_


