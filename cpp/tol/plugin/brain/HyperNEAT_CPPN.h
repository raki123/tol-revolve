#ifndef TOL_PLUGIN_HYPERNEAT_CPPN_H_
#define TOL_PLUGIN_HYPERNEAT_CPPN_H_

#include <string>
#include <vector>

#include <gazebo/gazebo.hh>

#include <revolve/msgs/neural_net.pb.h>
#include "revolve/gazebo/brain/Brain.h"

#include "Evaluator.h"
#include "brain/ConverterSplitBrain.h"
#include "brain/controller/ExtCPPNWeights.h"
#include "brain/learner/NEATLearner.h"
#include "brain/Types.h"

namespace tol {

class HyperNEAT_CPG
        : public revolve::gazebo::Brain
          , private revolve::brain::ConverterSplitBrain<revolve::brain::CPPNConfigPtr, CPPNEAT::GeneticEncodingPtr>
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
    HyperNEAT_CPG(std::string modelName,
               sdf::ElementPtr brain,
               tol::EvaluatorPtr evaluator,
               const std::vector<revolve::gazebo::MotorPtr> &actuators,
               const std::vector<revolve::gazebo::SensorPtr> &sensors);

    virtual ~HyperNEAT_CPG();

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

    static CPPNEAT::Learner::LearningConfiguration
    parseLearningSDF(sdf::ElementPtr brain);
};


} /* namespace tol */

#endif // TOL_PLUGIN_HYPERNEAT_CPPN_H_
