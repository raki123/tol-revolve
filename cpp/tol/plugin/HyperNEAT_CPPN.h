#ifndef REVOLVE_GAZEBO_BRAIN_HYPER_NEAT_SPLIT_BRAIN_H_
#define REVOLVE_GAZEBO_BRAIN_HYPER_NEAT_SPLIT_BRAIN_H_

#include "brain/ConvertingSplitBrain.h"
#include "brain/controller/ExtendedANNWeights.h"
#include "brain/learner/NEATLearner.h"
#include "Evaluator.h"
#include "revolve/gazebo/brain/Brain.h"


#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <revolve/msgs/neural_net.pb.h>

namespace tol
{

class HyperExtNN
        : public revolve::gazebo::Brain
          , private revolve::brain::ConvSplitBrain<boost::shared_ptr<revolve::brain::ExtNNConfig>, CPPNEAT::GeneticEncodingPtr>
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
    HyperExtNN(std::string modelName,
               sdf::ElementPtr brain,
               tol::EvaluatorPtr evaluator,
               const std::vector<revolve::gazebo::MotorPtr> &actuators,
               const std::vector<revolve::gazebo::SensorPtr> &sensors);

    virtual ~HyperExtNN();

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

#endif //REVOLVE_GAZEBO_BRAIN_HYPER_NEAT_SPLIT_BRAIN_H_


