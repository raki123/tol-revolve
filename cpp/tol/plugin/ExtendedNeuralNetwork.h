#ifndef REVOLVE_GAZEBO_BRAIN_DIFFERENTIAL_NEAT_H
#define REVOLVE_GAZEBO_BRAIN_DIFFERENTIAL_NEAT_H

#include "brain/ccpg/ExtendedNeuralNetwork.h"
#include "evaluator.h"
#include "revolve/gazebo/brain/Brain.h"


#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <revolve/msgs/neural_net.pb.h>

namespace tol {

    class ExtendedNeuralNetwork : public revolve::gazebo::Brain, private revolve::brain::ExtendedNeuralNetwork {

    public:
        ExtendedNeuralNetwork(std::string modelName,
		tol::EvaluatorPtr evaluator,
                sdf::ElementPtr node,
                const std::vector<revolve::gazebo::MotorPtr> &actuators,
                const std::vector<revolve::gazebo::SensorPtr> &sensors);

        virtual ~ExtendedNeuralNetwork();

        /**
         * Method for updating sensors readings, actuators positions, ranked list of policies and generating new policy
         * @param actuators: vector list of robot's actuators
         * @param sensors: vector list of robot's sensors
         * @param t:
         * @param step:
         */
        virtual void update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                            const std::vector<revolve::gazebo::SensorPtr> &sensors,
                            double t,
                            double step);

        static ExtNNConfig parseSDF(sdf::ElementPtr node,
				      const std::vector<revolve::gazebo::MotorPtr > & motors,
				      const std::vector<revolve::gazebo::SensorPtr > & sensors);
	
	static std::map<std::string, double> parseSDFElement(sdf::ElementPtr elem);
	
	static void connectionHelper(const std::string &src,
			      const std::string &dst,
			      const std::string &socket,
			      double weight,
			      const std::map<std::string, revolve::brain::NeuronPtr> &idToNeuron, 
			      ExtNNConfig& ret);
	
	static revolve::brain::NeuronPtr neuronHelper(sdf::ElementPtr neuron,
			       ExtNNConfig& ret);
	
	static revolve::brain::NeuronPtr addNeuron(const std::string &neuronId,
					   const std::string &neuronType,
					   const std::string &neuronLayer, // can be 'hidden', 'input' or 'output'
					   const std::map<std::string, double> &params, 
					   ExtNNConfig& ret);
	

    };

} /* namespace tol */

#endif //REVOLVE_GAZEBO_BRAIN_REINFORCEDLEARNING_H


