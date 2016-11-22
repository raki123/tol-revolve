#ifndef REVOLVE_GAZEBO_BRAIN_DIFFERENTIAL_SPLIT_BRAIN_H_
#define REVOLVE_GAZEBO_BRAIN_DIFFERENTIAL_SPLIT_BRAIN_H_

#include "brain/split_cpg/simple_split_brain.h"
#include "brain/split_cpg/extended_neural_network_controller.h"
#include "brain/split_cpg/weight_vector_learner.h"
#include "evaluator.h"
#include "revolve/gazebo/brain/Brain.h"


#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <revolve/msgs/neural_net.pb.h>

namespace tol {

    class ExtNN : public revolve::gazebo::Brain, private revolve::brain::SimpleSplitBrain<std::vector<double>> {

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
        ExtNN(std::string modelName,
		tol::EvaluatorPtr evaluator,
                sdf::ElementPtr node,
                const std::vector<revolve::gazebo::MotorPtr> &actuators,
                const std::vector<revolve::gazebo::SensorPtr> &sensors);

        virtual ~ExtNN();

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

	/**
	 * Method to transform the sdf file to a configuration
	 * @param node: sdf file to be transformed
         * @param motors: vector list of robot's motors
         * @param sensors: vector list of robot's sensors
	 * @return configuration needed for the neural network
	 */
        static revolve::brain::ExtNNController::ExtNNConfig parseSDF(sdf::ElementPtr node,
				    const std::vector<revolve::gazebo::MotorPtr > & motors,
				    const std::vector<revolve::gazebo::SensorPtr > & sensors);
	
	/**
	 * Method to get the parameters of neurons from an sdf ElementPtr
	 * @param elem: sdf that includes the parameters
	 * @return string to parameter mapping
	 */
	static std::map<std::string, double> parseSDFElement(sdf::ElementPtr elem);
	
	/** 
	* Helpermethod for getting the configuration
	* adds a new connection to the configuration
	* @param src: the beginning of the connetion
	* @param dst: the end of the connection
	* @param weight: weighting factor of the connection
	* @param idToNeuron: mapping between the neurons and their ids
	* @param ret: configuration for the network where the connection should be included
	*/
	static void connectionHelper(const std::string &src,
			      const std::string &dst,
			      const std::string &socket,
			      double weight,
			      const std::map<std::string, revolve::brain::NeuronPtr> &idToNeuron, 
			      revolve::brain::ExtNNController::ExtNNConfig &ret);
	/** 
	* Helpermethod for getting the configuration 
	* @param neuron: sdf for the neuron to be added
	* @param ret: configuration for the network where the neuron should be added
	*/
	static revolve::brain::NeuronPtr neuronHelper(sdf::ElementPtr neuron, 
						      revolve::brain::ExtNNController::ExtNNConfig &ret);
	
	/**
	* Helpermethod for getting the configuration 
	* This function creates neurons and adds them to the configuration
	* @param neuronId: id of the neuron
	* @param neuronType: type of the neuron
	* @param neuronLayer: layer the neuron should be added to, can be 'hidden', 'input' or 'output'
	* @param params: parameters of the new neuron
	*/
	static revolve::brain::NeuronPtr addNeuron(const std::string &neuronId,
					   const std::string &neuronType,
					   const std::string &neuronLayer, 
					   const std::map<std::string, double> &params, 
					   revolve::brain::ExtNNController::ExtNNConfig &ret);
	
    };


} /* namespace tol */

#endif //REVOLVE_GAZEBO_BRARN_DIFFERENTIAL_SPLIT_BRAIN_H_


