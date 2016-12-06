#ifndef GAZEBO_NEAT_BRAIN_H
#define GAZEBO_NEAT_BRAIN_H
#include "brain/basic_neat_brain.h"
#include "evaluator.h"
#include "neat/accneat/src/innovgenome/innovgenome.h"

#include "revolve/gazebo/brain/Brain.h"
#include <gazebo/gazebo.hh>
#include <revolve/msgs/neural_net.pb.h>


namespace tol {

    class BasicBrain : public revolve::gazebo::Brain ,private revolve::brain::BasicBrain {
    public:

        BasicBrain(tol::EvaluatorPtr evaluator,
		   sdf::ElementPtr node,
		    const std::vector <revolve::gazebo::MotorPtr> &actuators,
		    const std::vector <revolve::gazebo::SensorPtr> &sensors);

        virtual ~BasicBrain();

        /**
         * @param Motor list
         * @param Sensor list
         */
        virtual void update(const std::vector <revolve::gazebo::MotorPtr> &motors,
                            const std::vector <revolve::gazebo::SensorPtr> &sensors,
                            double t,
                            double step);
	
	static NEAT::InnovGenome::GenomeConfig parseSDF(sdf::ElementPtr node,
				    const std::vector<revolve::gazebo::MotorPtr > & motors,
				    const std::vector<revolve::gazebo::SensorPtr > & sensors);
	
	/**
	 * Method to get the parameters of neurons from an sdf ElementPtr
	 * @param elem: sdf that includes the parameters
	 * @return string to parameter mapping
	 */
	static NEAT::Trait parseSDFElement(sdf::ElementPtr elem);
	
	/** 
	* Helpermethod for getting the configuration
	* adds a new connection to the configuration
	* @param src: the beginning of the connetion
	* @param dst: the end of the connection
	* @param weight: weighting factor of the connection
	* @param idToNeuron: mapping between the neurons and their ids
	* @param ret: configuration for the network where the connection should be included
	*/
	static void connectionHelper(unsigned int src,
			      unsigned int dst,
			      double weight,
			      NEAT::InnovGenome::GenomeConfig& ret);
	/** 
	* Helpermethod for getting the configuration 
	* @param neuron: sdf for the neuron to be added
	* @param ret: configuration for the network where the neuron should be added
	*/
	static NEAT::InnovNodeGene neuronHelper(sdf::ElementPtr neuron, 
						      NEAT::InnovGenome::GenomeConfig& ret);
	
	/**
	* Helpermethod for getting the configuration 
	* This function creates neurons and adds them to the configuration
	* @param neuronId: id of the neuron
	* @param neuronType: type of the neuron
	* @param neuronLayer: layer the neuron should be added to, can be 'hidden', 'input' or 'output'
	* @param params: parameters of the new neuron
	*/
	static NEAT::InnovNodeGene addNeuron(const std::string &neuronType,
					   const std::string &neuronLayer, 
					   NEAT::Trait &params, 
					   NEAT::InnovGenome::GenomeConfig& ret);
    };

} /* namespace tol */

#endif // GAZEBO_NEAT_BRAIN_H
