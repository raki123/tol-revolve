#include "helper.h"
#include "basic_neat_brain.h"

#include <iostream>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

namespace tol {
    unsigned int getActSize(const std::vector <revolve::gazebo::MotorPtr> &actuators) {
	unsigned int ret = 0;
	for(auto it :actuators) {
	    ret += (*it).outputs();
	}
	return ret;
    }
    unsigned int getSenSize(const std::vector <revolve::gazebo::SensorPtr> &actuators) {
	unsigned int ret = 0;
	for(auto it :actuators) {
	    ret += (*it).inputs();
	}
	return ret;
    }
    BasicBrain::BasicBrain(tol::EvaluatorPtr evaluator,
			   sdf::ElementPtr node,
                         const std::vector <revolve::gazebo::MotorPtr> &actuators,
                         const std::vector <revolve::gazebo::SensorPtr> &sensors) :
                         revolve::brain::BasicBrain(evaluator, 
						    getActSize(actuators),
						    getSenSize(sensors),
						    parseSDF(node,actuators,sensors)) {
    }

    BasicBrain::~BasicBrain() {
    }



    void BasicBrain::update(const std::vector <revolve::gazebo::MotorPtr> &motors,
                           const std::vector <revolve::gazebo::SensorPtr> &sensors,
                           double t,
                           double step) {
        revolve::brain::BasicBrain::update(
                Helper::createWrapper(motors),
                Helper::createWrapper(sensors),
                t, step
        );
    }
NEAT::InnovGenome::GenomeConfig BasicBrain::parseSDF(sdf::ElementPtr node,
		       const std::vector< revolve::gazebo::MotorPtr > & motors,
		       const std::vector< revolve::gazebo::SensorPtr > & sensors) {
	NEAT::InnovGenome::GenomeConfig ret;
	ret.innov = 1;
	ret.node_id = 1;
	ret.trait_id = 1;
	// Map neuron sdf elements to their id's
	std::map<std::string, sdf::ElementPtr> neuronDescriptions;

	// List of all hidden neuron id's
	std::vector<std::string> hiddenIds;
	
	std::map<std::string, unsigned int> idToNeuron;

	// Get the first sdf neuron element
	auto neuron = node->HasElement("rv:neuron") ? node->GetElement("rv:neuron") : sdf::ElementPtr();

	while (neuron) {
		if (!neuron->HasAttribute("layer") || !neuron->HasAttribute("id")) {
			std::cerr << "Missing required neuron attributes (id or layer). '" << std::endl;
			throw std::runtime_error("Robot brain error");
		}
		auto layer = neuron->GetAttribute("layer")->GetAsString();
		auto neuronId = neuron->GetAttribute("id")->GetAsString();


		// check if a neuron with this id has been already added
		if (neuronDescriptions.count(neuronId)) {
			std::cerr << "Duplicate neuron ID '"
					<< neuronId << "'" << std::endl;
			throw std::runtime_error("Robot brain error");
		}

		// add this neuron to the id->sdf map
		neuronDescriptions[neuronId] = neuron;

		// add the neuron id to the appropriate list of id's
		if ("input" == layer) {
	//			inputIds.push_back(neuronId);
		}

		else if ("output" == layer) {
	//			outputIds.push_back(neuronId);
		}

		else if ("hidden" == layer) {
			hiddenIds.push_back(neuronId);
		}

		else {
			std::cerr << "Unknown neuron layer '" << layer << "'." << std::endl;
			throw std::runtime_error("Robot brain error");
		}

		neuron = neuron->GetNextElement("rv:neuron");
	}


	// Add output neurons for motors:

	// map of numbers of output neurons for each body part
	std::map<std::string, unsigned int> outputCountMap;

	for (auto it = motors.begin(); it != motors.end(); ++it) {
		auto motor = *it;
		auto partId = motor->partId();

		if (!outputCountMap.count(partId)) {
			outputCountMap[partId] = 0;
		}

		for (unsigned int i = 0, l = motor->outputs(); i < l; ++i) {
			std::stringstream neuronId;
			neuronId << partId << "-out-" << outputCountMap[partId];
			outputCountMap[partId]++;

			auto neuronDescription = neuronDescriptions.find(neuronId.str());
			if (neuronDescription == neuronDescriptions.end()) {
				std::cerr << "Required output neuron " << neuronId.str() <<
						" for motor could not be located"
						<< std::endl;
				throw std::runtime_error("Robot brain error");
			}
			
			auto newNeuron = neuronHelper(neuronDescription->second, ret);
			ret.nodes.push_back(newNeuron);
			idToNeuron[neuronId.str()] = newNeuron.node_id;
			
		}
	}
	// Add input neurons for sensors:

	// map of number of input neurons for each part:

	std::map<std::string, unsigned int> inputCountMap;

	for (auto it = sensors.begin(); it != sensors.end(); ++it) {
		auto sensor = *it;
		auto partId = sensor->partId();

		if (!inputCountMap.count(partId)) {
			inputCountMap[partId] = 0;
		}

		for (unsigned int i = 0, l = sensor->inputs(); i < l; ++i) {
			std::stringstream neuronId;
			neuronId << partId << "-in-" << inputCountMap[partId];
			inputCountMap[partId]++;

			auto neuronDescription = neuronDescriptions.find(neuronId.str());
			if (neuronDescription == neuronDescriptions.end()) {
				std::cerr << "Required input neuron " << neuronId.str() <<
						" for sensor could not be located"
						<< std::endl;
				throw std::runtime_error("Robot brain error");
			}

			auto newNeuron = neuronHelper(neuronDescription->second,ret);
			ret.nodes.push_back(newNeuron);
			idToNeuron[neuronId.str()] = newNeuron.node_id;
		}
	}



	// Add hidden neurons:
	for (auto it = hiddenIds.begin(); it != hiddenIds.end(); ++it) {
		auto neuronDescription = neuronDescriptions.find(*it);
		auto newNeuron = neuronHelper(neuronDescription->second,ret);
		ret.nodes.push_back(newNeuron);
		idToNeuron[(*it)] = newNeuron.node_id;
	}


	// Add connections:
	auto connection = node->HasElement("rv:neural_connection") ? node->GetElement("rv:neural_connection") : sdf::ElementPtr();
	while (connection) {
		if (!connection->HasAttribute("src") || !connection->HasAttribute("dst")
				|| !connection->HasAttribute("weight")) {
			std::cerr << "Missing required connection attributes (`src`, `dst` or `weight`)." << std::endl;
			throw std::runtime_error("Robot brain error");
		}

		auto src = connection->GetAttribute("src")->GetAsString();
		auto dst = connection->GetAttribute("dst")->GetAsString();
		
		auto srcNeuron = idToNeuron.find(src);
		if (srcNeuron == idToNeuron.end()) {
			std::cerr << "Could not find source neuron '" << src << "'" << std::endl;
			throw std::runtime_error("Robot brain error");
		}
		auto dstNeuron = idToNeuron.find(dst);
		if (dstNeuron == idToNeuron.end()) {
			std::cerr << "Could not find destination neuron '" << dst << "'" << std::endl;
			throw std::runtime_error("Robot brain error");
		}
		double weight;
		connection->GetAttribute("weight")->Get(weight);

		// Use connection helper to set the weight
		connectionHelper(srcNeuron->second, dstNeuron->second, weight, ret);

		// Load the next connection
		connection = connection->GetNextElement("rv:neural_connection");
	}
	return ret;
}

NEAT::Trait BasicBrain::parseSDFElement(sdf::ElementPtr elem)
{
	std::map<std::string, double> params;

	auto subElem = elem->GetFirstElement();
	while (subElem) {
		auto elName = subElem->GetName();
		double elValue = subElem->Get<double>();
		params[elName] = elValue;
		subElem = subElem->GetNextElement();
	}
	NEAT::Trait neuronParams;
	double min_value = -1.0, max_value = 1.0;
	neuronParams.params[0] = (params["rv:bias"]-min_value)/(max_value-min_value);
	min_value = 0.01, max_value = 5.0;
	neuronParams.params[1] = (params["rv:tau"]-min_value)/(max_value-min_value);
	min_value=0, max_value=1;
	neuronParams.params[2] = (params["rv:gain"]-min_value)/(max_value-min_value);
	min_value=0, max_value=10;
	neuronParams.params[3] = (params["rv:period"]-min_value)/(max_value-min_value);
	min_value=0, max_value=3.14;
	neuronParams.params[4] = (params["rv:phase_offset"]-min_value)/(max_value-min_value);
	min_value=0, max_value=10000;
	neuronParams.params[5] = (params["rv:amplitude"]-min_value)/(max_value-min_value);
	min_value = 0.05, max_value = 10.0;
	neuronParams.params[6] = (params["rv:alpha"]-min_value)/(max_value-min_value);
	min_value = 0.0, max_value = 25.0;
	neuronParams.params[7] = (params["rv:energy"]-min_value)/(max_value-min_value);
	return neuronParams;
}

void BasicBrain::connectionHelper( unsigned int src,
					     unsigned int dst,
					     double weight,
					     NEAT::InnovGenome::GenomeConfig &ret)
{

	NEAT::InnovLinkGene newConnection(1,
                                       weight,
                                       src,
                                       dst,
                                       false,
                                       ret.innov++,
                                       0.0);
	ret.links.push_back(newConnection);
}


NEAT::InnovNodeGene BasicBrain::neuronHelper(sdf::ElementPtr neuron,
		       NEAT::InnovGenome::GenomeConfig &ret)
{
	if (!neuron->HasAttribute("type")) {
		std::cerr << "Missing required `type` attribute for neuron." << std::endl;
		throw std::runtime_error("Robot brain error");
	}

	if (!neuron->HasAttribute("layer")) {
		std::cerr << "Missing required `layer` attribute for neuron." << std::endl;
		throw std::runtime_error("Robot brain error");
	}

	auto type = neuron->GetAttribute("type")->GetAsString();
	auto layer = neuron->GetAttribute("layer")->GetAsString();

	// map <std::string, double> of parameter names and values
	auto params = parseSDFElement(neuron);

	return addNeuron(type, layer, params, ret);
}



NEAT::InnovNodeGene BasicBrain::addNeuron(const std::string &neuronType,
					   const std::string &neuronLayer, // can be 'hidden', 'input' or 'output'
					   NEAT::Trait &params, 
					   NEAT::InnovGenome::GenomeConfig &ret)
{
	NEAT::InnovNodeGene newNeuron;
// 	std::cout << neuronType + " " + neuronId  + " was added in"+ " "+ neuronLayer << std::endl;
	if ("input" == neuronLayer) {
		newNeuron = NEAT::InnovNodeGene(NEAT::NT_SENSOR, ret.node_id++, NEAT::INPUT);
	}

	else if ("output" == neuronLayer) {

		if ("Sigmoid" == neuronType) {
			newNeuron = NEAT::InnovNodeGene(NEAT::NT_OUTPUT, ret.node_id++, NEAT::SIGMOID);
		}
		else if ("Simple" == neuronType) {
			newNeuron = NEAT::InnovNodeGene(NEAT::NT_OUTPUT, ret.node_id++, NEAT::SIMPLE);
		}
// 		else if ("Oscillator" == neuronType) {
// 			newNeuron.reset(new revolve::brain::OscillatorNeuron(neuronId, params));
// 		}
// 		else if ("V-Neuron" == neuronType) {
// 			newNeuron.reset(new revolve::brain::VOscillator(neuronId, params));
// 		}
// 		else if ("X-Neuron" == neuronType) {
// 			newNeuron.reset(new revolve::brain::XOscillator(neuronId, params));
// 		}
		else if ("Bias" == neuronType) {
			newNeuron = NEAT::InnovNodeGene(NEAT::NT_OUTPUT, ret.node_id++, NEAT::BIAS);
		}
// 		else if ("Leaky" == neuronType) {
// 			newNeuron.reset(new revolve::brain::LeakyIntegrator(neuronId, params));
// 		}
		else if ("DifferentialCPG" == neuronType) {
			newNeuron = NEAT::InnovNodeGene(NEAT::NT_OUTPUT, ret.node_id++, NEAT::DIFFERENTIAL_CPG);
		}
		else {
			std::cerr << "Unsupported neuron type `" << neuronType << '`' << std::endl;
			throw std::runtime_error("Robot brain error");
		}
	} else {
	  	if ("Sigmoid" == neuronType) {
			newNeuron = NEAT::InnovNodeGene(NEAT::NT_HIDDEN, ret.node_id++, NEAT::SIGMOID);
		}
		else if ("Simple" == neuronType) {
			newNeuron = NEAT::InnovNodeGene(NEAT::NT_HIDDEN, ret.node_id++, NEAT::SIMPLE);
		}
// 		else if ("Oscillator" == neuronType) {
// 			newNeuron.reset(new revolve::brain::OscillatorNeuron(neuronId, params));
// 		}
// 		else if ("V-Neuron" == neuronType) {
// 			newNeuron.reset(new revolve::brain::VOscillator(neuronId, params));
// 		}
// 		else if ("X-Neuron" == neuronType) {
// 			newNeuron.reset(new revolve::brain::XOscillator(neuronId, params));
// 		}
		else if ("Bias" == neuronType) {
			newNeuron = NEAT::InnovNodeGene(NEAT::NT_HIDDEN, ret.node_id++, NEAT::BIAS);
		}
// 		else if ("Leaky" == neuronType) {
// 			newNeuron.reset(new revolve::brain::LeakyIntegrator(neuronId, params));
// 		}
		else if ("DifferentialCPG" == neuronType) {
			newNeuron = NEAT::InnovNodeGene(NEAT::NT_HIDDEN, ret.node_id++, NEAT::DIFFERENTIAL_CPG);
		}
		else {
			std::cerr << "Unsupported neuron type `" << neuronType << '`' << std::endl;
			throw std::runtime_error("Robot brain error");
		}	
	}
	newNeuron.set_trait_id(ret.trait_id++);
	ret.traits.push_back(params);
	ret.nodes.push_back(newNeuron);
	return newNeuron;
}
} /* namespace tol */

