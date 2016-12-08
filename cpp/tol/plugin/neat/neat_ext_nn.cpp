#include "neat_ext_nn.h"
#include "brain/cppneat/neuron_gene.h"
#include "brain/cppneat/connection_gene.h"
#include "../helper.h"
#include "../sensor.h"
#include "../actuator.h"
#include "brain/split_cpg/conversion.h"
#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"


namespace tol {


    NeatExtNN::NeatExtNN(std::string modelName,
		     tol::EvaluatorPtr evaluator,
		     sdf::ElementPtr node,
                     const std::vector<revolve::gazebo::MotorPtr> &actuators,
                     const std::vector<revolve::gazebo::SensorPtr> &sensors) 
    :  revolve::brain::ConvSplitBrain<boost::shared_ptr<revolve::brain::ExtNNConfig>, CPPNEAT::GeneticEncodingPtr>(&revolve::brain::convertForController, &revolve::brain::convertForLearner) {
	
	int innov_number = 0;
	//sleep(20);
	revolve::brain::learning_configuration.start_from = parseSDF(node, actuators, sensors, innov_number);
	boost::shared_ptr<revolve::brain::ExtNNController1> swap1(new revolve::brain::ExtNNController1(modelName,
							revolve::brain::convertForController(revolve::brain::learning_configuration.start_from),  
							evaluator, 
							Helper::createWrapper(actuators),
							Helper::createWrapper(sensors)));
	controller = swap1;
	revolve::brain::set_learning_conf();
	revolve::brain::set_brain_spec();
	CPPNEAT::Mutator mutator(revolve::brain::brain_spec,
				1,
				0,
				100,
				std::vector<CPPNEAT::Neuron::Ntype>());
	boost::shared_ptr<CPPNEAT::Learner> swap2(new CPPNEAT::Learner(mutator, 
								       revolve::brain::learning_configuration));
	learner = boost::static_pointer_cast<revolve::brain::Learner<CPPNEAT::GeneticEncodingPtr>>(swap2);
	evaluator_ = evaluator;
    }

    NeatExtNN::~NeatExtNN()
    {

    }


    void NeatExtNN::update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                         const std::vector<revolve::gazebo::SensorPtr> &sensors,
                         double t,
                         double step) {
// 	std::cout << "yay" << std::endl;
        revolve::brain::ConvSplitBrain<boost::shared_ptr<revolve::brain::ExtNNConfig>, CPPNEAT::GeneticEncodingPtr>::update(
                Helper::createWrapper(actuators),
                Helper::createWrapper(sensors),
                t, step
        );
    }

    
CPPNEAT::GeneticEncodingPtr NeatExtNN::parseSDF(sdf::ElementPtr node,
		       const std::vector< revolve::gazebo::MotorPtr > & motors,
		       const std::vector< revolve::gazebo::SensorPtr > & sensors,
		       int &innov_number) 
{
	CPPNEAT::GeneticEncodingPtr ret(new CPPNEAT::GeneticEncoding());
	
	// Map neuron sdf elements to their id's
	std::map<std::string, sdf::ElementPtr> neuronDescriptions;

	// List of all hidden neuron id's
	std::vector<std::string> hiddenIds;

	std::map<std::string, CPPNEAT::NeuronGenePtr> idToNeuron_;

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
			
			auto newNeuron = neuronHelper(neuronDescription->second, ret, innov_number);
			idToNeuron_[neuronId.str()] = newNeuron;
			
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

			auto newNeuron = neuronHelper(neuronDescription->second,ret, innov_number);
			idToNeuron_[neuronId.str()] = newNeuron;
		}
	}



	// Add hidden neurons:
	for (auto it = hiddenIds.begin(); it != hiddenIds.end(); ++it) {
		auto neuronDescription = neuronDescriptions.find(*it);
		auto newNeuron = neuronHelper(neuronDescription->second,ret, innov_number);
		idToNeuron_[*it] = newNeuron;
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


		std::string dstSocketName;
		if (connection->HasAttribute("socket")) {
			dstSocketName = connection->GetAttribute("socket")->GetAsString();
		}
		else {
			dstSocketName = "None"; // this is the default socket name
		}

		double weight;
		connection->GetAttribute("weight")->Get(weight);

		// Use connection helper to set the weight
		connectionHelper(src, dst, dstSocketName, weight, idToNeuron_, ret, innov_number);

		// Load the next connection
		connection = connection->GetNextElement("rv:neural_connection");
	}
	return ret;
}

std::map<std::string, double> NeatExtNN::parseSDFElement(sdf::ElementPtr elem)
{
	std::map<std::string, double> params;

	auto subElem = elem->GetFirstElement();
	while (subElem) {
		auto elName = subElem->GetName();
		double elValue = subElem->Get<double>();
		params[elName] = elValue;
		subElem = subElem->GetNextElement();
	}

	return params;
}

void NeatExtNN::connectionHelper(const std::string &src,
					     const std::string &dst,
					     const std::string &socket,
					     double weight,
					     const std::map<std::string, CPPNEAT::NeuronGenePtr> &idToNeuron, 
					     CPPNEAT::GeneticEncodingPtr ret,
					     int &innov_number)
{
// 	std::cout << "connection from " + src + " to " + dst + " was added with weight: " << weight << std::endl;
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

	CPPNEAT::ConnectionGenePtr newConnection(new CPPNEAT::ConnectionGene(dstNeuron->second->getInnovNumber(),
										srcNeuron->second->getInnovNumber(),
										weight,
										innov_number++,
										true, 
										socket));
	ret->add_connection_gene(newConnection);
}


CPPNEAT::NeuronGenePtr NeatExtNN::neuronHelper(sdf::ElementPtr neuron,
						CPPNEAT::GeneticEncodingPtr ret,
						int &innov_number)
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
	auto id = neuron->GetAttribute("id")->GetAsString();

	// map <std::string, double> of parameter names and values
	auto params = parseSDFElement(neuron);

	return addNeuron(id, type, layer, params, ret, innov_number);
}



CPPNEAT::NeuronGenePtr NeatExtNN::addNeuron(const std::string &neuronId,
					   const std::string &neuronType,
					   const std::string &neuronLayer, // can be 'hidden', 'input' or 'output'
					   const std::map<std::string, double> &params, 
					   CPPNEAT::GeneticEncodingPtr ret,
					   int &innov_number)
{
	CPPNEAT::NeuronGenePtr newNeuronGene;
	CPPNEAT::NeuronPtr newNeuron;
// 	std::cout << neuronType + " " + neuronId  + " was added in"+ " "+ neuronLayer << std::endl;
	if ("input" == neuronLayer) {
		newNeuron.reset(new CPPNEAT::Neuron(neuronId, CPPNEAT::Neuron::INPUT_LAYER, CPPNEAT::Neuron::INPUT, params));

	}
	else if("output" == neuronLayer) {

		if ("Sigmoid" == neuronType) {
			newNeuron.reset(new CPPNEAT::Neuron(neuronId, CPPNEAT::Neuron::HIDDEN_LAYER, CPPNEAT::Neuron::SIGMOID, params));
		}
		else if ("Simple" == neuronType) {
			newNeuron.reset(new CPPNEAT::Neuron(neuronId, CPPNEAT::Neuron::HIDDEN_LAYER, CPPNEAT::Neuron::SIMPLE, params));
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
			newNeuron.reset(new CPPNEAT::Neuron(neuronId, CPPNEAT::Neuron::HIDDEN_LAYER, CPPNEAT::Neuron::BIAS, params));
		}
// 		else if ("Leaky" == neuronType) {
// 			newNeuron.reset(new revolve::brain::LeakyIntegrator(neuronId, params));
// 		}
		else if ("DifferentialCPG" == neuronType) {
			newNeuron.reset(new CPPNEAT::Neuron(neuronId, CPPNEAT::Neuron::HIDDEN_LAYER, CPPNEAT::Neuron::DIFFERENTIAL_CPG, params));
		}
		else {
			std::cerr << "Unsupported neuron type `" << neuronType << '`' << std::endl;
			throw std::runtime_error("Robot brain error");
		}
	} else {
		if ("Sigmoid" == neuronType) {
			newNeuron.reset(new CPPNEAT::Neuron(neuronId, CPPNEAT::Neuron::OUTPUT_LAYER, CPPNEAT::Neuron::SIGMOID, params));
		}
		else if ("Simple" == neuronType) {
			newNeuron.reset(new CPPNEAT::Neuron(neuronId, CPPNEAT::Neuron::OUTPUT_LAYER, CPPNEAT::Neuron::SIMPLE, params));
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
			newNeuron.reset(new CPPNEAT::Neuron(neuronId, CPPNEAT::Neuron::OUTPUT_LAYER, CPPNEAT::Neuron::BIAS, params));
		}
// 		else if ("Leaky" == neuronType) {
// 			newNeuron.reset(new revolve::brain::LeakyIntegrator(neuronId, params));
// 		}
		else if ("DifferentialCPG" == neuronType) {
			newNeuron.reset(new CPPNEAT::Neuron(neuronId, CPPNEAT::Neuron::OUTPUT_LAYER, CPPNEAT::Neuron::DIFFERENTIAL_CPG, params));
		}
		else {
			std::cerr << "Unsupported neuron type `" << neuronType << '`' << std::endl;
			throw std::runtime_error("Robot brain error");
		}
	}
	newNeuronGene.reset(new CPPNEAT::NeuronGene(newNeuron, innov_number++,true));
	ret->add_neuron_gene(newNeuronGene);
	return newNeuronGene;
}
} /* namespace tol */

