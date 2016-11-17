#include "extended_neural_network.h"
#include "helper.h"
#include "sensor.h"
#include "actuator.h"
#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"


namespace tol {

    ExtendedNeuralNetwork::ExtendedNeuralNetwork(std::string modelName,
		     tol::EvaluatorPtr evaluator,
		     sdf::ElementPtr node,
                     const std::vector<revolve::gazebo::MotorPtr> &actuators,
                     const std::vector<revolve::gazebo::SensorPtr> &sensors) :
            revolve::brain::ExtendedNeuralNetwork(
                    modelName,
                    parseSDF(node, actuators, sensors),  
		    evaluator, 
		    Helper::createWrapper(actuators),
		    Helper::createWrapper(sensors)) { }

    ExtendedNeuralNetwork::~ExtendedNeuralNetwork()
    {

    }


    void ExtendedNeuralNetwork::update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                         const std::vector<revolve::gazebo::SensorPtr> &sensors,
                         double t,
                         double step) {
	//std::cout << "yay" << std::endl;
        revolve::brain::ExtendedNeuralNetwork::update(
                Helper::createWrapper(actuators),
                Helper::createWrapper(sensors),
                t, step
        );
    }

    
ExtendedNeuralNetwork::ExtNNConfig ExtendedNeuralNetwork::parseSDF(sdf::ElementPtr node,
		       const std::vector< revolve::gazebo::MotorPtr > & motors,
		       const std::vector< revolve::gazebo::SensorPtr > & sensors) {
	ExtNNConfig ret;
	
	// Map neuron sdf elements to their id's
	std::map<std::string, sdf::ElementPtr> neuronDescriptions;

	// List of all hidden neuron id's
	std::vector<std::string> hiddenIds;

	// Number of input neurons for mapping them to the input buffer
	ret.numInputNeurons_ = 0;

	// Number of output neurons for mapping them to the output buffer
	ret.numOutputNeurons_ = 0;

	// Number of hidden neurons
	ret.numHiddenNeurons_ = 0;

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
			ret.idToNeuron_[neuronId.str()] = newNeuron;
			
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
			ret.idToNeuron_[neuronId.str()] = newNeuron;
		}
	}

	// initialize the array for sensor inputs:
	ret.inputs_ = new double[ret.numInputNeurons_];
	ret.outputs_ = new double[ret.numOutputNeurons_];


	// Add hidden neurons:
	for (auto it = hiddenIds.begin(); it != hiddenIds.end(); ++it) {
		auto neuronDescription = neuronDescriptions.find(*it);
		auto newNeuron = neuronHelper(neuronDescription->second,ret);
		ret.idToNeuron_[*it] = newNeuron;
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
		connectionHelper(src, dst, dstSocketName, weight, ret.idToNeuron_, ret);

		// Load the next connection
		connection = connection->GetNextElement("rv:neural_connection");
	}
	return ret;
}

std::map<std::string, double> ExtendedNeuralNetwork::parseSDFElement(sdf::ElementPtr elem)
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

void ExtendedNeuralNetwork::connectionHelper(const std::string &src,
					     const std::string &dst,
					     const std::string &socket,
					     double weight,
					     const std::map<std::string, revolve::brain::NeuronPtr> &idToNeuron, 
					     ExtNNConfig &ret)
{
	std::cout << "connection from " + src + " to " + dst + " was added with weight: " << weight << std::endl;
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

	revolve::brain::NeuralConnectionPtr newConnection(new revolve::brain::NeuralConnection(
		srcNeuron->second,
		dstNeuron->second,
		weight
	));

	// Add reference to this connection to the destination neuron
	(dstNeuron->second)->AddIncomingConnection(socket, newConnection);
	ret.connections_.push_back(newConnection);
}


revolve::brain::NeuronPtr ExtendedNeuralNetwork::neuronHelper(sdf::ElementPtr neuron,
		       ExtNNConfig &ret)
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

	return addNeuron(id, type, layer, params, ret);
}



revolve::brain::NeuronPtr ExtendedNeuralNetwork::addNeuron(const std::string &neuronId,
					   const std::string &neuronType,
					   const std::string &neuronLayer, // can be 'hidden', 'input' or 'output'
					   const std::map<std::string, double> &params, 
					   ExtNNConfig &ret)
{
	revolve::brain::NeuronPtr newNeuron;
	std::cout << neuronType + " " + neuronId  + " was added in"+ " "+ neuronLayer << std::endl;
	if ("input" == neuronLayer) {
		newNeuron.reset(new revolve::brain::InputNeuron(neuronId, params));

		ret.inputNeurons_.push_back(newNeuron);
		ret.inputPositionMap_[newNeuron] = ret.numInputNeurons_;
		ret.numInputNeurons_++;
	}

	else {

		if ("Sigmoid" == neuronType) {
			newNeuron.reset(new revolve::brain::SigmoidNeuron(neuronId, params));
		}
		else if ("Simple" == neuronType) {
			newNeuron.reset(new revolve::brain::LinearNeuron(neuronId, params));
		}
		else if ("Oscillator" == neuronType) {
			newNeuron.reset(new revolve::brain::OscillatorNeuron(neuronId, params));
		}
		else if ("V-Neuron" == neuronType) {
			newNeuron.reset(new revolve::brain::VOscillator(neuronId, params));
		}
		else if ("X-Neuron" == neuronType) {
			newNeuron.reset(new revolve::brain::XOscillator(neuronId, params));
		}
		else if ("Bias" == neuronType) {
			newNeuron.reset(new revolve::brain::BiasNeuron(neuronId, params));
		}
		else if ("Leaky" == neuronType) {
			newNeuron.reset(new revolve::brain::LeakyIntegrator(neuronId, params));
		}
		else if ("DifferentialCPG" == neuronType) {
			newNeuron.reset(new revolve::brain::DifferentialCPG(neuronId, params));
		}
		else {
			std::cerr << "Unsupported neuron type `" << neuronType << '`' << std::endl;
			throw std::runtime_error("Robot brain error");
		}

		if ("output" == neuronLayer) {
			ret.outputNeurons_.push_back(newNeuron);
			ret.outputPositionMap_[newNeuron] = ret.numOutputNeurons_;
			ret.numOutputNeurons_++;
		}
		else {
			ret.hiddenNeurons_.push_back(newNeuron);
			ret.numHiddenNeurons_++;
		}
	}

	ret.allNeurons_.push_back(newNeuron);
	return newNeuron;
}
} /* namespace tol */

