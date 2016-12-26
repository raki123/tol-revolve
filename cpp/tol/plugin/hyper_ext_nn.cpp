#include "hyper_ext_nn.h"
#include "helper.h"
#include "sensor.h"
#include "actuator.h"
#include "body.h"
#include "brain/conversion.h"
#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"


namespace tol {


    HyperExtNN::HyperExtNN(std::string modelName,
		     tol::EvaluatorPtr evaluator,
                     const std::vector<revolve::gazebo::MotorPtr> &actuators,
                     const std::vector<revolve::gazebo::SensorPtr> &sensors) 
    :  revolve::brain::ConvSplitBrain<boost::shared_ptr<revolve::brain::ExtNNConfig>, CPPNEAT::GeneticEncodingPtr>(&revolve::brain::convertForExtNNFromHyper, &revolve::brain::convertForHyperFromExtNN, modelName) {
// 	sleep(20);
	    
	//initialise controller
	std::string name(modelName.substr(0, modelName.find("-")) + ".yaml");
	Body body(name);
	std::pair<std::map<int, unsigned int>, std::map<int, unsigned int>> in_out = body.get_input_output_map(actuators, sensors);
	revolve::brain::input_map = in_out.first;
	revolve::brain::output_map = in_out.second;
	revolve::brain::cpg_network = revolve::brain::convertForController(body.get_coupled_cpg_network());
	revolve::brain::neuron_coordinates = body.get_id_to_coordinate_map();
	boost::shared_ptr<revolve::brain::ExtNNController1> swap1(new revolve::brain::ExtNNController1(modelName,
												       revolve::brain::cpg_network,  
												       Helper::createWrapper(actuators),
												       Helper::createWrapper(sensors)));
	controller = swap1;
	
	//initialise learner
	revolve::brain::set_learning_conf();
	std::cout << revolve::brain::getHyper() << std::endl;
	revolve::brain::set_brain_spec(true);
	revolve::brain::learning_configuration.start_from = body.get_hyper_neat_network();
	CPPNEAT::MutatorPtr mutator(new CPPNEAT::Mutator(revolve::brain::brain_spec,
					1,
					revolve::brain::learning_configuration.start_from->min_max_innov_numer().second,
					100,
					std::vector<CPPNEAT::Neuron::Ntype>(),
					true));
	boost::shared_ptr<CPPNEAT::Learner> swap2(new CPPNEAT::Learner(mutator, 
								       revolve::brain::learning_configuration));
	learner = boost::static_pointer_cast<revolve::brain::Learner<CPPNEAT::GeneticEncodingPtr>>(swap2);
	evaluator_ = evaluator;
    }

    HyperExtNN::~HyperExtNN()
    {

    }


    void HyperExtNN::update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
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

} /* namespace tol */

