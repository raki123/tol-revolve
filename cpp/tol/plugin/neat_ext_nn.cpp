#include "neat_ext_nn.h"
#include "helper.h"
#include "sensor.h"
#include "actuator.h"
#include "body.h"
#include "brain/conversion.h"
#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"


namespace tol {


    NeatExtNN::NeatExtNN(std::string modelName,
		     tol::EvaluatorPtr evaluator,
		     sdf::ElementPtr node,
                     const std::vector<revolve::gazebo::MotorPtr> &actuators,
                     const std::vector<revolve::gazebo::SensorPtr> &sensors) 
    :  revolve::brain::ConvSplitBrain<boost::shared_ptr<revolve::brain::ExtNNConfig>, CPPNEAT::GeneticEncodingPtr>(&revolve::brain::convertForController, &revolve::brain::convertForLearner, modelName) {
// 	sleep(20);
	    
	//initialise controller
	std::string name(modelName.substr(0, modelName.find("-")) + ".yaml");
	Body body(name);
	std::pair<std::map<int, unsigned int>, std::map<int, unsigned int>> in_out = body.get_input_output_map(actuators, sensors);
	revolve::brain::input_map = in_out.first;
	revolve::brain::output_map = in_out.second;
	revolve::brain::learning_configuration.start_from = body.get_coupled_cpg_network();
	boost::shared_ptr<revolve::brain::ExtNNController1> swap1(new revolve::brain::ExtNNController1(modelName,
													revolve::brain::convertForController(revolve::brain::learning_configuration.start_from),  
													Helper::createWrapper(actuators),
													Helper::createWrapper(sensors)));
// 	std::ofstream networkOutput("debug.dot"); //debug
// 	swap1->writeNetwork(networkOutput);       //debug
	int innov_number = body.getInnovNumber();
	controller = swap1;
	
	//initialise learner
	revolve::brain::set_learning_conf();
	revolve::brain::set_brain_spec(false);
	CPPNEAT::MutatorPtr mutator(new CPPNEAT::Mutator(revolve::brain::brain_spec,
					1,
					innov_number,
					100,
					std::vector<CPPNEAT::Neuron::Ntype>(),
					false));
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

} /* namespace tol */

