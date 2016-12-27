#include "hyper_splines.h"
#include "helper.h"
#include "sensor.h"
#include "actuator.h"
#include "body.h"
#include "brain/conversion.h"
#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"


namespace tol {


    HyperSplines::HyperSplines(std::string modelName,
			       sdf::ElementPtr brain,
			       tol::EvaluatorPtr evaluator,
                               const std::vector<revolve::gazebo::MotorPtr> &actuators,
                               const std::vector<revolve::gazebo::SensorPtr> &sensors) 
    :  revolve::brain::ConvSplitBrain<revolve::brain::PolicyPtr, CPPNEAT::GeneticEncodingPtr>(&revolve::brain::convertForSplinesFromHyper, &revolve::brain::convertForHyperFromSplines, modelName) {
// 	sleep(20);
	    
	//initialise controller
	std::string name(modelName.substr(0, modelName.find("-")) + ".yaml");
	Body body(name);
	revolve::brain::sorted_coordinates = body.get_coordinates_sorted(actuators);
	revolve::brain::RLPowerLearner::Config conf = parseSDF(brain);
	revolve::brain::update_rate = conf.update_step;
	revolve::brain::spline_size = conf.source_y_size;
	controller = boost::shared_ptr<revolve::brain::PolicyController>
		     (new revolve::brain::PolicyController(revolve::brain::sorted_coordinates.size(), 
							   conf.interpolation_spline_size));

	
	//initialise learner
	revolve::brain::set_learning_conf();
	std::cout << revolve::brain::getHyper() << std::endl;
	revolve::brain::set_brain_spec(true);
	revolve::brain::learning_configuration.start_from = revolve::brain::get_hyper_neat_net_splines();
	CPPNEAT::MutatorPtr mutator(new CPPNEAT::Mutator(revolve::brain::brain_spec,
							 1,
							 revolve::brain::learning_configuration.start_from->min_max_innov_numer().second,
							 100,
							 std::vector<CPPNEAT::Neuron::Ntype>(),
							 true));
	learner = boost::shared_ptr<revolve::brain::Learner<CPPNEAT::GeneticEncodingPtr>>
		  (new CPPNEAT::Learner(mutator, 
					revolve::brain::learning_configuration));
	evaluator_ = evaluator;
    }

    HyperSplines::~HyperSplines()
    {

    }


    void HyperSplines::update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                         const std::vector<revolve::gazebo::SensorPtr> &sensors,
                         double t,
                         double step) {
// 	std::cout << "yay" << std::endl;
        revolve::brain::ConvSplitBrain<revolve::brain::PolicyPtr, CPPNEAT::GeneticEncodingPtr>::update(
                Helper::createWrapper(actuators),
                Helper::createWrapper(sensors),
                t, step
        );
    }
    
    revolve::brain::RLPowerLearner::Config HyperSplines::parseSDF(sdf::ElementPtr brain) {
        revolve::brain::RLPowerLearner::Config config;

        // Read out brain configuration attributes
        config.evaluation_rate = brain->HasAttribute("evaluation_rate") ?
                                 std::stod(brain->GetAttribute("evaluation_rate")->GetAsString()) :
                                 revolve::brain::RLPowerLearner::EVALUATION_RATE;
        config.interpolation_spline_size = brain->HasAttribute("interpolation_spline_size") ?
                                          std::stoul(brain->GetAttribute("interpolation_spline_size")->GetAsString()) :
                                          revolve::brain::RLPowerLearner::INTERPOLATION_CACHE_SIZE;
        config.source_y_size = brain->HasAttribute("init_spline_size") ?
                               std::stoul(brain->GetAttribute("init_spline_size")->GetAsString()) :
                               revolve::brain::RLPowerLearner::INITIAL_SPLINE_SIZE;
        config.update_step = brain->HasAttribute("update_step") ?
                             std::stoul(brain->GetAttribute("update_step")->GetAsString()) :
                             revolve::brain::RLPowerLearner::UPDATE_STEP;

        return config;
    }

} /* namespace tol */

