//
// Created by Milan Jelisavcic on 28/03/16.
//

#include "rlpowered_network.h"
#include "brain/conversion.h"
#include "brain/controller/ext_nn_weights.h"
#include "body.h"
#include "helper.h"
#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

namespace tol {
      

  
    RLPowerNet::RLPowerNet(std::string modelName,
                     sdf::ElementPtr brain,
                     EvaluatorPtr evaluator,
                     std::vector<revolve::gazebo::MotorPtr> &actuators,
                     std::vector<revolve::gazebo::SensorPtr> &sensors) :
		revolve::brain::ConvSplitBrain<std::vector<double>, std::vector<std::vector<double>>>(&revolve::brain::forController, &revolve::brain::forLearner, modelName)  { 
//  	sleep(20);
	    
	//initialise controller
	std::string name(modelName.substr(0, modelName.find("-")) + ".yaml");
	Body body(name);
	std::pair<std::map<int, unsigned int>, std::map<int, unsigned int>> in_out = body.get_input_output_map(actuators, sensors);
	revolve::brain::input_map = in_out.first;
	revolve::brain::output_map = in_out.second;
	boost::shared_ptr<revolve::brain::ExtNNController> swap1(new revolve::brain::ExtNNController(modelName,
													revolve::brain::convertForController(body.get_coupled_cpg_network()),  
													Helper::createWrapper(actuators),
													Helper::createWrapper(sensors)));
	controller = swap1;
	revolve::brain::RLPowerLearner::Config config = parseSDF(brain);
	config.source_y_size = swap1->getGenome().size();
	boost::shared_ptr<revolve::brain::RLPowerLearner> swap2(new revolve::brain::RLPowerLearner(modelName,
												    config,
												    1));
	learner =  boost::static_pointer_cast<revolve::brain::Learner<std::vector<std::vector<double>>>>(swap2);
	evaluator_ = evaluator;
    }

    RLPowerNet::~RLPowerNet() { }

    void RLPowerNet::update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                         const std::vector<revolve::gazebo::SensorPtr> &sensors,
                         double t,
                         double step) {
        revolve::brain::ConvSplitBrain<std::vector<double>,std::vector<std::vector<double>>>::update(
                Helper::createWrapper(actuators),
                Helper::createWrapper(sensors),
                t, step
        );
    }

    revolve::brain::RLPowerLearner::Config RLPowerNet::parseSDF(sdf::ElementPtr brain) {
        revolve::brain::RLPowerLearner::Config config;

        // Read out brain configuration attributes
        config.algorithm_type = brain->HasAttribute("type") ? brain->GetAttribute("type")->GetAsString() : "A";

        config.evaluation_rate = brain->HasAttribute("evaluation_rate") ?
                                 std::stod(brain->GetAttribute("evaluation_rate")->GetAsString()) :
                                 revolve::brain::RLPowerLearner::EVALUATION_RATE;
        config.interpolation_spline_size = brain->HasAttribute("interpolation_spline_size") ?
                                          std::stoul(brain->GetAttribute("interpolation_spline_size")->GetAsString()) :
                                          revolve::brain::RLPowerLearner::INTERPOLATION_CACHE_SIZE;
        config.max_evaluations = brain->HasAttribute("max_evaluations") ?
                                 std::stoul(brain->GetAttribute("max_evaluations")->GetAsString()) :
                                 revolve::brain::RLPowerLearner::MAX_EVALUATIONS;
        config.max_ranked_policies = brain->HasAttribute("max_ranked_policies") ?
                                     std::stoul(brain->GetAttribute("max_ranked_policies")->GetAsString()) :
                                     revolve::brain::RLPowerLearner::MAX_RANKED_POLICIES;
        config.noise_sigma = 10;
        config.sigma_tau_correction = brain->HasAttribute("sigma_tau_correction") ?
                                      std::stod(brain->GetAttribute("sigma_tau_correction")->GetAsString()) :
                                      revolve::brain::RLPowerLearner::SIGMA_TAU_CORRECTION;
        config.source_y_size = 6;
        config.update_step = 0;
        config.policy_load_path = "";

        return config;
    }

} /* namespace tol */
