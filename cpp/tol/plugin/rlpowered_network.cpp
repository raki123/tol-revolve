//
// Created by Milan Jelisavcic on 28/03/16.
//

#include "rlpowered_network.h"
#include "simple_split_brain.h"
#include "helper.h"
#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

namespace tol {
      
    std::vector<double> forController(std::vector<std::vector<double>> toConvert) {
	return toConvert[0];
    }
    std::vector<std::vector<double>> forLearner(std::vector<double> toConvert) {
	return std::vector<std::vector<double>>(1,toConvert);
    }
  
    RLPowerNet::RLPowerNet(std::string modelName,
                     sdf::ElementPtr brain,
                     EvaluatorPtr evaluator,
                     std::vector<revolve::gazebo::MotorPtr> &actuators,
                     std::vector<revolve::gazebo::SensorPtr> &sensors) :
		revolve::brain::ConvSplitBrain<std::vector<double>, std::vector<std::vector<double>>>(&forController, &forLearner, modelName)  { 
	boost::shared_ptr<revolve::brain::ExtNNController> swap1(new revolve::brain::ExtNNController(modelName,
												      ExtNN::parseSDF(brain, actuators, sensors),  
												      evaluator, 
												      Helper::createWrapper(actuators),
												      Helper::createWrapper(sensors)));
	controller = boost::static_pointer_cast<revolve::brain::Controller<std::vector<double>>>(swap1);
	boost::shared_ptr<revolve::brain::RLPowerLearner> swap2(new revolve::brain::RLPowerLearner(modelName,
												    parseSDF(brain),
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
