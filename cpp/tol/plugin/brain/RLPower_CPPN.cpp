//
// Created by Milan Jelisavcic on 28/03/16.
//

#include "RLPower_CPPN.h"

#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include "Body.h"
#include "Helper.h"
#include "brain/Conversion.h"
#include "brain/controller/ExtCPPNWeights.h"

namespace tol {

RLPower_CPG::RLPower_CPG(std::string model_name,
                         sdf::ElementPtr brain,
                         EvaluatorPtr evaluator,
                         std::vector<revolve::gazebo::MotorPtr> &actuators,
                         std::vector<revolve::gazebo::SensorPtr> &sensors)
        : revolve::brain::ConverterSplitBrain<std::vector<double>,
                                              revolve::brain::PolicyPtr>
                  (&revolve::brain::convertPolicyToDouble,
                   &revolve::brain::convertDoubleToNull,
                   model_name)
{

  //initialise controller
  std::string name(model_name.substr(0, model_name.find("-")) + ".yaml");
  Body body(name);

  std::pair<std::map<int, size_t>, std::map<int, size_t>> in_out =
          body.get_input_output_map(actuators, sensors);
  revolve::brain::input_map = in_out.first;
  revolve::brain::output_map = in_out.second;

  controller_ = boost::shared_ptr<revolve::brain::ExtNNController>
          (new revolve::brain::ExtNNController(model_name,
                                               revolve::brain::convertForController(body.get_coupled_cpg_network()),
                                               Helper::createWrapper(actuators),
                                               Helper::createWrapper(sensors)));
  revolve::brain::RLPowerLearner::Config config = parseSDF(brain);
  config.source_y_size = (size_t)controller_->getPhenotype().size();

  learner_ = boost::shared_ptr<revolve::brain::RLPowerLearner>
          (new revolve::brain::RLPowerLearner(model_name, config, 1));
  evaluator_ = evaluator;
}

RLPower_CPG::~RLPower_CPG()
{}

void
RLPower_CPG::update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                   const std::vector<revolve::gazebo::SensorPtr> &sensors,
                   double t,
                   double step)
{
  revolve::brain::ConverterSplitBrain<std::vector<double>, revolve::brain::PolicyPtr>::update(
          Helper::createWrapper(actuators),
          Helper::createWrapper(sensors),
          t,
          step
  );
}

revolve::brain::RLPowerLearner::Config
RLPower_CPG::parseSDF(sdf::ElementPtr brain)
{
  revolve::brain::RLPowerLearner::Config config;

  // Read out brain configuration attributes
  config.algorithm_type = brain->HasAttribute("type") ? brain->GetAttribute("type")
                                                             ->GetAsString() : "A";

  config.evaluation_rate = brain->HasAttribute("evaluation_rate") ?
                           std::stod(brain->GetAttribute("evaluation_rate")
                                          ->GetAsString()) :
                           revolve::brain::RLPowerLearner::EVALUATION_RATE;
  config.interpolation_spline_size = brain->HasAttribute("interpolation_spline_size") ?
                                     std::stoul(brain->GetAttribute("interpolation_spline_size")
                                                     ->GetAsString()) :
                                     revolve::brain::RLPowerLearner::INTERPOLATION_CACHE_SIZE;
  config.max_evaluations = brain->HasAttribute("max_evaluations") ?
                           std::stoul(brain->GetAttribute("max_evaluations")
                                           ->GetAsString()) :
                           revolve::brain::RLPowerLearner::MAX_EVALUATIONS;
  config.max_ranked_policies = brain->HasAttribute("max_ranked_policies") ?
                               std::stoul(brain->GetAttribute("max_ranked_policies")
                                               ->GetAsString()) :
                               revolve::brain::RLPowerLearner::MAX_RANKED_POLICIES;
  config.noise_sigma = brain->HasAttribute("init_sigma") ?
                       std::stod(brain->GetAttribute("init_sigma")
                                      ->GetAsString()) :
                       revolve::brain::RLPowerLearner::SIGMA_START_VALUE;
  config.sigma_tau_correction = brain->HasAttribute("sigma_tau_correction") ?
                                std::stod(brain->GetAttribute("sigma_tau_correction")
                                               ->GetAsString()) :
                                revolve::brain::RLPowerLearner::SIGMA_TAU_CORRECTION;
  config.update_step = 0;
  config.policy_load_path = "";

  return config;
}

} /* namespace tol */
