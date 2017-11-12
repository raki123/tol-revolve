#include "NEAT_CPPN.h"

#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include "../Actuator.h"
#include "BodyParser.h"
#include "Helper.h"
#include "brain/Conversion.h"

namespace tol {


NeatExtNN::NeatExtNN(std::string modelName,
                     sdf::ElementPtr node,
                     tol::EvaluatorPtr evaluator,
                     const std::vector<revolve::gazebo::MotorPtr> &actuators,
                     const std::vector<revolve::gazebo::SensorPtr> &sensors)
        :
        revolve::brain::ConverterSplitBrain<boost::shared_ptr<revolve::brain::CPPNConfig>,
                                       CPPNEAT::GeneticEncodingPtr>(&revolve::brain::convertForController,
                                                                    &revolve::brain::convertForLearner,
                                                                    modelName)
{

  //initialise controller
  std::string name(modelName.substr(0, modelName.find("-")) + ".yaml");
  BodyParser body(name);
  std::pair<std::map<int, size_t>, std::map<int, size_t>>
          in_out = body.InputOutputMap(actuators,
                                       sensors);
  revolve::brain::InputMap = in_out.first;
  revolve::brain::OutputMap = in_out.second;
  CPPNEAT::NEATLearner::LearningConfiguration learn_conf = parseLearningSDF(node);
  learn_conf.start_from = body.CoupledCpgNetwork();
  revolve::brain::RafCPGControllerPtr new_controller(new revolve::brain::RafCPGController(modelName,
                                                     revolve::brain::convertForController(learn_conf.start_from),
                                                     Helper::createWrapper(actuators),
                                                     Helper::createWrapper(sensors)));
  int innov_number = body.InnovationNumber();
  controller_ = new_controller;

  //initialise learner
  revolve::brain::SetBrainSpec(false);
  CPPNEAT::MutatorPtr mutator(new CPPNEAT::Mutator(revolve::brain::brain_spec,
                                                   0.8,
                                                   innov_number,
                                                   100,
                                                   std::vector<CPPNEAT::Neuron::Ntype>(),
                                                   false));
  std::string mutator_path = node->HasAttribute("path_to_mutator") ?
                             node->GetAttribute("path_to_mutator")->GetAsString() : "none";
  learner_ = boost::shared_ptr<CPPNEAT::NEATLearner>(new CPPNEAT::NEATLearner(
          mutator,
          mutator_path,
          "none",
          "none",
          learn_conf));
  evaluator_ = evaluator;
}

NeatExtNN::~NeatExtNN()
{
}


void
NeatExtNN::update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                  const std::vector<revolve::gazebo::SensorPtr> &sensors,
                  double t,
                  double step)
{
// 	std::cout << "yay" << std::endl;
  revolve::brain::ConverterSplitBrain<revolve::brain::CPPNConfigPtr, CPPNEAT::GeneticEncodingPtr>::update(
          Helper::createWrapper(actuators),
          Helper::createWrapper(sensors),
          t,
          step
  );
}


CPPNEAT::NEATLearner::LearningConfiguration
NeatExtNN::parseLearningSDF(sdf::ElementPtr brain)
{
  CPPNEAT::NEATLearner::LearningConfiguration config;

  // Read out brain configuration attributes
  config.asexual = brain->HasAttribute("asexual") ?
                   (brain->GetAttribute("asexual")->GetAsString() == "true") :
                   CPPNEAT::NEATLearner::ASEXUAL;
  config.pop_size = brain->HasAttribute("pop_size") ?
                    std::stoi(brain->GetAttribute("pop_size")->GetAsString()) :
                    CPPNEAT::NEATLearner::POP_SIZE;
  config.tournament_size = brain->HasAttribute("tournament_size") ?
                           std::stoi(brain->GetAttribute("tournament_size")->GetAsString()) :
                           CPPNEAT::NEATLearner::TOURNAMENT_SIZE;
  config.num_children = brain->HasAttribute("num_children") ?
                        std::stoi(brain->GetAttribute("num_children")->GetAsString()) :
                        CPPNEAT::NEATLearner::NUM_CHILDREN;
  config.weight_mutation_probability = brain->HasAttribute("weight_mutation_probability") ?
                                       std::stod(brain->GetAttribute("weight_mutation_probability")->GetAsString()) :
                                       CPPNEAT::NEATLearner::WEIGHT_MUTATION_PROBABILITY;
  config.weight_mutation_sigma = brain->HasAttribute("weight_mutation_sigma") ?
                                 std::stod(brain->GetAttribute("weight_mutation_sigma")->GetAsString()) :
                                 CPPNEAT::NEATLearner::WEIGHT_MUTATION_SIGMA;
  config.param_mutation_probability = brain->HasAttribute("param_mutation_probability") ?
                                      std::stod(brain->GetAttribute("param_mutation_probability")->GetAsString()) :
                                      CPPNEAT::NEATLearner::PARAM_MUTATION_PROBABILITY;
  config.param_mutation_sigma = brain->HasAttribute("param_mutation_sigma") ?
                                std::stod(brain->GetAttribute("param_mutation_sigma")->GetAsString()) :
                                CPPNEAT::NEATLearner::PARAM_MUTATION_SIGMA;
  config.structural_augmentation_probability = brain->HasAttribute("structural_augmentation_probability") ?
                                               std::stod(brain->GetAttribute("structural_augmentation_probability")->GetAsString())
                                                                                                          :
                                               CPPNEAT::NEATLearner::STRUCTURAL_AUGMENTATION_PROBABILITY;
  config.structural_removal_probability = brain->HasAttribute("structural_removal_probability") ?
                                          std::stod(brain->GetAttribute("structural_removal_probability")->GetAsString())
                                                                                                :
                                          CPPNEAT::NEATLearner::STRUCTURAL_REMOVAL_PROBABILITY;
  config.max_generations = brain->HasAttribute("max_generations") ?
                           std::stoi(brain->GetAttribute("max_generations")->GetAsString()) :
                           CPPNEAT::NEATLearner::MAX_GENERATIONS;
  config.speciation_threshold = brain->HasAttribute("speciation_threshold") ?
                                std::stod(brain->GetAttribute("speciation_threshold")->GetAsString()) :
                                CPPNEAT::NEATLearner::SPECIATION_TRESHOLD;
  config.repeat_evaluations = brain->HasAttribute("repeat_evaluations") ?
                              std::stoi(brain->GetAttribute("repeat_evaluations")->GetAsString()) :
                              CPPNEAT::NEATLearner::REPEAT_EVALUATIONS;
  config.initial_structural_mutations = brain->HasAttribute("initial_structural_mutations") ?
                                        std::stoi(brain->GetAttribute("initial_structural_mutations")->GetAsString()) :
                                        CPPNEAT::NEATLearner::INITIAL_STRUCTURAL_MUTATIONS;
  config.interspecies_mate_probability = brain->HasAttribute("interspecies_mate_probability") ?
                                         std::stod(brain->GetAttribute("interspecies_mate_probability")->GetAsString())
                                                                                              :
                                         CPPNEAT::NEATLearner::INTERSPECIES_MATE_PROBABILITY;
  return config;
}

} /* namespace tol */

