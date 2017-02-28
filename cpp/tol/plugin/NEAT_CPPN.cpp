#include "NEAT_CPPN.h"
#include "Helper.h"
#include "Sensor.h"
#include "Actuator.h"
#include "Body.h"
#include "brain/conversion.h"
#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"


namespace tol
{


NeatExtNN::NeatExtNN(std::string modelName,
                     sdf::ElementPtr node,
                     tol::EvaluatorPtr evaluator,
                     const std::vector<revolve::gazebo::MotorPtr> &actuators,
                     const std::vector<revolve::gazebo::SensorPtr> &sensors)
        :
        revolve::brain::ConvSplitBrain<boost::shared_ptr<revolve::brain::ExtNNConfig>, CPPNEAT::GeneticEncodingPtr>(&revolve::brain::convertForController,
                                                                                                                    &revolve::brain::convertForLearner,
                                                                                                                    modelName)
{

  //initialise controller
  std::string name(modelName.substr(0,
                                    modelName.find("-")) + ".yaml");
  Body body(name);
  std::pair<std::map<int, unsigned int>, std::map<int, unsigned int>>
          in_out = body.get_input_output_map(actuators,
                                             sensors);
  revolve::brain::input_map = in_out.first;
  revolve::brain::output_map = in_out.second;
  CPPNEAT::Learner::LearningConfiguration learn_conf = parseLearningSDF(node);
  learn_conf.start_from = body.get_coupled_cpg_network();
  boost::shared_ptr<revolve::brain::ExtNNController1>
          swap1(new revolve::brain::ExtNNController1(modelName,
                                                     revolve::brain::convertForController(learn_conf.start_from),
                                                     Helper::createWrapper(actuators),
                                                     Helper::createWrapper(sensors)));
  int innov_number = body.getInnovNumber();
  controller = swap1;

  //initialise learner
  revolve::brain::set_brain_spec(false);
  CPPNEAT::MutatorPtr mutator(new CPPNEAT::Mutator(revolve::brain::brain_spec,
                                                   0.8,
                                                   innov_number,
                                                   100,
                                                   std::vector<CPPNEAT::Neuron::Ntype>(),
                                                   false));
  std::string mutator_path = node->HasAttribute("path_to_mutator") ?
                             node->GetAttribute("path_to_mutator")
                                     ->GetAsString()
                                                                   : "none";
  learner = boost::shared_ptr<CPPNEAT::Learner>(new CPPNEAT::Learner(mutator,
                                                                     mutator_path,
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
  revolve::brain::ConvSplitBrain<boost::shared_ptr<revolve::brain::ExtNNConfig>, CPPNEAT::GeneticEncodingPtr>::update(
          Helper::createWrapper(actuators),
          Helper::createWrapper(sensors),
          t,
          step
  );
}


CPPNEAT::Learner::LearningConfiguration
NeatExtNN::parseLearningSDF(sdf::ElementPtr brain)
{
  CPPNEAT::Learner::LearningConfiguration config;

  // Read out brain configuration attributes
  config.asexual = brain->HasAttribute("asexual") ?
                   (brain->GetAttribute("asexual")->GetAsString() == "true") :
                   CPPNEAT::Learner::ASEXUAL;
  config.pop_size = brain->HasAttribute("pop_size") ?
                    std::stoi(brain->GetAttribute("pop_size")->GetAsString()) :
                    CPPNEAT::Learner::POP_SIZE;
  config.tournament_size = brain->HasAttribute("tournament_size") ?
                           std::stoi(brain->GetAttribute("tournament_size")->GetAsString()) :
                           CPPNEAT::Learner::TOURNAMENT_SIZE;
  config.num_children = brain->HasAttribute("num_children") ?
                        std::stoi(brain->GetAttribute("num_children")->GetAsString()) :
                        CPPNEAT::Learner::NUM_CHILDREN;
  config.weight_mutation_probability = brain->HasAttribute("weight_mutation_probability") ?
                                       std::stod(brain->GetAttribute("weight_mutation_probability")->GetAsString()) :
                                       CPPNEAT::Learner::WEIGHT_MUTATION_PROBABILITY;
  config.weight_mutation_sigma = brain->HasAttribute("weight_mutation_sigma") ?
                                 std::stod(brain->GetAttribute("weight_mutation_sigma")->GetAsString()) :
                                 CPPNEAT::Learner::WEIGHT_MUTATION_SIGMA;
  config.param_mutation_probability = brain->HasAttribute("param_mutation_probability") ?
                                      std::stod(brain->GetAttribute("param_mutation_probability")->GetAsString()) :
                                      CPPNEAT::Learner::PARAM_MUTATION_PROBABILITY;
  config.param_mutation_sigma = brain->HasAttribute("param_mutation_sigma") ?
                                std::stod(brain->GetAttribute("param_mutation_sigma")->GetAsString()) :
                                CPPNEAT::Learner::PARAM_MUTATION_SIGMA;
  config.structural_augmentation_probability = brain->HasAttribute("structural_augmentation_probability") ?
                                               std::stod(brain->GetAttribute("structural_augmentation_probability")->GetAsString()) :
                                               CPPNEAT::Learner::STRUCTURAL_AUGMENTATION_PROBABILITY;
  config.structural_removal_probability = brain->HasAttribute("structural_removal_probability") ?
                                          std::stod(brain->GetAttribute("structural_removal_probability")->GetAsString()) :
                                          CPPNEAT::Learner::STRUCTURAL_REMOVAL_PROBABILITY;
  config.max_generations = brain->HasAttribute("max_generations") ?
                           std::stoi(brain->GetAttribute("max_generations")->GetAsString()) :
                           CPPNEAT::Learner::MAX_GENERATIONS;
  config.speciation_threshold = brain->HasAttribute("speciation_threshold") ?
                                std::stod(brain->GetAttribute("speciation_threshold")->GetAsString()) :
                                CPPNEAT::Learner::SPECIATION_TRESHOLD;
  config.repeat_evaluations = brain->HasAttribute("repeat_evaluations") ?
                              std::stoi(brain->GetAttribute("repeat_evaluations")->GetAsString()) :
                              CPPNEAT::Learner::REPEAT_EVALUATIONS;
  config.initial_structural_mutations = brain->HasAttribute("initial_structural_mutations") ?
                                        std::stoi(brain->GetAttribute("initial_structural_mutations")->GetAsString()) :
                                        CPPNEAT::Learner::INITIAL_STRUCTURAL_MUTATIONS;
  config.interspecies_mate_probability = brain->HasAttribute("interspecies_mate_probability") ?
                                         std::stod(brain->GetAttribute("interspecies_mate_probability")->GetAsString()) :
                                         CPPNEAT::Learner::INTERSPECIES_MATE_PROBABILITY;
  return config;
}

} /* namespace tol */

