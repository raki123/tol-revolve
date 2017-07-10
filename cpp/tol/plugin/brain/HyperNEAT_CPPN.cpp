#include "HyperNEAT_CPPN.h"

#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include "../Actuator.h"
#include "Body.h"
#include "Helper.h"
#include "brain/Conversion.h"
#include "brain/Types.h"

namespace tol {

HyperNEAT_CPG::HyperNEAT_CPG(std::string modelName,
                             sdf::ElementPtr brain,
                             tol::EvaluatorPtr evaluator,
                             const std::vector<revolve::gazebo::MotorPtr> &actuators,
                             const std::vector<revolve::gazebo::SensorPtr> &sensors)
        : revolve::brain::ConverterSplitBrain<revolve::brain::CPPNConfigPtr,
                                              CPPNEAT::GeneticEncodingPtr>
                  (&revolve::brain::convertGeneticEncodingToCPPNConfig,
                   &revolve::brain::convertCPPNConfigToGeneticEncoding,
                   modelName)
{
//  	sleep(20);

  //initialise controller
  std::string name(modelName.substr(0, modelName.find("-")) + ".yaml");
  Body body(name);

  std::pair<std::map<int, size_t >, std::map<int, size_t >> in_out = body.get_input_output_map(actuators, sensors);
  revolve::brain::input_map = in_out.first;
  revolve::brain::output_map = in_out.second;
  revolve::brain::cpg_network = revolve::brain::convertForController(body.get_coupled_cpg_network());
  revolve::brain::neuron_coordinates = body.get_id_to_coordinate_map();

  // initialise controller
  controller_ = revolve::brain::CPPNControllerPtr(new revolve::brain::CPPNController(modelName,
                                                                                    revolve::brain::cpg_network,
                                                                                    Helper::createWrapper(actuators),
                                                                                    Helper::createWrapper(sensors)));

  //initialise learner
  CPPNEAT::Learner::LearningConfiguration learn_conf = parseLearningSDF(brain);
  revolve::brain::set_brain_spec(true);
  learn_conf.start_from = body.get_hyper_neat_network();
  CPPNEAT::MutatorPtr mutator(new CPPNEAT::Mutator(revolve::brain::brain_spec, 0.8,
                                                   learn_conf.start_from->min_max_innov_numer().second, 100,
                                                   std::vector<CPPNEAT::Neuron::Ntype>(),
                                                   true));
  std::string mutator_path = brain->HasAttribute("path_to_mutator") ?
                             brain->GetAttribute("path_to_mutator")->GetAsString() : "none";

  // initialise learner
  learner_ = boost::shared_ptr<CPPNEAT::Learner>(new CPPNEAT::Learner(mutator, mutator_path, learn_conf));

  //initialise starting population
  int number_of_brains_from_first = brain->HasAttribute("number_of_brains_from_first") ?
                                    std::stoi(brain->GetAttribute("number_of_brains_from_first")->GetAsString()) : 0;
  int number_of_brains_from_second = brain->HasAttribute("number_of_brains_from_second") ?
                                     std::stoi(brain->GetAttribute("number_of_brains_from_second")->GetAsString()) : 0;
  std::string path_to_first_brains = brain->HasAttribute("path_to_first_brains") ?
                                     brain->GetAttribute("path_to_first_brains")->GetAsString() : "";

  std::vector<CPPNEAT::GeneticEncodingPtr> brains_from_init =
          boost::dynamic_pointer_cast<CPPNEAT::Learner>(learner_)->get_init_brains();
  std::vector<CPPNEAT::GeneticEncodingPtr> brains_from_first;
  if (path_to_first_brains == "" || path_to_first_brains == "none") {
    number_of_brains_from_first = 0;
  } else {
    brains_from_first = boost::dynamic_pointer_cast<CPPNEAT::Learner>(learner_)->get_brains_from_yaml(path_to_first_brains, -1);
  }
  std::string path_to_second_brains = brain->HasAttribute("path_to_second_brains") ?
                                      brain->GetAttribute("path_to_second_brains")->GetAsString() : "";
  std::vector<CPPNEAT::GeneticEncodingPtr> brains_from_second;
  if (path_to_second_brains == "" || path_to_second_brains == "none") {
    number_of_brains_from_second = 0;
  } else {
    brains_from_second = boost::dynamic_pointer_cast<CPPNEAT::Learner>(learner_)->get_brains_from_yaml(path_to_second_brains, -1);
  }

  std::vector<CPPNEAT::GeneticEncodingPtr> init_brains;
  int cur_number = 0;
  int i = 0;
  while (cur_number < number_of_brains_from_first) {
    init_brains.push_back(brains_from_first[i]);
    i++;
    cur_number++;
  }
  i = 0;
  while (cur_number < number_of_brains_from_first + number_of_brains_from_second) {
    init_brains.push_back(brains_from_second[i]);
    i++;
    cur_number++;
  }
  i = 0;
  while (cur_number < learn_conf.pop_size) {
    init_brains.push_back(brains_from_init[i]);
    i++;
    cur_number++;
  }
  boost::dynamic_pointer_cast<CPPNEAT::Learner>(learner_)->initialise(init_brains);

  // initialise evaluator
  evaluator_ = evaluator;
}

HyperNEAT_CPG::~HyperNEAT_CPG()
{
}

void
HyperNEAT_CPG::update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                   const std::vector<revolve::gazebo::SensorPtr> &sensors,
                   double t,
                   double step)
{
  revolve::brain::ConverterSplitBrain<boost::shared_ptr<revolve::brain::CPPNConfig>, CPPNEAT::GeneticEncodingPtr>::update(
          Helper::createWrapper(actuators),
          Helper::createWrapper(sensors),
          t,
          step
  );
}

CPPNEAT::Learner::LearningConfiguration
HyperNEAT_CPG::parseLearningSDF(sdf::ElementPtr brain)
{
  CPPNEAT::Learner::LearningConfiguration config;

  // Read out brain configuration attributes
  config.asexual = brain->HasAttribute("asexual") ?
                   (brain->GetAttribute("asexual")
                         ->GetAsString() == "true") :
                   CPPNEAT::Learner::ASEXUAL;
  config.pop_size = brain->HasAttribute("pop_size") ?
                    std::stoi(brain->GetAttribute("pop_size")
                                   ->GetAsString()) :
                    CPPNEAT::Learner::POP_SIZE;
  config.tournament_size = brain->HasAttribute("tournament_size") ?
                           std::stoi(brain->GetAttribute("tournament_size")
                                          ->GetAsString()) :
                           CPPNEAT::Learner::TOURNAMENT_SIZE;
  config.num_children = brain->HasAttribute("num_children") ?
                        std::stoi(brain->GetAttribute("num_children")
                                       ->GetAsString()) :
                        CPPNEAT::Learner::NUM_CHILDREN;
  config.weight_mutation_probability = brain->HasAttribute("weight_mutation_probability") ?
                                       std::stod(brain->GetAttribute("weight_mutation_probability")
                                                      ->GetAsString()) :
                                       CPPNEAT::Learner::WEIGHT_MUTATION_PROBABILITY;
  config.weight_mutation_sigma = brain->HasAttribute("weight_mutation_sigma") ?
                                 std::stod(brain->GetAttribute("weight_mutation_sigma")
                                                ->GetAsString()) :
                                 CPPNEAT::Learner::WEIGHT_MUTATION_SIGMA;
  config.param_mutation_probability = brain->HasAttribute("param_mutation_probability") ?
                                      std::stod(brain->GetAttribute("param_mutation_probability")
                                                     ->GetAsString()) :
                                      CPPNEAT::Learner::PARAM_MUTATION_PROBABILITY;
  config.param_mutation_sigma = brain->HasAttribute("param_mutation_sigma") ?
                                std::stod(brain->GetAttribute("param_mutation_sigma")
                                               ->GetAsString()) :
                                CPPNEAT::Learner::PARAM_MUTATION_SIGMA;
  config.structural_augmentation_probability = brain->HasAttribute("structural_augmentation_probability") ?
                                               std::stod(brain->GetAttribute("structural_augmentation_probability")
                                                              ->GetAsString()) :
                                               CPPNEAT::Learner::STRUCTURAL_AUGMENTATION_PROBABILITY;
  config.structural_removal_probability = brain->HasAttribute("structural_removal_probability") ?
                                          std::stod(brain->GetAttribute("structural_removal_probability")
                                                         ->GetAsString()) :
                                          CPPNEAT::Learner::STRUCTURAL_REMOVAL_PROBABILITY;
  config.max_generations = brain->HasAttribute("max_generations") ?
                           std::stoi(brain->GetAttribute("max_generations")
                                          ->GetAsString()) :
                           CPPNEAT::Learner::MAX_GENERATIONS;
  config.speciation_threshold = brain->HasAttribute("speciation_threshold") ?
                                std::stod(brain->GetAttribute("speciation_threshold")
                                               ->GetAsString()) :
                                CPPNEAT::Learner::SPECIATION_TRESHOLD;
  config.repeat_evaluations = brain->HasAttribute("repeat_evaluations") ?
                              std::stoi(brain->GetAttribute("repeat_evaluations")
                                             ->GetAsString()) :
                              CPPNEAT::Learner::REPEAT_EVALUATIONS;
  config.initial_structural_mutations = brain->HasAttribute("initial_structural_mutations") ?
                                        std::stoi(brain->GetAttribute("initial_structural_mutations")
                                                       ->GetAsString()) :
                                        CPPNEAT::Learner::INITIAL_STRUCTURAL_MUTATIONS;
  config.interspecies_mate_probability = brain->HasAttribute("interspecies_mate_probability") ?
                                         std::stod(brain->GetAttribute("interspecies_mate_probability")
                                                        ->GetAsString()) :
                                         CPPNEAT::Learner::INTERSPECIES_MATE_PROBABILITY;
  return config;
}

} /* namespace tol */
