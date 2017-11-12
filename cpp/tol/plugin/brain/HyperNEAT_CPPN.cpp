#include "HyperNEAT_CPPN.h"

#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include "../Actuator.h"
#include "BodyParser.h"
#include "Helper.h"
#include "brain/Conversion.h"

namespace rb = revolve::brain;
namespace rg = revolve::gazebo;

namespace tol
{
  HyperNEAT_CPG::HyperNEAT_CPG(
          std::string modelName,
          sdf::ElementPtr brain,
          tol::EvaluatorPtr evaluator,
          const std::vector< rg::MotorPtr > &actuators,
          const std::vector< rg::SensorPtr > &sensors
  )
          : rb::ConverterSplitBrain< rb::CPPNConfigPtr,
                                     CPPNEAT::GeneticEncodingPtr >
          (&rb::convertGEtoNN,
           &rb::convertNNtoGE,
           modelName)
  {

    // Initialise controller
    std::string name(modelName.substr(0, modelName.find("-")) + ".yaml");
    BodyParser body(name);

    std::tie(rb::InputMap, rb::OutputMap) = body.InputOutputMap(
            actuators,
            sensors);

    rb::RafCpgNetwork = rb::convertForController(body.CoupledCpgNetwork());
    rb::neuron_coordinates = body.IdToCoordinatesMap();

    // Initialise controller
    controller_ = rb::RafCPGControllerPtr(new rb::RafCPGController(
            modelName,
            rb::RafCpgNetwork,
            Helper::createWrapper(actuators),
            Helper::createWrapper(sensors))
    );

    // Initialise learner
    auto learn_conf = parseLearningSDF(brain);
    rb::SetBrainSpec(true);
    learn_conf.start_from = body.CppnNetwork();
    CPPNEAT::MutatorPtr mutator(new CPPNEAT::Mutator(
            rb::brain_spec,
            0.8,
            learn_conf.start_from->min_max_innov_numer().second,
            100,
            std::vector< CPPNEAT::Neuron::Ntype >(),
            true)
    );
    std::string mutator_path =
            brain->HasAttribute("path_to_mutator") ?
            brain->GetAttribute("path_to_mutator")->GetAsString() : "none";

    // initialise learner
    this->learner_ =
            boost::shared_ptr< CPPNEAT::NEATLearner >(new CPPNEAT::NEATLearner(
                    mutator,
//                    mutator_path,
                    modelName + ".innovations",
                    learn_conf)
            );

    //initialise starting population
    int number_of_brains_from_first =
            brain->HasAttribute("number_of_brains_from_first") ?
            std::stoi(brain->GetAttribute("number_of_brains_from_first")->GetAsString())
                                                               : 0;
    int number_of_brains_from_second =
            brain->HasAttribute("number_of_brains_from_second") ?
            std::stoi(brain->GetAttribute("number_of_brains_from_second")->GetAsString())
                                                                : 0;
    std::string path_to_first_brains =
            brain->HasAttribute("path_to_first_brains") ?
            brain->GetAttribute("path_to_first_brains")->GetAsString() : "";

    std::vector< CPPNEAT::GeneticEncodingPtr > brains_from_init =
            boost::dynamic_pointer_cast< CPPNEAT::NEATLearner >(learner_)->get_init_brains();
    std::vector< CPPNEAT::GeneticEncodingPtr > brains_from_first;
    if (path_to_first_brains == "" || path_to_first_brains == "none")
    {
      number_of_brains_from_first = 0;
    }
    else
    {
      brains_from_first = boost::dynamic_pointer_cast< CPPNEAT::NEATLearner >(
              learner_)->get_brains_from_yaml(path_to_first_brains, -1);
    }
    std::string path_to_second_brains =
            brain->HasAttribute("path_to_second_brains") ?
            brain->GetAttribute("path_to_second_brains")->GetAsString() : "";
    std::vector< CPPNEAT::GeneticEncodingPtr > brains_from_second;
    if (path_to_second_brains == "" or path_to_second_brains == "none")
    {
      number_of_brains_from_second = 0;
    }
    else
    {
      brains_from_second = boost::dynamic_pointer_cast< CPPNEAT::NEATLearner >(
              learner_)->get_brains_from_yaml(path_to_second_brains, -1);
    }

    std::vector< CPPNEAT::GeneticEncodingPtr > init_brains;
    int cur_number = 0;
    int i = 0;
    while (cur_number < number_of_brains_from_first)
    {
      init_brains.push_back(brains_from_first[i]);
      i++;
      cur_number++;
    }
    i = 0;
    while (cur_number < number_of_brains_from_first + number_of_brains_from_second)
    {
      init_brains.push_back(brains_from_second[i]);
      i++;
      cur_number++;
    }
    i = 0;
    while (cur_number < learn_conf.pop_size)
    {
      init_brains.push_back(brains_from_init[i]);
      i++;
      cur_number++;
    }
    boost::dynamic_pointer_cast< CPPNEAT::NEATLearner >(learner_)->initialise(
            init_brains);

    // initialise evaluator
    evaluator_ = evaluator;
  }

  HyperNEAT_CPG::~HyperNEAT_CPG()
  {
  }

  void
  HyperNEAT_CPG::update(
          const std::vector< rg::MotorPtr > &actuators,
          const std::vector< rg::SensorPtr > &sensors,
          double t,
          double step)
  {
    rb::ConverterSplitBrain< boost::shared_ptr< rb::CPPNConfig >,
                             CPPNEAT::GeneticEncodingPtr >::update(
            Helper::createWrapper(actuators),
            Helper::createWrapper(sensors),
            t,
            step
    );
  }

  CPPNEAT::NEATLearner::LearningConfiguration
  HyperNEAT_CPG::parseLearningSDF(sdf::ElementPtr brain)
  {
    CPPNEAT::NEATLearner::LearningConfiguration config;

    // Read out brain configuration attributes
    config.asexual =
            brain->HasAttribute("asexual") ?
            (brain->GetAttribute("asexual")->GetAsString() == "true") :
            CPPNEAT::NEATLearner::ASEXUAL;
    config.pop_size =
            brain->HasAttribute("pop_size") ?
            std::stoi(brain->GetAttribute("pop_size")->GetAsString()) :
            CPPNEAT::NEATLearner::POP_SIZE;
    config.tournament_size =
            brain->HasAttribute("tournament_size") ?
            std::stoi(brain->GetAttribute("tournament_size")->GetAsString()) :
            CPPNEAT::NEATLearner::TOURNAMENT_SIZE;
    config.num_children =
            brain->HasAttribute("num_children") ?
            std::stoi(brain->GetAttribute("num_children")->GetAsString()) :
            CPPNEAT::NEATLearner::NUM_CHILDREN;
    config.weight_mutation_probability =
            brain->HasAttribute("weight_mutation_probability") ?
            std::stod(brain->GetAttribute("weight_mutation_probability")->GetAsString())
                                                               :
            CPPNEAT::NEATLearner::WEIGHT_MUTATION_PROBABILITY;
    config.weight_mutation_sigma =
            brain->HasAttribute("weight_mutation_sigma") ?
            std::stod(brain->GetAttribute("weight_mutation_sigma")->GetAsString())
                                                         :
            CPPNEAT::NEATLearner::WEIGHT_MUTATION_SIGMA;
    config.param_mutation_probability =
            brain->HasAttribute("param_mutation_probability") ?
            std::stod(brain->GetAttribute("param_mutation_probability")->GetAsString())
                                                              :
            CPPNEAT::NEATLearner::PARAM_MUTATION_PROBABILITY;
    config.param_mutation_sigma =
            brain->HasAttribute("param_mutation_sigma") ?
            std::stod(brain->GetAttribute("param_mutation_sigma")->GetAsString())
                                                        :
            CPPNEAT::NEATLearner::PARAM_MUTATION_SIGMA;
    config.structural_augmentation_probability =
            brain->HasAttribute("structural_augmentation_probability") ?
            std::stod(brain->GetAttribute("structural_augmentation_probability")->GetAsString())
                                                                       :
            CPPNEAT::NEATLearner::STRUCTURAL_AUGMENTATION_PROBABILITY;
    config.structural_removal_probability =
            brain->HasAttribute("structural_removal_probability") ?
            std::stod(brain->GetAttribute("structural_removal_probability")->GetAsString())
                                                                  :
            CPPNEAT::NEATLearner::STRUCTURAL_REMOVAL_PROBABILITY;
    config.max_generations =
            brain->HasAttribute("max_generations") ?
            std::stoi(brain->GetAttribute("max_generations")->GetAsString()) :
            CPPNEAT::NEATLearner::MAX_GENERATIONS;
    config.speciation_threshold =
            brain->HasAttribute("speciation_threshold") ?
            std::stod(brain->GetAttribute("speciation_threshold")->GetAsString())
                                                        :
            CPPNEAT::NEATLearner::SPECIATION_TRESHOLD;
    config.repeat_evaluations =
            brain->HasAttribute("repeat_evaluations") ?
            std::stoi(brain->GetAttribute("repeat_evaluations")->GetAsString())
                                                      :
            CPPNEAT::NEATLearner::REPEAT_EVALUATIONS;
    config.initial_structural_mutations =
            brain->HasAttribute("initial_structural_mutations") ?
            std::stoi(brain->GetAttribute("initial_structural_mutations")->GetAsString())
                                                                :
            CPPNEAT::NEATLearner::INITIAL_STRUCTURAL_MUTATIONS;
    config.interspecies_mate_probability =
            brain->HasAttribute("interspecies_mate_probability") ?
            std::stod(brain->GetAttribute("interspecies_mate_probability")->GetAsString())
                                                                 :
            CPPNEAT::NEATLearner::INTERSPECIES_MATE_PROBABILITY;
    return config;
  }

} /* namespace tol */
