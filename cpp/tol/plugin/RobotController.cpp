/*
 * RobotController.cpp
 *
 *  Created on: May 9, 2015
 *      Author: elte
 */

#include "RobotController.h"

#include "RLPower_Splines.h"
#include "RLPower_CPPN.h"
#include "HyperNEAT_CPPN.h"
#include "HyperNEAT_Splines.h"
#include "MLMPCPGBrain.h"
#include "GenericLearnerBrain.h"
#include "YamlBodyParser.h"
#include "SUPGBrain.h"
#include "SUPGBrainPhototaxis.h"
#include "brain/supg/SUPGGenomeManager.h"
#include "neat/accneat/src/neat.h"
#include "Helper.h"

#include <iostream>
#include <exception>
#include <cmath>
#include <revolve/gazebo/motors/Motor.h>
#include <revolve/gazebo/sensors/VirtualSensor.h>
#include <brain/learner/HyperAccNEATLearner_CPGController.h>
#include <neat/AsyncNEAT.h>

namespace tol {

const char* getVARenv(const char* var_name)
{
  const char* env_p = std::getenv(var_name);
  if(env_p) {
    std::cout << "ENV " << var_name << " is: " << env_p << std::endl;
  } else {
    std::cout << "ENV " << var_name << " not found, using default value: ";
  }
  return env_p;
}

const NEAT::GeneticSearchType getGeneticSearchType(const std::string &value) {
  if (value.compare("PHASED") == 0)
    return NEAT::GeneticSearchType::PHASED;

  if (value.compare("BLENDED") == 0)
    return NEAT::GeneticSearchType::BLENDED;

  if (value.compare("COMPLEXIFY") == 0)
    return NEAT::GeneticSearchType::COMPLEXIFY;

  //default value
  return NEAT::GeneticSearchType::PHASED;
}

const char* getGeneticSearchType(const NEAT::GeneticSearchType value) {
  switch (value) {
    case NEAT::GeneticSearchType::BLENDED:
      return "NEAT::GeneticSearchType::BLENDED";
    case NEAT::GeneticSearchType::PHASED:
      return "NEAT::GeneticSearchType::PHASED";
    case NEAT::GeneticSearchType::COMPLEXIFY:
      return "NEAT::GeneticSearchType::COMPLEXIFY";
    default:
      return "undefined";
  }
}

RobotController::RobotController() {
  AsyncNeat::Init(
      std::unique_ptr<NEAT::GenomeManager>(
          new SUPGGenomeManager()
      ));
  unsigned int populationSize = 10;
  NEAT::real_t mutate_add_node_prob = 0.01;
  NEAT::real_t mutate_add_link_prob = 0.3;
  NEAT::GeneticSearchType geneticSearchType = NEAT::GeneticSearchType::PHASED;

  if(const char* env_p = getVARenv("NEAT_POP_SIZE")) {
    try {
      populationSize = (unsigned int) std::stoul(env_p);
    } catch (const std::invalid_argument &e) {
      std::cout << "ERROR DECODING STRING \"NEAT_POP_SIZE\" to unsigned long:"
                << " using default value " << populationSize << " instead" << std::endl;
    }

  } else {
    std::cout << populationSize << std::endl;
  }

  if(const char* env_p = getVARenv("NEAT_MUTATE_ADD_NODE_PROB")) {
    try {
      mutate_add_node_prob = (float) std::stod(env_p);
    } catch (const std::invalid_argument &e) {
      std::cout << "ERROR DECODING STRING \"NEAT_MUTATE_ADD_NODE_PROB\" to double:"
                << " using default value " << mutate_add_node_prob << " instead" << std::endl;
    }
  } else {
    std::cout << mutate_add_node_prob << std::endl;
  }

  if(const char* env_p = getVARenv("NEAT_MUTATE_ADD_LINK_PROB")) {
    try {
      mutate_add_link_prob = (float) std::stod(env_p);
    } catch (const std::invalid_argument &e) {
      std::cout << "ERROR DECODING STRING \"NEAT_MUTATE_ADD_LINK_PROB\" to double:"
                << " using default value " << mutate_add_link_prob << " instead" << std::endl;
    }
  } else {
    std::cout << mutate_add_link_prob << std::endl;
  }

  if(const char* env_p = getVARenv("NEAT_SEARCH_TYPE")) {
    geneticSearchType = getGeneticSearchType(env_p);
  } else {
    std::cout << getGeneticSearchType(geneticSearchType) << std::endl;
  }

  AsyncNeat::SetPopulationSize(populationSize); // 10 - 25 - 50 - 75 - 100 - 1000
  AsyncNeat::SetMutateAddNodeProb(mutate_add_node_prob);
  AsyncNeat::SetMutateAddLinkProb(mutate_add_link_prob);
  std::cout << "Setting up genetic search type to: " << getGeneticSearchType(geneticSearchType) << std::endl;
  AsyncNeat::SetSearchType(geneticSearchType);
}

RobotController::~RobotController() {
  AsyncNeat::CleanUp();
}

void
RobotController::Load(::gazebo::physics::ModelPtr _parent,
                      sdf::ElementPtr _sdf)
{
  ::revolve::gazebo::RobotController::Load(_parent,
                                           _sdf);
  std::cout << "ToL Robot loaded." << std::endl;
}

void
RobotController::LoadBrain(sdf::ElementPtr sdf)
{
  try {
    evaluator_ = boost::make_shared<Evaluator>();

    unsigned int motor_n = 0; //motors_.size();
    for (const auto &motor : motors_)
      motor_n += motor->outputs();
    unsigned int sensor_n = 0; //sensors_.size();
    for (const auto &sensor : sensors_)
      sensor_n += sensor->inputs();

    if (!sdf->HasElement("rv:brain")) {
      std::cerr << "No robot brain detected, this is probably an error." << std::endl;
      return;
    }
    auto brain = sdf->GetElement("rv:brain");

    if (!brain->HasAttribute("algorithm")) {
      std::cerr << "Brain does not define type, this is probably an error." << std::endl;
      return;
    }

    if (!brain->HasAttribute("algorithm")) {
      std::cerr << "Brain does not define type, this is probably an error." << std::endl;
      return;
    }
    if (brain->GetAttribute("algorithm")->GetAsString() == "rlpower::spline") {
      brain_.reset(new tol::RLPowerSplines(this->model->GetName(),
                                           brain,
                                           evaluator_,
                                           motors_,
                                           sensors_));
    } else if (brain->GetAttribute("algorithm")->GetAsString() == "rlpower::net") {
      brain_.reset(new tol::RLPowerNet(this->model->GetName(),
                                       brain,
                                       evaluator_,
                                       motors_,
                                       sensors_));
    } else if (brain->GetAttribute("algorithm")->GetAsString() == "hyperneat::net") {
      brain_.reset(new tol::HyperExtNN(this->model->GetName(),
                                       brain,
                                       evaluator_,
                                       motors_,
                                       sensors_));
    } else if (brain->GetAttribute("algorithm")->GetAsString() == "hyperneat::spline") {
      brain_.reset(new tol::HyperSplines(this->model->GetName(),
                                         brain,
                                         evaluator_,
                                         motors_,
                                         sensors_));
    } else if (brain->GetAttribute("algorithm")->GetAsString() == "rlpower::mlmp_cpg") {
      brain_.reset(new tol::MlmpCPGBrain(this->model->GetName(),
                                         evaluator_,
                                         motor_n,
                                         sensor_n));
    } else if (brain->GetAttribute("algorithm")->GetAsString() == "hyperneat::mlmp_cpg") {

//      this->model->GetName()
      tol::YamlBodyParser* parser = new tol::YamlBodyParser();
      parser->parseFile("./res/robots/spider9.yaml");
      std::vector<std::vector<bool>> connections = parser->connections();
      std::vector<std::vector<float>> cpgs_coordinates = parser->coordinates();
      brain_.reset(new tol::GenericLearnerBrain(
          new revolve::brain::HyperAccNEATLearner_CPGController(
              evaluator_,
              sensor_n,
              motor_n,
              2, // coordinates cardinality
              connections,
              cpgs_coordinates,
              30, //seconds
              -1 // infinite evaluations
          )
      ));
    } else if (brain->GetAttribute("algorithm")->GetAsString() == "supg::phototaxis") {
      std::vector<std::vector<float> > coordinates;

      const std::string robot_type_str = getVARenv("ROBOT_TYPE");
      const Helper::RobotType robot_type = Helper::parseRobotType(robot_type_str);
      std::cout << "Loading SUPG configuration for robot " << robot_type << std::endl;
      switch (robot_type) {
        case Helper::spider9:
          // SPIDER 9
          //     #
          //     #
          // # # O # #
          //     #
          //     #
          coordinates = std::vector<std::vector<float> >
              ({
                   // Leg00Joint Leg01Joint
                   {1,    0},//    1},
                   {.5,   0},//    -1},
                   // Leg10Joint Leg11Joint
                   {-1,   0},//    1},
                   {-.5f, 0},//    -1},
                   // Leg20Joint Leg21Joint
                   {0,    1},//    1},
                   {0,    .5},//   -1},
                   // Leg30Joint Leg31Joint
                   {0,    -1},//   1},
                   {0,    -.5f}//, -1}
               });
          break;
        case Helper::spider13:
          // SPIDER 13
          //       #
          //       #
          //       #
          // # # # O # # #
          //       #
          //       #
          //       #
          coordinates = std::vector<std::vector<float> >
              ({
                   // Leg00Joint Leg01Joint Leg02Joint
                   {.333,   0},
                   {.666,   0},
                   {1,      0},
                   // Leg10Joint Leg11Joint Leg12Joint
                   {-.333f, 0},
                   {-.666f, 0},
                   {-1,     0},
                   // Leg20Joint Leg21Joint Leg22Joint
                   {0,      .333},
                   {0,      .666},
                   {0,      1},
                   // Leg30Joint Leg31Joint Leg32Joint
                   {0,      -.333f},
                   {0,      -.666f},
                   {0,      -1},
               });
          break;
        case Helper::spider17:
          // SPIDER 17
          //         #
          //         #
          //         #
          //         #
          // # # # # O # # # #
          //         #
          //         #
          //         #
          //         #
          coordinates = std::vector<std::vector<float> >
              ({
                   // Leg00Joint Leg01Joint Leg02Joint
                   {.25,   0},
                   {.5,    0},
                   {.75,   0},
                   {1,     0},
                   // Leg10Joint Leg11Joint Leg12Joint
                   {-.25f, 0},
                   {-.5f,  0},
                   {-.75f, 0},
                   {-1,    0},
                   // Leg20Joint Leg21Joint Leg22Joint
                   {0,     .25},
                   {0,     .5},
                   {0,     .75},
                   {0,     1},
                   // Leg30Joint Leg31Joint Leg32Joint
                   {0,     -.25f},
                   {0,     -.5f},
                   {0,     -.75f},
                   {0,     -1},
               });
          break;
        case Helper::gecko7:
          // GECKO 5
          // #   #
          // O # #
          // #   #
          coordinates = std::vector<std::vector<float> >
              ({
                   // Leg00Joint
                   {-1,   +1},
                   // Leg01Joint
                   {-1,   -1},
                   // BodyJoint0
                   {-.5f, 0},
                   // BodyJoint1
                   {+.5f, 0},
                   // Leg10Joint
                   {+1,   +1},
                   // Leg11Joint
                   {+1,   -1},
               });
          break;
        case Helper::gecko12:

          // GECKO 12
          // #     #
          // #     #
          // O # # #
          // #     #
          // #     #
          coordinates = std::vector<std::vector<float> >
              ({
                   // Leg00Joint Leg001Joint
                   {-1.0f, +0.5f},
                   {-1,    +1},
                   // Leg01Joint Leg011Joint
                   {-1.0f, -0.5f},
                   {-1,    -1},
                   // BodyJoint0 BodyJoint1 BodyJoint2
                   {-.5f,  0},
                   {0,     0},
                   {+.5f,  0},
                   // Leg10Joint Leg101Joint
                   {+1,    +0.5f},
                   {+1,    +1},
                   // Leg11Joint Leg111Joint
                   {+1,    -0.5f},
                   {+1,    -1},
               });
          break;
        case Helper::gecko17:
          // GECKO 17
          // #     #
          // #     #
          // O # # #
          // #     #
          // #     #
          coordinates = std::vector<std::vector<float> >
              ({
                   // Leg00Joint Leg001Joint Leg002Joint
                   {-1.0f,  +.333f},
                   {-1.0f,  +.666f},
                   {-1,     +1},
                   // Leg01Joint Leg011Joint Leg012Joint
                   {-1.0f,  -.333f},
                   {-1.0f,  -.333f},
                   {-1,     -1},
                   // BodyJoint0 BodyJoint1 BodyJoint2 BodyJoint3
                   {-.666f, 0},
                   {-.333f, 0},
                   {+.333f, 0},
                   {+.666f, 0},
                   // Leg10Joint Leg101Joint Leg102Joint
                   {+1,     +.333f},
                   {+1,     +.666f},
                   {+1,     +1},
                   // Leg11Joint Leg111Joint Leg112Joint
                   {+1,     -.333f},
                   {+1,     -.666f},
                   {+1,     -1},
               });
          break;
        case Helper::snake5:


          // SNAKE 5
          //
          // # # O # #
          //
          coordinates = std::vector<std::vector<float> >
              ({
                   // Leg00Joint
                   {-.5f, 0},
                   // Leg01Joint
                   {-1,   0},
                   // Leg10Joint
                   {+.5f, 0},
                   // Leg11Joint
                   {+1,   0},
               });
          break;
        case Helper::snake7:
          // SNAKE 7
          //
          // # # # O # # #
          //
          coordinates = std::vector<std::vector<float> >
              ({
                   // Leg00Joint
                   {-.333f, 0},
                   // Leg01Joint
                   {-.666f, 0},
                   // Leg02Joint
                   {-1,     0},
                   // Leg10Joint
                   {+.333f, 0},
                   // Leg11Joint
                   {+.666f, 0},
                   // Leg12Joint
                   {+1,     0},
               });
          break;
        case Helper::snake9:
          // SNAKE 9
          //
          // # # # # O # # # #
          //
          coordinates = std::vector<std::vector<float> >
              ({
                   // Leg00Joint
                   {-.25f, 0},
                   // Leg01Joint
                   {-.50f, 0},
                   // Leg02Joint
                   {-.75f, 0},
                   // Leg03Joint
                   {-1,    0},
                   // Leg10Joint
                   {+.25f, 0},
                   // Leg11Joint
                   {+.50f, 0},
                   // Leg12Joint
                   {+.75f, 0},
                   // Leg13Joint
                   {+1,    0},
               });
          break;
        case Helper::babyA:

          // BABY 1
          // #
          // #   #
          // O # #
          // #   #
          coordinates = std::vector<std::vector<float> >
              ({
                   // Leg00Joint
                   {-1.0f,   +1},
                   // Leg01Joint
                   {-1.0f,  -.3f},
                   // Leg011Joint
                   {-1.0f,  -.6f},
                   // Leg021Joint
                   {-1.0f,   -1.0f},
                   // BodyJoint0
                   {-.5f, 0},
                   // BodyJoint1
                   {+.5f, 0},
                   // Leg10Joint
                   {+1,   +1},
                   // Leg11Joint
                   {+1,   -1},
               });
          break;
        case Helper::babyB:
          // BABY 2
          //
          //       #
          // # # # O # # #
          //       #
          //       #
          //       #
          coordinates = std::vector<std::vector<float> >
              ({
                   // Leg00Joint Leg01Joint Leg02Joint
                   {1,     0},
                   {.666f, 0},
                   {.333f, 0},
                   // Leg10Joint
                   {-1,    0},
                   // Leg20Joint Leg21Joint Leg22Joint
                   {0,     1},
                   {0,     .666f},
                   {0,     .333f},
                   // Leg30Joint Leg31Joint Leg32Joint
                   {0,     -1},
                   {0,     -.666f},
                   {0,     .333f},
               });
          break;
        case Helper::babyC:
          // BABY 3
          // #       #
          // #       x
          // #       #
          // O # # # #
          // #       #
          // #       #
          // #       #
          coordinates = std::vector<std::vector<float> >
              ({
                   // Leg00Joint Leg001Joint Leg002Joint
                   {-1.0f,  +.333f},
                   {-1.0f,  +.666f},
                   {-1.0f,  +1},
                   // Leg01Joint Leg011Joint Leg012Joint
                   {-1.0f,  -.333f},
                   {-1.0f,  -.333f},
                   {-1.0f,  -1},
                   // BodyJoint0 BodyJoint1 BodyJoint2 BodyJoint3
                   {-.666f, 0},
                   {-.333f, 0},
                   {+.333f, 0},
                   {+.666f, 0},
                   // Leg10Joint Leg101Joint Leg102Joint
                   {+1.0f,  +.333f},
                   {+1.0f,  +.666f},
                   {+1.0f,  +1},
                   // Leg11Joint Leg111Joint Leg112Joint
                   {+1.0f,  -.333f},
                   {+1.0f,  -.666f},
                   {+1.0f,  -.9f},
               });
          break;
      }


//     brain_.reset(new SUPGBrain(evaluator_, coordinates, motors_, sensors_));
      brain_.reset(new SUPGBrainPhototaxis(evaluator_,
                                           50,
                                           coordinates,
                                           motors_,
                                           sensors_));

    } else {
      std::cout << "Calling default ANN brain." << std::endl;
      revolve::gazebo::RobotController::LoadBrain(sdf);
    }
  } catch (std::exception &e) {
    // needed because otherwise the exception dies silently and debugging is a nightmare
    std::cerr << "Exception occurred while running RobotController::LoadBrain:\n"
              << "exception: " << e.what() << std::endl;
  }
}

void
RobotController::DoUpdate(const gazebo::common::UpdateInfo info)
{
  revolve::gazebo::RobotController::DoUpdate(info);
  evaluator_->updatePosition(this->model->GetRelativePose().Ign());
    auto pose = this->model->GetRelativePose().Ign();
    evaluator_->updatePosition(pose);
    reinterpret_cast<SUPGBrainPhototaxis&>(*brain_).updateRobotPosition(pose);
}

// EVALUATOR CODE
RobotController::Evaluator::Evaluator()
{
  currentPosition_.Reset();
  previousPosition_.Reset();
}

void
RobotController::Evaluator::start()
{
  previousPosition_ = currentPosition_;
}

double
RobotController::Evaluator::fitness()
{
  double dS = sqrt(
          std::pow(previousPosition_.Pos().X() - currentPosition_.Pos().X(),
              2) +
          std::pow(previousPosition_.Pos().Y() - currentPosition_.Pos().Y(),
              2));
  previousPosition_ = currentPosition_;
  return dS / 30.0; // dS / RLPower::FREQUENCY_RATE
}

void
RobotController::Evaluator::updatePosition(const ignition::math::Pose3d pose)
{
  currentPosition_ = pose;
}


} /* namespace tol */
