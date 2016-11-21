/*
 * RobotController.cpp
 *
 *  Created on: May 9, 2015
 *      Author: elte
 */

#include "RobotController.h"
#include "rlpower.h"
#include "supgbrain.h"
#include "supgbrainphototaxis.h"
#include "brain/supg/supggenomemanager.h"
#include "neat/accneat/src/neat.h"

#include <iostream>
#include <cstdlib>
#include <string>

#include <boost/make_shared.hpp>

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
    unsigned long populationSize = 10;
    NEAT::GeneticSearchType geneticSearchType = NEAT::GeneticSearchType::PHASED;

    if(const char* env_p = getVARenv("NEAT_POP_SIZE")) {
        //TODO catch exception
        populationSize = std::stol(env_p);
    } else {
        std::cout << populationSize << std::endl;
    }

    if(const char* env_p = getVARenv("NEAT_SEARCH_TYPE")) {
        geneticSearchType = getGeneticSearchType(env_p);
    } else {
        std::cout << getGeneticSearchType(geneticSearchType) << std::endl;
    }

    AsyncNeat::SetPopulationSize(populationSize); // 10 - 25 - 50 - 75 - 100 - 1000
    std::cout << "Setting up genetic search type to: " << getGeneticSearchType(geneticSearchType) << std::endl;
    AsyncNeat::SetSearchType(geneticSearchType);
}

RobotController::~RobotController() {
    AsyncNeat::CleanUp();
}

void RobotController::Load(::gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    ::revolve::gazebo::RobotController::Load(_parent, _sdf);
    std::cout << "ToL Robot loaded." << std::endl;
}

void RobotController::LoadBrain(sdf::ElementPtr sdf)
{
    //revolve::gazebo::RobotController::LoadBrain(sdf);
    evaluator_ = boost::make_shared<Evaluator>();

/* RLPOWER brian */
//     brain_.reset(new tol::RLPower(this->model->GetName(), evaluator_, motors_, sensors_));

/*  SUPG brain   */
    // joints 00 and 10 are opposites, therefore on the same axis

    // SPIDER 9
    //     #
    //     #
    // # # O # #
    //     #
    //     #
    std::vector< std::vector< float> > coordinates
    ( {
      // Leg00Joint Leg01Joint
         {  1,  0, 1}, { .5,  0, -1},
      // Leg10Joint Leg11Joint
         { -1,  0, 1}, {-.5,  0, -1},
      // Leg20Joint Leg21Joint
         {  0,  1, 1}, {  0, .5, -1},
      // Leg30Joint Leg31Joint
         {  0, -1, 1}, {  0,-.5, -1}
    } );

    // SPIDER 13
    //       #
    //       #
    //       #
    // # # # O # # #
    //       #
    //       #
    //       #
//     std::vector< std::vector< float> > coordinates
//     ( {
//       // Leg00Joint Leg01Joint Leg02Joint
//          {  .333,  0}, {  .666,  0}, {  1,  0},
//       // Leg10Joint Leg11Joint Leg12Joint
//          { -.333,  0}, { -.666,  0}, { -1,  0},
//       // Leg20Joint Leg21Joint Leg22Joint
//          {  0,  .333}, {  0,  .666}, {  0,  1},
//       // Leg30Joint Leg31Joint Leg32Joint
//          {  0, -.333}, {  0, -.666}, {  0, -1},
//     } );

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
//     std::vector< std::vector< float> > coordinates
//     ( {
//       // Leg00Joint Leg01Joint Leg02Joint
//          {  .25,  0}, {  .5,  0}, {  .75,  0}, {  1,  0},
//       // Leg10Joint Leg11Joint Leg12Joint
//          { -.25,  0}, { -.5,  0}, { -.75,  0}, { -1,  0},
//       // Leg20Joint Leg21Joint Leg22Joint
//          {  0,  .25}, {  0,  .5}, {  0,  .75}, {  0,  1},
//       // Leg30Joint Leg31Joint Leg32Joint
//          {  0, -.25}, {  0, -.5}, {  0, -.75}, {  0, -1},
//     } );

    // GECKO 5
    // #   #
    // O # #
    // #   #
//     std::vector< std::vector< float> > coordinates
//     ( {
//       // Leg00Joint
//          { -1, +1},
//       // Leg01Joint
//          { -1, -1},
//       // BodyJoint0
//          { -.5, 0},
//       // BodyJoint1
//          { +.5, 0},
//       // Leg10Joint
//          { +1, +1},
//       // Leg11Joint
//          { +1, -1},
//     } );

    // GECKO 12
    // #     #
    // #     #
    // O # # #
    // #     #
    // #     #
//     std::vector< std::vector< float> > coordinates
//     ( {
//       // Leg00Joint Leg001Joint
//          { -1, +0.5}, { -1, +1},
//       // Leg01Joint Leg011Joint
//          { -1, -0.5}, { -1, -1},
//       // BodyJoint0 BodyJoint1 BodyJoint2
//          { -.5, 0},
//          {   0, 0},
//          { +.5, 0},
//       // Leg10Joint Leg101Joint
//          { +1, +0.5}, { +1, +1},
//       // Leg11Joint Leg111Joint
//          { +1, -0.5}, { +1, -1},
//     } );

    // GECKO 17
    // #     #
    // #     #
    // O # # #
    // #     #
    // #     #
//     std::vector< std::vector< float> > coordinates
//     ( {
//       // Leg00Joint Leg001Joint Leg002Joint
//          { -1, +.333}, { -1, +.666}, { -1, +1},
//       // Leg01Joint Leg011Joint Leg012Joint
//          { -1, -.333}, { -1, -.333}, { -1, -1},
//       // BodyJoint0 BodyJoint1 BodyJoint2 BodyJoint3
//          { -.666, 0},
//          { -.333, 0},
//          { +.333, 0},
//          { +.666, 0},
//       // Leg10Joint Leg101Joint Leg102Joint
//          { +1, +.333}, { +1, +.666}, { +1, +1},
//       // Leg11Joint Leg111Joint Leg112Joint
//          { +1, -.333}, { +1, -.666}, { +1, -1},
//     } );

    // SNAKE 5
    //
    // # # O # #
    //
//     std::vector< std::vector< float> > coordinates
//     ( {
//       // Leg00Joint
//          { -.5, 0},
//       // Leg01Joint
//          { -1,  0},
//       // Leg10Joint
//          { +.5, 0},
//       // Leg11Joint
//          { +1,  0},
//     } );

    // SNAKE 7
    //
    // # # # O # # #
    //
//     std::vector< std::vector< float> > coordinates
//     ( {
//       // Leg00Joint
//          { -.333, 0},
//       // Leg01Joint
//          { -.666, 0},
//       // Leg02Joint
//          {    -1, 0},
//       // Leg10Joint
//          { +.333, 0},
//       // Leg11Joint
//          { +.666, 0},
//       // Leg12Joint
//          {    +1, 0},
//     } );

    // SNAKE 9
    //
    // # # # # O # # # #
    //
//     std::vector< std::vector< float> > coordinates
//     ( {
//       // Leg00Joint
//          { -.25, 0},
//       // Leg01Joint
//          { -.50, 0},
//       // Leg02Joint
//          { -.75, 0},
//       // Leg03Joint
//          {   -1, 0},
//       // Leg10Joint
//          { +.25, 0},
//       // Leg11Joint
//          { +.50, 0},
//       // Leg12Joint
//          { +.75, 0},
//       // Leg13Joint
//          {   +1, 0},
//     } );

    // BABY 1
    // #
    // #   #
    // O # #
    // #   #
//     std::vector< std::vector< float> > coordinates
//     ( {
//       // Leg00Joint
//          { -1, +1},
//       // Leg01Joint
//          { -1, -.3},
//       // Leg011Joint
//          { -1, -.6},
//       // Leg021Joint
//          { -1, -1},
//       // BodyJoint0
//          { -.5, 0},
//       // BodyJoint1
//          { +.5, 0},
//       // Leg10Joint
//          { +1, +1},
//       // Leg11Joint
//          { +1, -1},
//     } );

    // BABY 2
    //
    //       #
    // # # # O # # #
    //       #
    //       #
    //       #
//     std::vector< std::vector< float> > coordinates
//     ( {
//       // Leg00Joint Leg01Joint Leg02Joint
//          {  1,  0}, { .666,  0}, { .333,  0},
//       // Leg10Joint
//          { -1,  0},
//       // Leg20Joint Leg21Joint Leg22Joint
//          {  0,  1}, {  0, .666}, { 0,  .333},
//       // Leg30Joint Leg31Joint Leg32Joint
//          {  0, -1}, {  0,-.666}, { 0,  .333},
//     } );

    // BABY 3
    // #       #
    // #       x
    // #       #
    // O # # # #
    // #       #
    // #       #
    // #       #
//     std::vector< std::vector<float> > coordinates
//     ( {
//       // Leg00Joint Leg001Joint Leg002Joint
//          { -1, +.333}, { -1, +.666}, { -1, +1},
//       // Leg01Joint Leg011Joint Leg012Joint
//          { -1, -.333}, { -1, -.333}, { -1, -1},
//       // BodyJoint0 BodyJoint1 BodyJoint2 BodyJoint3
//          { -.666, 0},
//          { -.333, 0},
//          { +.333, 0},
//          { +.666, 0},
//       // Leg10Joint Leg101Joint Leg102Joint
//          { +1, +.333}, { +1, +.666}, { +1, +1},
//       // Leg11Joint Leg111Joint Leg112Joint
//          { +1, -.333}, { +1, -.666}, { +1, -.9},
//     } );

//     brain_.reset(new SUPGBrain(evaluator_, coordinates, motors_, sensors_));
    brain_.reset(new SUPGBrainPhototaxis(evaluator_,
                                         50,
                                         coordinates,
                                         motors_,
                                         sensors_));

//    if (!sdf->HasElement("rv:brain")) {
//        std::cerr << "No robot brain detected, this is probably an error." << std::endl;
//        return;
//    }
//    auto brain = sdf->GetElement("rv:brain");
//
//    if (!brain->HasAttribute("algorithm")) {
//        std::cerr << "Brain does not define type, this is probably an error." << std::endl;
//        return;
//    }
//
//    if (brain->GetAttribute("algorithm")->GetAsString() == "rlpower") {
//        brain_.reset(new tol::RLPower(this->model->GetName(), brain, evaluator_, motors_, sensors_));
//    } else {
//        std::cout << "Calling default ANN brain." << std::endl;
//        revolve::gazebo::RobotController::LoadBrain(sdf);
//    }
}

void RobotController::DoUpdate(const gazebo::common::UpdateInfo info)
{
    revolve::gazebo::RobotController::DoUpdate(info);
    auto pose = this->model->GetRelativePose().Ign();
    evaluator_->updatePosition(pose);
    reinterpret_cast<SUPGBrainPhototaxis&>(*brain_).updateLightPosition(pose);
}

// EVALUATOR CODE
RobotController::Evaluator::Evaluator()
{
    currentPosition_.Reset();
    previousPosition_.Reset();
}

void RobotController::Evaluator::start()
{
    previousPosition_ = currentPosition_;
}

double RobotController::Evaluator::fitness()
{
        double dS = sqrt(
                pow(previousPosition_.Pos().X() - currentPosition_.Pos().X(), 2) +
                pow(previousPosition_.Pos().Y() - currentPosition_.Pos().Y(), 2));
        previousPosition_ = currentPosition_;
        return dS / 30.0; // dS / RLPower::FREQUENCY_RATE
}
void RobotController::Evaluator::updatePosition(const ignition::math::Pose3d pose)
{
    currentPosition_ = pose;
}


} /* namespace tol */
