/*
 * RobotController.cpp
 *
 *  Created on: May 9, 2015
 *      Author: elte
 */

#include "RobotController.h"
#include "rlpower.h"
#include "neat/accneat/src/neat.h"

#include <iostream>
#include <cstdlib>
#include <string>

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
    AsyncNeat::Init();
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
    evaluator_ = std::make_shared<Evaluator>();

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
//     std::vector< std::vector< float> > coordinates
//     ( {
//       // Leg00Joint Leg01Joint
//          {  1,  0}, { .5,  0},
//       // Leg10Joint Leg11Joint
//          { -1,  0}, {-.5,  0},
//       // Leg20Joint Leg21Joint
//          {  0,  1}, {  0, .5},
//       // Leg30Joint Leg31Joint
//          {  0, -1}, {  0,-.5}
//     } );

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
    std::vector< std::vector< float> > coordinates
    ( {
      // Leg00Joint Leg01Joint Leg02Joint
         {  .25,  0}, {  .5,  0}, {  .75,  0}, {  1,  0},
      // Leg10Joint Leg11Joint Leg12Joint
         { -.25,  0}, { -.5,  0}, { -.75,  0}, { -1,  0},
      // Leg20Joint Leg21Joint Leg22Joint
         {  0,  .25}, {  0,  .5}, {  0,  .75}, {  0,  1},
      // Leg30Joint Leg31Joint Leg32Joint
         {  0, -.25}, {  0, -.5}, {  0, -.75}, {  0, -1},
    } );

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


    brain_.reset(new SUPGBrain(evaluator_, coordinates, motors_, sensors_));

}

void RobotController::DoUpdate(const gazebo::common::UpdateInfo info)
{
    revolve::gazebo::RobotController::DoUpdate(info);
    evaluator_->updatePosition(this->model->GetRelativePose().Ign());
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
        return dS;
}
void RobotController::Evaluator::updatePosition(const ignition::math::Pose3d pose)
{
    currentPosition_ = pose;
}


} /* namespace tol */
