/*
 * RobotController.cpp
 *
 *  Created on: May 9, 2015
 *      Author: elte
 */

#include "RobotController.h"
#include "rlpower.h"
#include "simple_split_brain.h"
#include "extended_neural_network.h"
#include "rlpowered_network.h"
#include "neat_ext_nn.h"

#include <iostream>

namespace tol {

RobotController::RobotController() {}

RobotController::~RobotController() {}

void RobotController::Load(::gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  ::revolve::gazebo::RobotController::Load(_parent, _sdf);
    std::cout << "ToL Robot loaded." << std::endl;
}

void RobotController::LoadBrain(sdf::ElementPtr sdf)
{
    evaluator_ = boost::make_shared<Evaluator>();
    std::cout << sdf->ToString("") <<std::endl;

    if (!sdf->HasElement("rv:brain")) {
        std::cerr << "No robot brain detected, this is probably an error." << std::endl;
        return;
    }
    auto brain = sdf->GetElement("rv:brain");

    if (!brain->HasAttribute("algorithm")) {
        std::cerr << "Brain does not define type, this is probably an error." << std::endl;
        return;
    }
    if (brain->GetAttribute("algorithm")->GetAsString() == "rlpower") {
//         brain_.reset(new tol::RLPower(this->model->GetName(),  brain,evaluator_, motors_, sensors_));
//	brain_.reset(new tol::RLPowerNet(this->model->GetName(),  brain,evaluator_, motors_, sensors_));
// 	brain_.	reset(new tol::ExtNN(this->model->GetName(), evaluator_, brain, motors_, sensors_));
//	brain_.reset(new tol::ExtendedNeuralNetwork(this->model->GetName(), evaluator_, brain, motors_, sensors_));
	brain_.reset(new tol::NeatExtNN(this->model->GetName(), evaluator_, brain, motors_, sensors_));
    } else {
        std::cout << "Calling default ANN brain." << std::endl;
        revolve::gazebo::RobotController::LoadBrain(sdf);
    }
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
        return dS / 30.0; // dS / RLPower::FREQUENCY_RATE
}
void RobotController::Evaluator::updatePosition(const ignition::math::Pose3d pose)
{
    currentPosition_ = pose;
}


} /* namespace tol */
