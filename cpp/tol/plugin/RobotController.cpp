/*
 * RobotController.cpp
 *
 *  Created on: May 9, 2015
 *      Author: elte
 */

#include "RobotController.h"
#include "rlpower.h"
#include "mlmpcpgbrain.h"

#include <iostream>
#include <exception>
#include <revolve/gazebo/motors/Motor.h>
#include <revolve/gazebo/sensors/VirtualSensor.h>

namespace tol {

RobotController::RobotController() {}

RobotController::~RobotController() {}

void RobotController::Load(::gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    ::revolve::gazebo::RobotController::Load(_parent, _sdf);
    std::cout << "ToL Robot loaded." << std::endl;
}

void RobotController::LoadBrain(sdf::ElementPtr sdf)
{
    try {
        evaluator_ = boost::make_shared<Evaluator>();

//         if (!sdf->HasElement("rv:brain")) {
//             std::cerr << "No robot brain detected, this is probably an error." << std::endl;
//             return;
//         }
//         auto brain = sdf->GetElement("rv:brain");
//
//         if (!brain->HasAttribute("algorithm")) {
//             std::cerr << "Brain does not define type, this is probably an error." << std::endl;
//             return;
//         }

        unsigned int motor_n = 0; //motors_.size();
        for (const auto &motor : motors_)
            motor_n += motor->outputs();
        unsigned int sensor_n = 0; //sensors_.size();
        for (const auto &sensor : sensors_)
            sensor_n += sensor->inputs();

//         std::string algorithm = brain->GetAttribute("algorithm")->GetAsString();
//         if ( algorithm == "rlpower") {
//             brain_.reset(new tol::RLPower(this->model->GetName(),
//                                         brain,
//                                         evaluator_,
//                                         motor_n,
//                                         sensor_n));
//
//         } else { if ( algorithm == "mlmp-cpg") {
            std::cerr << "###########################Using MlmpCPGBrain" << std::endl;

            brain_.reset(new tol::MlmpCPGBrain(this->model->GetName(),
                                            evaluator_,
                                            motor_n,
                                            sensor_n));
//         } else {
//             std::cout << "Calling default ANN brain." << std::endl;
//             revolve::gazebo::RobotController::LoadBrain(sdf);
//         }
    } catch (std::exception &e) {
        std::cerr << "Exception occurred while running RobotController::LoadBrain:\n"
                  << "exception: " << e.what() << std::endl;
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
