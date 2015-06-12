/*
 * RobotController.cpp
 *
 *  Created on: May 9, 2015
 *      Author: elte
 */

#include "RobotController.h"
#include <iostream>

namespace tol {

RobotController::RobotController() {}

RobotController::~RobotController() {}

void RobotController::Load(::gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
	std::cout << "ToL Robot loaded." << std::endl;
	::revolve::gazebo::RobotController::Load(_parent, _sdf);
}
} /* namespace tol */
