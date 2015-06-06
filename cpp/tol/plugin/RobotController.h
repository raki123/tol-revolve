/*
 * RobotController.h
 *
 *  Created on: May 9, 2015
 *      Author: elte
 */

#ifndef TOL_ROBOTCONTROLLER_H_
#define TOL_ROBOTCONTROLLER_H_

#include <revolve/gazebo/plugin/RobotController.h>

namespace tol {

class RobotController: public revolve::gazebo::RobotController {
public:
	RobotController();
	virtual ~RobotController();
};

} /* namespace tol */

GZ_REGISTER_MODEL_PLUGIN(tol::RobotController)

#endif /* TOL_ROBOTCONTROLLER_H_ */
