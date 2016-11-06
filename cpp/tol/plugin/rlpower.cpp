//
// Created by Milan Jelisavcic on 28/03/16.
//

#include "rlpower.h"
#include "sensor.h"
#include "actuator.h"
#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include <algorithm>
#include <stdexcept>
#include <cstdlib>
#include <map>
#include <string>
#include <sstream>
#include <cmath>

#include <random>
#include <iostream>
#include <fstream>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>

namespace tol {

    RLPower::RLPower(std::string modelName,
                     sdf::ElementPtr brain,
                     EvaluatorPtr evaluator,
                     std::vector<revolve::gazebo::MotorPtr> &actuators,
                     std::vector<revolve::gazebo::SensorPtr> &sensors) :
            revolve::brain::RLPower(
                    modelName,
                    brain,
                    evaluator,
                    actuators.size(),
                    sensors.size()
            ) { }

    RLPower::~RLPower() { }

    void RLPower::update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                         const std::vector<revolve::gazebo::SensorPtr> &sensors,
                         double t,
                         double step) {
        revolve::brain::RLPower::update(
                Helper::createWrapper(actuators),
                Helper::createWrapper(sensors),
                t, step
        );
    }

} /* namespace tol */
