//
// Created by Milan Jelisavcic on 28/03/16.
//

#include "rlpower.h"
#include "sensor.h"
#include "actuator.h"
#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"
#include "helper.h"

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

#include <boost/make_shared.hpp>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>

using namespace tol;


RLPower::RLPower(std::string modelName,
                 tol::EvaluatorPtr evaluator,
                 std::vector< revolve::gazebo::MotorPtr >& actuators,
                 std::vector< revolve::gazebo::SensorPtr >& sensors)
    : revolve::brain::RLPower(
        evaluator,
        actuators.size(),
        sensors.size())
{

}

RLPower::~RLPower() {
    // `boost::shared_ptr< Policy >` should take care of memory management for us
}

void RLPower::update(const std::vector <revolve::gazebo::MotorPtr> &actuators,
                        const std::vector <revolve::gazebo::SensorPtr> &sensors, double t, double step)
{
    revolve::brain::RLPower::update(
        Helper::createWrapper(actuators),
        Helper::createWrapper(sensors),
        t, step
    );
}
