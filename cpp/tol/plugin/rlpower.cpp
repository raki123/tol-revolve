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

using namespace tol;

const std::vector< revolve::brain::ActuatorPtr > createWrapper(const std::vector < revolve::gazebo::MotorPtr > original)
{
    std::vector< revolve::brain::ActuatorPtr > result;
    for (int i=0; i<original.size(); i++) {
        result.push_back(std::make_shared<tol::Actuator>(tol::Actuator(original[i])));
    }

    return result;
}

const std::vector< revolve::brain::SensorPtr > createWrapper(const std::vector < revolve::gazebo::SensorPtr > original)
{
    std::vector< revolve::brain::SensorPtr > result;
    for (int i=0; i<original.size(); i++) {
        result.push_back(std::make_shared<tol::Sensor>(tol::Sensor(original[i])));
    }

    return result;
}

RLPower::RLPower(std::string modelName, tol::EvaluatorPtr evaluator, std::vector< revolve::gazebo::MotorPtr >& actuators, std::vector< revolve::gazebo::SensorPtr >& sensors)
    : revolve::brain::RLPower(
        modelName,
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
        createWrapper(actuators),
        createWrapper(sensors),
        t, step
    );
}
