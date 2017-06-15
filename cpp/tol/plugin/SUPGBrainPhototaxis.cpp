/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2016  Matteo De Carlo <matteo.dek@covolunablu.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "SUPGBrainPhototaxis.h"
#include "Helper.h"
#include <boost/make_shared.hpp>

using namespace tol;

SUPGBrainPhototaxis::SUPGBrainPhototaxis(revolve::brain::EvaluatorPtr evaluator,
                                         double light_radius_distance,
                                         const std::vector <std::vector<float>> &neuron_coordinates,
                                         const std::vector <revolve::gazebo::MotorPtr> &actuators,
                                         std::vector <revolve::gazebo::SensorPtr> &sensors)
    : revolve::brain::SUPGBrainPhototaxis(
        evaluator,
        nullptr,
        nullptr,
        light_radius_distance,
        neuron_coordinates,
        Helper::createWrapper(actuators),
        createEnhancedSensorWrapper(sensors)
    )
{
    light_constructor_left = [this] (std::vector<float> coordinates)
        -> boost::shared_ptr<FakeLightSensor>
    {
        ignition::math::Vector3d offset(coordinates[0]/100, coordinates[1]/100, 0);
        ignition::math::Vector3d light_pos = this->robot_position.CoordPositionAdd(offset);
        // this function is not supposed to delete the light
        light_sensor_left.reset(new FakeLightSensor("sensor_left", 160, light_pos));
        return light_sensor_left;
    };

    light_constructor_right = [this] (std::vector<float> coordinates)
        -> boost::shared_ptr<FakeLightSensor>
    {
        ignition::math::Vector3d offset(coordinates[0]/100, coordinates[1]/100, 0);
        ignition::math::Vector3d light_pos = this->robot_position.CoordPositionAdd(offset);
        // this function is not supposed to delete the light
        light_sensor_right.reset(new FakeLightSensor("sensor_right", 160, light_pos));
        return light_sensor_right;
    };

    light_constructor_left({1,1});
    light_constructor_right({1,1});
    sensors.push_back(light_sensor_left);
    sensors.push_back(light_sensor_right);
}

SUPGBrainPhototaxis::~SUPGBrainPhototaxis()
{}

void SUPGBrainPhototaxis::update(const std::vector <revolve::gazebo::MotorPtr> &motors,
                                 const std::vector <revolve::gazebo::SensorPtr> &sensors,
                                 double t, double step)
{
    revolve::brain::SUPGBrainPhototaxis::update(Helper::createWrapper(motors),
                                                Helper::createWrapper(sensors),
                                                t, step);
}

void SUPGBrainPhototaxis::updateRobotPosition(ignition::math::Pose3d &robot_position)
{
    this->robot_position = ignition::math::Pose3d(robot_position);
    light_sensor_left->updateRobotPosition(robot_position);
    light_sensor_right->updateRobotPosition(robot_position);
}


const std::vector<revolve::brain::SensorPtr> tol::SUPGBrainPhototaxis::createEnhancedSensorWrapper(const std::vector<revolve::gazebo::SensorPtr>& original)
{
    std::vector<revolve::brain::SensorPtr> result = Helper::createWrapper(original);
    result.push_back(boost::make_shared<tol::FakeLightSensor>("sensor_1_fake_filler", 0, ignition::math::Vector3d()));
    result.push_back(boost::make_shared<tol::FakeLightSensor>("sensor_2_fake_filler", 0, ignition::math::Vector3d()));

    return result;
}
