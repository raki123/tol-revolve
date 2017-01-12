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

#include "supgbrainphototaxis.h"
#include "helper.h"

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
        Helper::createWrapper(sensors)
    )
{

//     ignition::math::Vector3d offset(0,0,0);
//     ignition::math::Vector3d light_pos = this->robot_position.CoordPositionAdd(offset);

    light_constructor_left = [this] (std::vector<float> coordinates) -> FakeLightSensor * {
        ignition::math::Vector3d offset(coordinates[0]/100, coordinates[1]/100, 0);
        ignition::math::Vector3d light_pos = this->robot_position.CoordPositionAdd(offset);
        // this function is not supposed to delete the light
        light_sensor_left = new FakeLightSensor("sensor_left", 160, light_pos);
        return light_sensor_left;
    };

    light_constructor_right = [this] (std::vector<float> coordinates) -> FakeLightSensor * {
        ignition::math::Vector3d offset(coordinates[0]/100, coordinates[1]/100, 0);
        ignition::math::Vector3d light_pos = this->robot_position.CoordPositionAdd(offset);
        // this function is not supposed to delete the light
        light_sensor_right = new FakeLightSensor("sensor_right", 160, light_pos);
        return light_sensor_right;
    };
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

