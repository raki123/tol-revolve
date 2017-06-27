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

#include "FakeLightSensor.h"

using namespace tol;

FakeLightSensor::FakeLightSensor(std::string name, float fov, ignition::math::Vector3d light_pos)
    : revolve::brain::FakeLightSensor(fov)
    , revolve::gazebo::VirtualSensor(nullptr, name, name, 1)
    , sensor_name(name)
    , light_pos(light_pos)
{
}

#include <iostream>

FakeLightSensor::~FakeLightSensor()
{
//     std::cout << "~FakeLightSensor()" << std::endl;
}


double FakeLightSensor::light_distance()
{
    return 1/(robot_position.Pos() - light_pos).Length();
}

double FakeLightSensor::light_angle()
{
    return 0;
}

std::string FakeLightSensor::sensorId() const
{
    return this->sensor_name;
}

void tol::FakeLightSensor::updateRobotPosition(const ignition::math::Pose3d& robot_position)
{
    this->robot_position = ignition::math::Pose3d(robot_position);
}
