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

#ifndef FAKELIGHTSENSOR_H
#define FAKELIGHTSENSOR_H

#include "brain/fakelightsensor.h"
#include <ignition/math/Pose3.hh>
#include <string>

namespace tol {

class FakeLightSensor : public revolve::brain::FakeLightSensor
{
public:
    FakeLightSensor(std::string name, float fov, ignition::math::Vector3d light_pos);
    virtual ~FakeLightSensor();

    virtual float light_distance() override;
    virtual float light_angle() override;

    virtual std::string sensorId() const override;

    virtual void updateRobotPosition(ignition::math::Pose3d &robot_position);

private:
    std::string sensor_name;
    ignition::math::Vector3d light_pos;
    ignition::math::Pose3d robot_position;
};

}

#endif // FAKELIGHTSENSOR_H
