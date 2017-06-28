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

#ifndef TOL_PLUGIN_HELPER_H_
#define TOL_PLUGIN_HELPER_H_

#include <vector>

#include "Actuator.h"
#include "Sensor.h"

namespace tol {

class Helper
{
public:
    static const std::vector<revolve::brain::ActuatorPtr>
    createWrapper(const std::vector<revolve::gazebo::MotorPtr> &original);

    static const std::vector<revolve::brain::SensorPtr>
    createWrapper(const std::vector<revolve::gazebo::SensorPtr> &original);

    enum RobotType {
        spider9,
        spider13,
        spider17,
        gecko7,
        gecko12,
        gecko17,
        snake5,
        snake7,
        snake9,
        babyA,
        babyB,
        babyC
    };

    static RobotType parseRobotType(const std::string &str);

private:
    explicit Helper()
    {}
};
}

std::ostream& operator<<(std::ostream& os, tol::Helper::RobotType type);

#endif // TOL_PLUGIN_HELPER_H_
