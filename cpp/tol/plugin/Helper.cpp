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

#include "Helper.h"

#include <boost/make_shared.hpp>

using namespace tol;

const std::vector<revolve::brain::ActuatorPtr>
Helper::createWrapper(const std::vector<revolve::gazebo::MotorPtr> &original) {
    std::vector<revolve::brain::ActuatorPtr> result;
    for (unsigned int i = 0; i < original.size(); i++) {
        result.push_back(boost::make_shared<tol::Actuator>(tol::Actuator(original[i])));
    }

    return result;
}

const std::vector<revolve::brain::SensorPtr>
Helper::createWrapper(const std::vector<revolve::gazebo::SensorPtr> &original) {
    std::vector<revolve::brain::SensorPtr> result;
    for (unsigned int i = 0; i < original.size(); i++) {
        result.push_back(boost::make_shared<tol::Sensor>(tol::Sensor(original[i])));
    }

    return result;
}
