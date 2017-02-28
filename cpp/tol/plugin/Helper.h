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

#ifndef HELPER_H
#define HELPER_H

#include "Actuator.h"
#include "Sensor.h"

#include <vector>

namespace tol {

    class Helper {
    public:
        static const std::vector<revolve::brain::ActuatorPtr>
        createWrapper(const std::vector<revolve::gazebo::MotorPtr> &original);

        static const std::vector<revolve::brain::SensorPtr>
        createWrapper(const std::vector<revolve::gazebo::SensorPtr> &original);

    private:
        explicit Helper() {}
    };
}

#endif // HELPER_H
