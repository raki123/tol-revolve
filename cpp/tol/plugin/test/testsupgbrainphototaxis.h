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

#ifndef TESTSUPGBRAINPHOTOTAXIS_H
#define TESTSUPGBRAINPHOTOTAXIS_H

#include "../SUPGBrainPhototaxis.h"

class testSUPGBrainPhototaxis : private tol::SUPGBrainPhototaxis
{
public:
    testSUPGBrainPhototaxis(revolve::brain::EvaluatorPtr evaluator,
                            std::function<revolve::brain::FakeLightSensor *(std::vector<float> coordinates)> _light_constructor_left,
                            std::function<revolve::brain::FakeLightSensor *(std::vector<float> coordinates)> _light_constructor_right,
                            double light_radius_distance,
                            const std::vector< std::vector< float > > &neuron_coordinates,
                            const std::vector< revolve::gazebo::MotorPtr >& motors,
                            const std::vector< revolve::gazebo::SensorPtr >& sensors);

    void update(const std::vector< revolve::gazebo::MotorPtr >& motors,
                const std::vector< revolve::gazebo::SensorPtr >& sensors,
                double t, double step);
};

#endif // TESTSUPGBRAINPHOTOTAXIS_H
