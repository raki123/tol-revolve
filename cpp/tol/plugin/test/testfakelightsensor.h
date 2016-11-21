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

#ifndef TESTFAKELIGHTSENSOR_H
#define TESTFAKELIGHTSENSOR_H

#include <../../tol-revolve/cpp/tol/plugin/fakelightsensor.h>

class TestFakeLightSensor : public tol::FakeLightSensor
{
public:
    explicit TestFakeLightSensor(float fov,
                                 const ignition::math::Pose3d robot_sensor_offset,
                                 const ignition::math::Vector3d light_position)
        : tol::FakeLightSensor(
            "test_fake_light_sensor",
            fov,
            robot_sensor_offset,
            light_position
        )
    {}

// Expose protected methods
    float expose_light_distance() {
        return light_distance();
    }

    float expose_light_angle() {
        return light_angle();
    }

};

#endif // TESTFAKELIGHTSENSOR_H
