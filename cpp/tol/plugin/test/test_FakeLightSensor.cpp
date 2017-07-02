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

#include "test_FakeLightSensor.h"

#define BOOST_TEST_MODULE FakeLightSensor test
#define BOOST_TEST_DYN_LINK
#include <boost/test/included/unit_test.hpp>

#include <iostream>
#include <cmath>

const double pi = std::acos(-1);
const double angle_15 = pi/12;
const double angle_52_5 = 7*pi/24;

void test_sensor_angle(double target_angle, double fov,
                       const ignition::math::Pose3d robot_sensor_offset,
                       const ignition::math::Vector3d light_position,
                       double eps = 0.0001,
                       std::string text = ""
                      )
{

    TestFakeLightSensor sensor(
        fov,
        robot_sensor_offset,
        light_position
    );

    auto angle = sensor.expose_light_angle();
    BOOST_TEST((std::fabs(angle - target_angle) <= eps),
        text << angle << " - " << target_angle << " <= " << eps
        << " does not hold (angle test)"
    );
}

void test_sensor_distance(double target_distance, double fov,
                          const ignition::math::Pose3d robot_sensor_offset,
                          const ignition::math::Vector3d light_position,
                          double eps = 0.0001,
                          std::string text = ""
                         )
{

    TestFakeLightSensor sensor(
        fov,
        robot_sensor_offset,
        light_position
    );

    auto distance = sensor.expose_light_distance();
    BOOST_TEST((std::fabs(distance - target_distance) <= eps),
        text << distance << " - " << target_distance << " <= " << eps
        << " does not hold (distance test)"
    );
}


BOOST_AUTO_TEST_CASE(fake_sensor_distance_different_lights)
{

    ignition::math::Pose3d robot_pose(0,0,0,0,0,0);

    test_sensor_distance(
        50,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,50,0)
    );

    test_sensor_distance(
        50,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,50)
    );

    test_sensor_distance(
        5,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,5)
    );

    test_sensor_distance(
        50,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,-50)
    );

    test_sensor_distance(
        5,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,-5)
    );

    test_sensor_distance(
        std::sin(pi/4),
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,.5,.5)
    );

    test_sensor_distance(
        std::sin(pi/4)*2,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,1,1)
    );

    test_sensor_distance(
        .5,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(1,0,.5)
    );

    test_sensor_distance(
        .5,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(-1,0,.5)
    );

    test_sensor_distance(
        std::sin(pi/4),
        150.0f,
        robot_pose,
        ignition::math::Vector3d(100,.5,.5)
    );
}

BOOST_AUTO_TEST_CASE(fake_sensor_angle_different_lights)
{

    ignition::math::Pose3d robot_pose(0,0,0,0,0,0);

    test_sensor_angle(
        0,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,50,0)
    );

    test_sensor_angle(
        1,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,50)
    );

    test_sensor_angle(
        1,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,5)
    );

    test_sensor_angle(
        -1,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,-50)
    );

    test_sensor_angle(
        -1,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,-5)
    );

    test_sensor_angle(
        std::sin(pi/4),
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,50,50)
    );
    test_sensor_angle(
        std::sin(pi/4),
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,.5,.5)
    );


    test_sensor_angle(
        1,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(1,0,.5)
    );

    test_sensor_angle(
        1,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(-1,0,.5)
    );

    test_sensor_angle(
        std::sin(pi/4),
        150.0f,
        robot_pose,
        ignition::math::Vector3d(100,.5,.5)
    );
}

BOOST_AUTO_TEST_CASE(fake_sensor_angle_different_sensors_positions)
{

    ignition::math::Vector3d light_position(0,0,0);

    test_sensor_angle(
        0,
        150.0f,
        ignition::math::Pose3d(0,0,0,0,0,0),
        light_position
    );

    test_sensor_angle(
        1,
        150.0f,
        ignition::math::Pose3d(0,0,-50,0,0,0),
        light_position
    );

    test_sensor_angle(
        -1,
        150.0f,
        ignition::math::Pose3d(0,0,50,0,0,0),
        light_position
    );

    test_sensor_angle(
        1,
        150.0f,
        ignition::math::Pose3d(0,0,-1,0,0,0),
        light_position
    );

    test_sensor_angle(
        -1,
        150.0f,
        ignition::math::Pose3d(0,0,1,0,0,0),
        light_position
    );

    test_sensor_angle(
        std::sin(pi/4),
        150.0f,
        ignition::math::Pose3d(0,-1,-1,0,0,0),
        light_position
    );
}

BOOST_AUTO_TEST_CASE(fake_sensor_angle_different_sensors_angles)
{

    ignition::math::Vector3d light_position(0,0,0);

    test_sensor_angle(
        .1,
        150.0f,
        ignition::math::Pose3d(0,.5,0,1,0,0),
        light_position
    );

    test_sensor_angle(
        .1,
        150.0f,
        ignition::math::Pose3d(0,.5,0,0,-1,0),
        light_position
    );

    test_sensor_angle(
        .1,
        150.0f,
        ignition::math::Pose3d(0,.5,0,0,0,.1),
        light_position
    );

    test_sensor_angle(
        .1,
        150.0f,
        ignition::math::Pose3d(0,.5,0,0,0,0),
        light_position
    );

    test_sensor_angle(
        .1,
        150.0f,
        ignition::math::Pose3d(0,.5,0,0,0,0),
        light_position
    );

    test_sensor_angle(
        std::sin(pi/4),
        150.0f,
        ignition::math::Pose3d(0,.5,0,0,0,0),
        light_position
    );
}

