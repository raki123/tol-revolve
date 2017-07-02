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

#include "test_SUPGBrainPhototaxis.h"
#include "../FakeLightSensor.h"

#include <revolve/gazebo/motors/Motor.h>
#include <revolve/gazebo/sensors/VirtualSensor.h>
#include <gazebo/physics/Model.hh>

// #define BOOST_TEST_NO_MAIN
#define BOOST_TEST_MODULE SUPGBrainPhototaxis test
#define BOOST_TEST_DYN_LINK
#include <boost/test/included/unit_test.hpp>
#include <iostream>

class Motor : public revolve::gazebo::Motor {
public:
    Motor()
        : revolve::gazebo::Motor(
            nullptr, //::gazebo::physics::ModelPtr model
            "partId", //std::string partId
            "motorId", //std::string motorId
            1 // unsigned int outputs
        )
    {}

    virtual void update(double * output, double step) {}
};
typedef boost::shared_ptr< Motor > MotorPtr;

class Sensor : public revolve::gazebo::VirtualSensor {

};
typedef boost::shared_ptr< Sensor > SensorPtr;

class Evaluator : public tol::Evaluator {
public:
    Evaluator(double fitness) : _fitness(fitness) {}
    virtual void start() {}
    virtual double fitness() {
        return _fitness;
    }

    double _fitness;
};

BOOST_AUTO_TEST_CASE(create_instance)
{
    // GENERIC BRAIN STUFF
    boost::shared_ptr< Evaluator > evaluator_ = boost::make_shared<Evaluator>(1);

    std::vector< revolve::gazebo::MotorPtr > motors_
    ( {
        boost::make_shared<Motor>(),
        boost::make_shared<Motor>(),
        boost::make_shared<Motor>(),
        boost::make_shared<Motor>()
    } );

    std::vector< revolve::gazebo::SensorPtr > sensors_
    ( {

    } );

    // SUPG BRAIN STUFF
    AsyncNeat::Init();
    std::vector< std::vector< float> > coordinates
    ( {
        // Leg00Joint
        { -.5, 0},
        // Leg01Joint
        { -1,  0},
        // Leg10Joint
        { +.5, 0},
        // Leg11Joint
        { +1,  0},
    } );

    // SPECIFIC SUPG BRAIN PHOTOTAXIS STUFF
    static const float fov = 150.0f;
    static const float sensor_offset = 0.05;

    revolve::brain::FakeLightSensor *sensor_left;
    revolve::brain::FakeLightSensor *sensor_right;

    std::function<revolve::brain::FakeLightSensor *(std::vector<float> coordinates)> light_constructor_left;
    std::function<revolve::brain::FakeLightSensor *(std::vector<float> coordinates)> light_constructor_right;

    auto light_constructor_common = [] (const std::vector<float> &coordinates,
                                        float sensor_offset)
                                    -> revolve::brain::FakeLightSensor*
    {
        if (coordinates.size() != 3) {
            std::cerr << "ERROR! COORDINATES TO THE LIGHT CONSTRUCTOR ARE THE WRONG SIZE!" << std::endl;
        }

        ignition::math::Vector3d light_position(
            coordinates[0],
            coordinates[1],
            coordinates[2]
        );

        return new tol::FakeLightSensor(
            "test_fake_light_sensor",
            fov,
            ignition::math::Pose3d(0,sensor_offset,0,0,0,0),
            light_position
        );
    };

    light_constructor_left = [&sensor_left, light_constructor_common] (std::vector<float> coordinates)
                             -> revolve::brain::FakeLightSensor *
    {
        delete sensor_left;
        sensor_left = light_constructor_common(coordinates, -sensor_offset);
        return sensor_left;
    };

    light_constructor_right = [&sensor_right, light_constructor_common] (std::vector<float> coordinates)
                              -> revolve::brain::FakeLightSensor *
    {
        delete sensor_right;
        sensor_right = light_constructor_common(coordinates, sensor_offset);
        return sensor_right;
    };

    testSUPGBrainPhototaxis testObject(
        evaluator_,
        light_constructor_left,
        light_constructor_right,
        0.5,
        coordinates,
        motors_,
        sensors_);

    AsyncNeat::CleanUp();
}


BOOST_AUTO_TEST_CASE(run_once)
{
    // GENERIC BRAIN STUFF
    boost::shared_ptr< Evaluator > evaluator_ = boost::make_shared<Evaluator>(1);

    std::vector< revolve::gazebo::MotorPtr > motors_
    ( {
        boost::make_shared<Motor>(),
        boost::make_shared<Motor>(),
        boost::make_shared<Motor>(),
        boost::make_shared<Motor>()
    } );

    std::vector< revolve::gazebo::SensorPtr > sensors_
    ( {

    } );

    // SUPG BRAIN STUFF
    AsyncNeat::Init();
    std::vector< std::vector< float> > coordinates
    ( {
        // Leg00Joint
        { -.5, 0},
        // Leg01Joint
        { -1,  0},
        // Leg10Joint
        { +.5, 0},
        // Leg11Joint
        { +1,  0},
    } );

    // SPECIFIC SUPG BRAIN PHOTOTAXIS STUFF
    static const float fov = 150.0f;
    static const float sensor_offset = 0.05;

    revolve::brain::FakeLightSensor *sensor_left;
    revolve::brain::FakeLightSensor *sensor_right;

    std::function<revolve::brain::FakeLightSensor *(std::vector<float> coordinates)> light_constructor_left;
    std::function<revolve::brain::FakeLightSensor *(std::vector<float> coordinates)> light_constructor_right;

    auto light_constructor_common = [] (const std::vector<float> &coordinates,
                                        float sensor_offset)
                                    -> revolve::brain::FakeLightSensor*
    {
        if (coordinates.size() != 3) {
            std::cerr << "ERROR! COORDINATES TO THE LIGHT CONSTRUCTOR ARE THE WRONG SIZE!" << std::endl;
        }

        ignition::math::Vector3d light_position(
            coordinates[0],
            coordinates[1],
            coordinates[2]
        );

        return new tol::FakeLightSensor(
            "test_fake_light_sensor",
            fov,
            ignition::math::Pose3d(0,sensor_offset,0,0,0,0),
            light_position
        );
    };

    light_constructor_left = [&sensor_left, light_constructor_common] (std::vector<float> coordinates)
                             -> revolve::brain::FakeLightSensor *
    {
        delete sensor_left;
        sensor_left = light_constructor_common(coordinates, -sensor_offset);
        return sensor_left;
    };

    light_constructor_right = [&sensor_right, light_constructor_common] (std::vector<float> coordinates)
                              -> revolve::brain::FakeLightSensor *
    {
        delete sensor_right;
        sensor_right = light_constructor_common(coordinates, sensor_offset);
        return sensor_right;
    };

    testSUPGBrainPhototaxis testObject(
        evaluator_,
        light_constructor_left,
        light_constructor_right,
        0.5,
        coordinates,
        motors_,
        sensors_);

    testObject.update(motors_, sensors_, 1, 0.1);

    AsyncNeat::CleanUp();
}

BOOST_AUTO_TEST_CASE(run_multiple_times)
{
    // GENERIC BRAIN STUFF
    boost::shared_ptr< Evaluator > evaluator_ = boost::make_shared<Evaluator>(1);

    std::vector< revolve::gazebo::MotorPtr > motors_
    ( {
        boost::make_shared<Motor>(),
        boost::make_shared<Motor>(),
        boost::make_shared<Motor>(),
        boost::make_shared<Motor>()
    } );

    std::vector< revolve::gazebo::SensorPtr > sensors_
    ( {

    } );

    // SUPG BRAIN STUFF
    AsyncNeat::Init();
    std::vector< std::vector< float> > coordinates
    ( {
        // Leg00Joint
        { -.5, 0},
        // Leg01Joint
        { -1,  0},
        // Leg10Joint
        { +.5, 0},
        // Leg11Joint
        { +1,  0},
    } );

    // SPECIFIC SUPG BRAIN PHOTOTAXIS STUFF
    static const float fov = 150.0f;
    static const float sensor_offset = 0.05;

    revolve::brain::FakeLightSensor *sensor_left;
    revolve::brain::FakeLightSensor *sensor_right;

    std::function<revolve::brain::FakeLightSensor *(std::vector<float> coordinates)> light_constructor_left;
    std::function<revolve::brain::FakeLightSensor *(std::vector<float> coordinates)> light_constructor_right;

    auto light_constructor_common = [] (const std::vector<float> &coordinates,
                                        float sensor_offset)
                                    -> revolve::brain::FakeLightSensor*
    {
        if (coordinates.size() != 3) {
            std::cerr << "ERROR! COORDINATES TO THE LIGHT CONSTRUCTOR ARE THE WRONG SIZE!" << std::endl;
        }

        ignition::math::Vector3d light_position(
            coordinates[0],
            coordinates[1],
            coordinates[2]
        );

        return new tol::FakeLightSensor(
            "test_fake_light_sensor",
            fov,
            ignition::math::Pose3d(0,sensor_offset,0,0,0,0),
            light_position
        );
    };

    light_constructor_left = [&sensor_left, light_constructor_common] (std::vector<float> coordinates)
                             -> revolve::brain::FakeLightSensor *
    {
        delete sensor_left;
        sensor_left = light_constructor_common(coordinates, -sensor_offset);
        return sensor_left;
    };

    light_constructor_right = [&sensor_right, light_constructor_common] (std::vector<float> coordinates)
                              -> revolve::brain::FakeLightSensor *
    {
        delete sensor_right;
        sensor_right = light_constructor_common(coordinates, sensor_offset);
        return sensor_right;
    };

    testSUPGBrainPhototaxis testObject(
        evaluator_,
        light_constructor_left,
        light_constructor_right,
        0.5,
        coordinates,
        motors_,
        sensors_);

    double step = 0.1;
    for (double time = 0; time < 100; time +=step)
        testObject.update(motors_, sensors_, time, step);

    AsyncNeat::CleanUp();
}

testSUPGBrainPhototaxis::testSUPGBrainPhototaxis(revolve::brain::EvaluatorPtr evaluator,
        std::function<revolve::brain::FakeLightSensor *(std::vector<float> coordinates)> _light_constructor_left,
        std::function<revolve::brain::FakeLightSensor *(std::vector<float> coordinates)> _light_constructor_right,
        double light_radius_distance,
        const std::vector< std::vector< float > > &neuron_coordinates,
        const std::vector< revolve::gazebo::MotorPtr >& motors,
        const std::vector< revolve::gazebo::SensorPtr >& sensors)
    : tol::SUPGBrainPhototaxis(evaluator,
                               _light_constructor_left,
                               _light_constructor_right,
                               light_radius_distance,
                               neuron_coordinates,
                               motors,
                               sensors)
{
}

void testSUPGBrainPhototaxis::update(const std::vector< revolve::gazebo::MotorPtr >& motors,
                                     const std::vector< revolve::gazebo::SensorPtr >& sensors,
                                     double t, double step)
{
    tol::SUPGBrainPhototaxis::update(motors, sensors, t, step);
}
