#include "supgbrain.h"
#include "sensor.h"
#include "actuator.h"
#include "revolve/gazebo/sensors/VirtualSensor.h"
#include "revolve/gazebo/motors/Motor.h"

const std::vector< revolve::brain::ActuatorPtr > createWrapper(const std::vector < revolve::gazebo::MotorPtr > &original)
{
    std::vector< revolve::brain::ActuatorPtr > result;
    for (int i=0; i<original.size(); i++) {
        result.push_back(boost::make_shared<tol::Actuator>(tol::Actuator(original[i])));
    }

    return result;
}

const std::vector< revolve::brain::SensorPtr > createWrapper(const std::vector < revolve::gazebo::SensorPtr > &original)
{
    std::vector< revolve::brain::SensorPtr > result;
    for (int i=0; i<original.size(); i++) {
        result.push_back(boost::make_shared<tol::Sensor>(tol::Sensor(original[i])));
    }

    return result;
}

tol::SUPGBrain::SUPGBrain(revolve::brain::EvaluatorPtr evaluator,
                          const std::vector< std::vector< float > > &neuron_coordinates,
                          const std::vector< revolve::gazebo::MotorPtr >& motors,
                          const std::vector< revolve::gazebo::SensorPtr >& sensors)
 : revolve::brain::SUPGBrain(
     evaluator,
     neuron_coordinates,
     createWrapper(motors),
     createWrapper(sensors)
    )
{}

tol::SUPGBrain::~SUPGBrain()
{

}


void tol::SUPGBrain::update(const std::vector< revolve::gazebo::MotorPtr >& motors,
                            const std::vector< revolve::gazebo::SensorPtr >& sensors,
                            double t, double step)
{
    revolve::brain::SUPGBrain::update(
        createWrapper(motors),
        createWrapper(sensors),
        t,step
    );

}
