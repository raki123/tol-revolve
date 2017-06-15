#include "SUPGBrain.h"
#include "Sensor.h"
#include "Actuator.h"
#include "revolve/gazebo/sensors/VirtualSensor.h"
#include "revolve/gazebo/motors/Motor.h"
#include "Helper.h"

using namespace tol;

SUPGBrain::SUPGBrain(revolve::brain::EvaluatorPtr evaluator,
                          const std::vector< std::vector< float > > &neuron_coordinates,
                          const std::vector< revolve::gazebo::MotorPtr >& motors,
                          const std::vector< revolve::gazebo::SensorPtr >& sensors)
 : revolve::brain::SUPGBrain(
     evaluator,
     neuron_coordinates,
     Helper::createWrapper(motors),
     Helper::createWrapper(sensors)
    )
{
    std::cerr<<"tol::SUPGBrain::SUPGBrain()"<<std::endl;
}


SUPGBrain::~SUPGBrain()
{

}


void tol::SUPGBrain::update(const std::vector< revolve::gazebo::MotorPtr >& motors,
                            const std::vector< revolve::gazebo::SensorPtr >& sensors,
                            double t, double step)
{
    revolve::brain::SUPGBrain::update(
        Helper::createWrapper(motors),
        Helper::createWrapper(sensors),
        t,step
    );

}
