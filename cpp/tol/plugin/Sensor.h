#ifndef TOL_PLUGIN_SENSOR_H_
#define TOL_PLUGIN_SENSOR_H_

#include "brain/Sensor.h"
#include "revolve/gazebo/sensors/Sensor.h"

namespace tol {

class Sensor
        : public revolve::brain::Sensor
{

public:
    explicit Sensor(revolve::gazebo::SensorPtr sensorPtr)
            :
            sensorPtr(sensorPtr)
    {}

    virtual unsigned int
    inputs() const
    {
      return sensorPtr->inputs();
    }

    virtual void
    read(double *input_vector)
    {
      sensorPtr->read(input_vector);
    }

    virtual std::string
    sensorId() const
    {
      return sensorPtr->sensorId();
    }

private:
    revolve::gazebo::SensorPtr sensorPtr;
};

}

#endif // TOL_PLUGIN_SENSOR_H_
