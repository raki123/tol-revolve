#ifndef TOL_ACTUATOR_H
#define TOL_ACTUATOR_H

#include "brain/actuator.h"
#include "revolve/gazebo/motors/Motor.h"

namespace tol {

class Actuator : public revolve::brain::Actuator
{
public:
explicit Actuator(revolve::gazebo::MotorPtr actuatorPtr)
 : actuatorPtr(actuatorPtr)
{}

virtual unsigned int outputs() const
{
    return actuatorPtr->outputs();
}

virtual void update(double* output_vector, double step)
{
    actuatorPtr->update(output_vector, step);
}

private:
    revolve::gazebo::MotorPtr actuatorPtr;
};

}

#endif // TOL_ACTUATOR_H
