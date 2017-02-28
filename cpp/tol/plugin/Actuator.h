#ifndef TOL_ACTUATOR_H
#define TOL_ACTUATOR_H

#include "brain/actuator.h"
#include "revolve/gazebo/motors/Motor.h"

namespace tol
{

class Actuator
        : public revolve::brain::Actuator
{
public:
    explicit Actuator(revolve::gazebo::MotorPtr actuatorPtr)
            :
            actuatorPtr(actuatorPtr)
    {}

    virtual unsigned int
    outputs() const
    {
      return actuatorPtr->outputs();
    }

    virtual void
    update(double *output_vector,
           double step)
    {
      unsigned int size = this->outputs();
      double *output_copy = new double[size];

      for (unsigned int i = 0; i < size; i++) {
        output_copy[i] = (output_vector[i] + 1) / 2;
      }

      actuatorPtr->update(output_copy,
                          step);

      delete[] output_copy;
    }

private:
    revolve::gazebo::MotorPtr actuatorPtr;
};

}

#endif // TOL_ACTUATOR_H
