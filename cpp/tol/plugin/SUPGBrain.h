#ifndef SUPGBRAIN_H
#define SUPGBRAIN_H

#include "brain/SUPGBrain.h"
#include "Evaluator.h"
#include "revolve/gazebo/brain/Brain.h"

#include <gazebo/gazebo.hh>

#include <vector>
#include <memory>

namespace tol {

class SUPGBrain : public revolve::gazebo::Brain, private revolve::brain::SUPGBrain
{
public:
    SUPGBrain(revolve::brain::EvaluatorPtr evaluator,
              const std::vector< std::vector< float > > &neuron_coordinates,
              const std::vector< revolve::gazebo::MotorPtr >& motors,
              const std::vector< revolve::gazebo::SensorPtr >& sensors);

    ~SUPGBrain();

    virtual void update(const std::vector< revolve::gazebo::MotorPtr >& motors,
                        const std::vector< revolve::gazebo::SensorPtr >& sensors,
                        double t, double step) override;

};

}

#endif // SUPGBRAIN_H
