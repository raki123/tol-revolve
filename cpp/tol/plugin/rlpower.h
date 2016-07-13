//
// Created by Milan Jelisavcic on 28/03/16.
//

#ifndef REVOLVE_GAZEBO_BRAIN_REINFORCEDLEARNING_H
#define REVOLVE_GAZEBO_BRAIN_REINFORCEDLEARNING_H

#include "brain/rlpower.h"
#include "evaluator.h"
#include "revolve/gazebo/brain/Brain.h"

#include <gazebo/gazebo.hh>
#include <revolve/msgs/neural_net.pb.h>

#include <string>
#include <vector>

namespace tol {

class RLPower : public revolve::gazebo::Brain, private revolve::brain::RLPower {
public:
    RLPower(std::string modelName, std::shared_ptr< tol::Evaluator > evaluator, std::vector< revolve::gazebo::MotorPtr >& actuators, std::vector< revolve::gazebo::SensorPtr >& sensors);

    virtual ~RLPower();

    /**
        * @param Motor list
        * @param Sensor list
        */
    virtual void update(const std::vector <revolve::gazebo::MotorPtr> &actuators,
                        const std::vector <revolve::gazebo::SensorPtr> &sensors,
                        double t, double step);

};

} /* namespace tol */

#endif //REVOLVE_GAZEBO_BRAIN_REINFORCEDLEARNING_H

