//
// Created by matteo on 3/15/17.
//

#ifndef TRIANGLEOFLIFE_GENERICLEARNERBRAIN_H
#define TRIANGLEOFLIFE_GENERICLEARNERBRAIN_H

#include <revolve/gazebo/brain/Brain.h>
#include "brain/GenericLearnerBrain.h"

namespace tol {

    class GenericLearnerBrain : public revolve::gazebo::Brain,
                                private revolve::brain::GenericLearnerBrain {
    public:
        GenericLearnerBrain(const std::unique_ptr<revolve::brain::BaseLearner> &learner);

        GenericLearnerBrain(revolve::brain::BaseLearner *learner);

        void update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                    const std::vector<revolve::gazebo::SensorPtr> &sensors,
                    double t, double step) override;
    };

}


#endif //TRIANGLEOFLIFE_GENERICLEARNERBRAIN_H
