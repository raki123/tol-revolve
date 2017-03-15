//
// Created by matteo on 3/15/17.
//

#include "GenericLearnerBrain.h"
#include "Helper.h"

using namespace tol;

GenericLearnerBrain::GenericLearnerBrain(const std::unique_ptr<revolve::brain::BaseLearner> &learner)
    : GenericLearnerBrain(learner)
{}

GenericLearnerBrain::GenericLearnerBrain(revolve::brain::BaseLearner *learner)
    : GenericLearnerBrain(learner)
{}

void GenericLearnerBrain::update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                                 const std::vector<revolve::gazebo::SensorPtr> &sensors,
                                 double t, double step)
{
  revolve::brain::GenericLearnerBrain::update(
      Helper::createWrapper(actuators),
      Helper::createWrapper(sensors),
      t,
      step
  );
}
