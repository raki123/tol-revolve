#ifndef SUPGBRAIN_H
#define SUPGBRAIN_H

#include "brain/SUPGBrain.h"
#include "Evaluator.h"
#include "revolve/gazebo/brain/Brain.h"

#include <gazebo/gazebo.hh>

#include <vector>
#include <memory>

namespace tol {

class SUPGBrain
        : public revolve::gazebo::Brain
        , private revolve::brain::SUPGBrain
{
  public:

  /// \brief Constructor
  SUPGBrain(const std::string &robot_name,
            revolve::brain::EvaluatorPtr evaluator,
            const std::vector<std::vector<float> > &neuron_coordinates,
            const std::vector<revolve::gazebo::MotorPtr> &motors,
            const std::vector<revolve::gazebo::SensorPtr> &sensors);

  /// \brief Destructor
  ~SUPGBrain();

  using revolve::brain::SUPGBrain::update;
  /// \brief Update sensors reading, actuators position, and `brain` state
  /// \param[inout] actuators List of actuators
  /// \param[inout] sensors List of sensors
  /// \param[in] t Time value
  /// \param[in] step Time step
  virtual void update(const std::vector<revolve::gazebo::MotorPtr> &motors,
                      const std::vector<revolve::gazebo::SensorPtr> &sensors,
                      double t, double step) override;

};

}

#endif // SUPGBRAIN_H
