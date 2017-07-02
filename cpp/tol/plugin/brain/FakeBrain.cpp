#include "FakeBrain.h"

#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

namespace tol {

FakeBrain::FakeBrain(std::string modelName,
                     std::vector<revolve::gazebo::MotorPtr> &actuators,
                     std::vector<revolve::gazebo::SensorPtr> &sensors)
        : nActuators_(actuators.size())
          , nSensors_(sensors.size())
          , start_eval_time_(0)
          , cycle_start_time_(-1)
{
  std::cout << "FakeBrain::FakeBrain()" << std::endl;

//        // Create transport node
//        node_.reset(new ::gazebo::transport::Node());
//        node_->Init();
//
//        // Listen to network modification requests
//        alterSub_ = node_->Subscribe("~/" + modelName + "/modify_neural_network", &FakeBrain::modify, this);
}

FakeBrain::~FakeBrain()
{
}


void
FakeBrain::modify(ConstModifyNeuralNetworkPtr &req)
{
  std::cout << "FakeBrain::modify()" << std::endl;
}

void
FakeBrain::update(const std::vector<revolve::gazebo::MotorPtr> &motors,
                  const std::vector<revolve::gazebo::SensorPtr> &sensors,
                  double t,
                  double step)
{
  std::cout << "FakeBrain::update()" << std::endl;
}

} /* namespace tol */

