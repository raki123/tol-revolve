#ifndef FAKEBRAIN_H
#define FAKEBRAIN_H

#include <gazebo/gazebo.hh>

#include <revolve/msgs/neural_net.pb.h>
#include "revolve/gazebo/brain/Brain.h"

namespace tol
{

class FakeBrain
        : public revolve::gazebo::Brain
{
public:
    typedef const boost::shared_ptr<revolve::msgs::ModifyNeuralNetwork const> ConstModifyNeuralNetworkPtr;

    FakeBrain(std::string modelName,
              std::vector<revolve::gazebo::MotorPtr> &actuators,
              std::vector<revolve::gazebo::SensorPtr> &sensors);

    virtual ~FakeBrain();

    /**
     * @param Motor list
     * @param Sensor list
     */
    virtual void
    update(const std::vector<revolve::gazebo::MotorPtr> &motors,
           const std::vector<revolve::gazebo::SensorPtr> &sensors,
           double t,
           double step);

protected:
    /**
     * Request handler to modify the neural network
     */
    void
    modify(ConstModifyNeuralNetworkPtr &req);

    // Mutex for stepping / updating the network
    //boost::mutex networkMutex_;

    unsigned int nActuators_;
    unsigned int nSensors_;

    double start_eval_time_;

private:
    double cycle_start_time_;

    /**
     * Transport node
     */
//     ::gazebo::transport::NodePtr node_;

    /**
     * Network modification subscriber
     */
//     ::gazebo::transport::SubscriberPtr alterSub_;
};

} /* namespace tol */

#endif // FAKEBRAIN_H
