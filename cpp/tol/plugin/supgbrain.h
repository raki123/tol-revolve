#ifndef SUPGBRAIN_H
#define SUPGBRAIN_H

#include <../../home/matteo/progetti/revolve/revolve/build/include/revolve/gazebo/brain/Brain.h>

class SUPGBrain : public revolve::gazebo::Brain
{
public:
    SUPGBrain();
    ~SUPGBrain();

    virtual void update(const std::vector< revolve::gazebo::MotorPtr >& motors, const std::vector< revolve::gazebo::SensorPtr >& sensors, double t, double step);
};

#endif // SUPGBRAIN_H
