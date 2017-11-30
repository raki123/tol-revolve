/*
 * RobotController.h
 *
 *  Created on: May 9, 2015
 *      Author: elte
 */

#ifndef TOL_PLUGIN_ROBOTCONTROLLER_H_
#define TOL_PLUGIN_ROBOTCONTROLLER_H_

#include <revolve/gazebo/plugin/RobotController.h>
#include <boost/make_shared.hpp>

#include "brain/Evaluator.h"
#include "brain/RLPower.h"

namespace tol {

class RobotController
        : public revolve::gazebo::RobotController
{
public:
    RobotController();

    virtual ~RobotController();

    virtual void
    Load(::gazebo::physics::ModelPtr _parent,
         sdf::ElementPtr _sdf);

    virtual void
    LoadBrain(sdf::ElementPtr sdf);

    virtual void
    DoUpdate(const gazebo::common::UpdateInfo info);

private:
    class Evaluator
            : public tol::Evaluator
    {
    public:
        Evaluator();

        virtual void
        start();

        virtual double
        fitness();

        void
        updatePosition(const ignition::math::Pose3d pose);

        ignition::math::Pose3d currentPosition_;
        ignition::math::Pose3d previousPosition_;
    };

    boost::shared_ptr<Evaluator> evaluator_;
};

} /* namespace tol */

GZ_REGISTER_MODEL_PLUGIN(tol::RobotController)

#endif // TTOL_PLUGIN_ROBOTCONTROLLER_H_
