//
// Created by elte on 6-6-15.
//

#ifndef TRIANGLEOFLIFE_WORLDCONTROLLER_H
#define TRIANGLEOFLIFE_WORLDCONTROLLER_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>

#include "insert_sdf.pb.h"

namespace tol {

typedef const boost::shared_ptr<const msgs::InsertSdfRequest> ConstInsertSdfRequestPtr;

class WorldController: public gazebo::WorldPlugin {
public:
	void Load(::gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);
private:
	// Listener for analysis requests
	void InsertRequest(ConstInsertSdfRequestPtr &request);

	// Stores the world
	gazebo::physics::WorldPtr world_;

	// Transport node
	gazebo::transport::NodePtr node_;

	// Subscriber
	gazebo::transport::SubscriberPtr insertSub_;
};

} // namespace tol

GZ_REGISTER_WORLD_PLUGIN(tol::WorldController)

#endif //TRIANGLEOFLIFE_WORLDCONTROLLER_H
