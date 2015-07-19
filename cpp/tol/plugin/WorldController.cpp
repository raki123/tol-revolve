/**
 * World controller for now just registers Revolve's world controller.
 */

#include <gazebo/gazebo.hh>
#include <revolve/gazebo/plugin/WorldController.h>
GZ_REGISTER_WORLD_PLUGIN(revolve::gazebo::WorldController)
