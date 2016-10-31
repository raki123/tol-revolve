/**
 * World controller for now just registers Revolve's world controller.
 */

#include <revolve/gazebo/plugin/WorldController.h>
#include <iostream>

class WorldController : public revolve::gazebo::WorldController {
public:
    WorldController() :
        revolve::gazebo::WorldController()
    {
        std::cout<<"Starging World Controller!"<<std::endl;
    }
};

GZ_REGISTER_WORLD_PLUGIN(WorldController)
