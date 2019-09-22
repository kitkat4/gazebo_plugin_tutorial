#include <gazebo/gazebo.hh>

#include <iostream>

namespace gazebo{

class WorldPluginTutorial : public ModelPlugin{

public:
    
    WorldPluginTutorial() : WorldPlugin(){

        std::cout << "Hello World!" << std::endl;
    }

    void Load(physics::WorldPtr world, sdf::ElementPtr sdf){
    }
};

GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)

}

