#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <functional>
#include <iostream>

namespace gazebo{

class ModelPluginTest0 : public ModelPlugin{

public:

    // Constructor
    ModelPluginTest0() : ModelPlugin(){
    }
    
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf){

        this->model = model;

        auto tmp_func = std::bind(&ModelPluginTest0::OnUpdate, this);

        // Listen to the update event. This event is broadcast every simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(tmp_func);
    }

    // Called by the world update start event
    void OnUpdate(){

        this->model->SetLinearVel(ignition::math::Vector3d(0.3, 0.0, 0.0));
    }

private:

    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
};

GZ_REGISTER_MODEL_PLUGIN(ModelPluginTest0)

}

