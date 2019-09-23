#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <exception>
#include <functional>
#include <iostream>

#include "my_utils_kk4.hpp"

namespace gazebo{

class ModelPluginTest2 : public ModelPlugin{

public:

    // Constructor
    ModelPluginTest2()
        : ModelPlugin(),
          control_in_force(0.0, 0.0, 0.0){
    }
    
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf){

        try{

            this->model = model;

            auto tmp_func = std::bind(&ModelPluginTest2::OnUpdate, this);

            // Listen to the update event. This event is broadcast every simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(tmp_func);

            // std::make_shared will cause an exception. Use gazebo_ros::Node::Get
            node = gazebo_ros::Node::Get(sdf);

            // Initialize this->sub
            {
                auto tmp_callback = std::bind(&ModelPluginTest2::controlInForceCallback, this, std::placeholders::_1);

                // queue size = 1.
                // Insteaed of KeepLast, you can also use KeepAll etc.
                rclcpp::KeepLast tmp_qos(1);
                
                sub = node->create_subscription<geometry_msgs::msg::Vector3>("control_in_force",
                                                                             rclcpp::QoS(tmp_qos),
                                                                             tmp_callback);

                pub = node->create_publisher<geometry_msgs::msg::Vector3>("current_position", 1);
                
            }

            // set the update rate (you can also set it as a parameter from the sdf file)
            update_interval = common::Time(0, common::Time::SecToNano(1.0));

            // initialize the prevUpdateTime
            prev_update_time = common::Time::GetWallTime();

        }catch(std::exception& ex){
            
            std::cerr << "[ERROR] Exception: " << ex.what() << std::endl;
            throw ex;
        }

    }

    // Called by the world update start event
    void OnUpdate(){

        this->model->GetLink("base_link")->SetForce(control_in_force);
        fps.trigger();

        if(common::Time::GetWallTime() > prev_update_time + update_interval){
            prev_update_time = common::Time::GetWallTime();
            std::cerr << "[ INFO] OnUpdate is called at about " << fps.getFps() << " Hz. Publishing current position as \"current_position\"" << std::endl;

            auto com_pos = this->model->GetLink("base_link")->WorldCoGPose();

            auto tmp_msg = std::make_shared<geometry_msgs::msg::Vector3>();
            tmp_msg->x = com_pos.Pos()[0];
            tmp_msg->y = com_pos.Pos()[1];
            tmp_msg->z = com_pos.Pos()[2];

            pub->publish(*tmp_msg);
        }
    }

private:

    void controlInForceCallback(const geometry_msgs::msg::Vector3::SharedPtr msg){

        std::cerr << "[ INFO] Received " << msg->x << ", " << msg->y << ", " << msg->z << std::endl;

        control_in_force.Set(msg->x, msg->y, msg->z);
    }

    gazebo_ros::Node::SharedPtr node;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub;
    
    ignition::math::Vector3d control_in_force;
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    common::Time update_interval;
    common::Time prev_update_time;
    

    my_utils_kk4::Fps fps;
};

GZ_REGISTER_MODEL_PLUGIN(ModelPluginTest2)

}

