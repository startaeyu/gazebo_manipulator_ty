#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <stdio.h>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "manipulator/three.h"
#include "manipulator/nine.h"

#define pi 3.14159265359

 ros::NodeHandle nh;
 ros::Publisher pub_actp = nh.advertise<manipulator::three>("manipulator_actp", 100);
 ros::Publisher pub_actv = nh.advertise<manipulator::three>("manipulator_actv", 100);
 ros::Publisher pub_sentq = nh.advertise<manipulator::nine>("manipulator_sentq", 100);

 namespace gazebo {
    class ROSmanipulatorPlugin : public ModelPlugin
    {
        public : ROSmanipulatorPlugin()
        {
            //Start up ROS
            std::string name = "ros_model_plugin_node";
            int argc = 0;
            ros::init(argc, NULL, name);
        }

        public : ~ROSmanipulatorPlugin()
        {
            delete this->node;
        }

        public : void Load(physics::ModelPtr _parent, sdf::ElementPtr /*sdf*/)  //connect with gazebo
        {
            //Store the pointer to the model
            this->model = _parent;

            //ROS Nodehandle
            this->node = new ros::NodeHandle("~");

            //Getting joint angle
            this->joint1_ = this->model->GetJoint("J1");

            //Store the joint controller to control joint
            this->joint1_Controller_ = this->model->GetJointController();

            //Subscribe for position, velocity and torque of manipulation
            this->sub_jointp = this->node->subscribe<manipulator::three>("manipulator_jointp", 100, &ROSmanipulatorPlugin::ROSCallbackp, this);
            this->sub_jointt = this->node->subscribe<manipulator::three>("manipulator_jointt", 100, &ROSmanipulatorPlugin::ROSCallbackt, this);
            this->sub_jointv = this->node->subscribe<manipulator::three>("manipulator_jointv", 100, &ROSmanipulatorPlugin::ROSCallbackv, this);

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ROSmanipulatorPlugin::OnUpdate, this));
        }

        // Called by the world update start event
        public: void OnUpdate()
        {
            //Getting joint position
            manipulator::three actp;

            actp.a = this->joint1_->Position(1); //for rotation axes, the values in radians

            pub_actp.publish(actp);


            //Getting joint velocity
            manipulator::three actv;

            actv.a = this->joint1_->GetVelocity(1);

            pub_actv.publish(actv);

            //Getting joint torque
            physics::JointWrench joint1ft = this->joint1_->GetForceTorque(0); //force

            ignition::math::Vector3d j1torque = joint1ft.body2Torque; // torque
                                                                      //Where JointWrench.body1Force contains the force applied by the parent Link on the Joint specified in the parent Link frame,
                                                                      //and JointWrench.body2Force contains the force applied by the child Link on the Joint specified in the child Link frame.

            manipulator::nine sentq;
            sentq.a = j1torque.X();
            sentq.b = j1torque.Y();
            sentq.c = j1torque.Z();

            pub_sentq.publish(sentq);
            ros::spinOnce;
        }
/////////////////////////////////////////////////////////////////////////////////////////////////////
        void ROSCallbackp(const manipulator::three::ConstPtr& position)
        {// when call it it will move joint position
            this->joint1_Controller_->SetJointPosition(joint1_, position->a, 0);
        }

        void ROSCallbackt(const manipulator::three::ConstPtr& torque)
        {
            this->joint1_->SetForce(1, torque->a);
        }

        void ROSCallbackv(const manipulator::three::ConstPtr& velocity)
        {
            this->joint1_->SetVelocity(0, velocity->a);
        }
/////////////////////////////////////////////////////////////////////////////////////////////////////

        // Pointer to the model
        private: physics::ModelPtr model;

        //Pointer to the joint
        private: physics::JointPtr joint1_;

        //Pointer to the joint controller
        private: physics::JointControllerPtr joint1_Controller_;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;

        // ROS Nodehandle
        private: ros::NodeHandle* node;

        //ROS Subscriber
        ros::Subscriber sub_jointp;
        ros::Subscriber sub_jointt;
        ros::Subscriber sub_jointv;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ROSmanipulatorPlugin)
 }
