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
//oh my third commit
//vscode communication
//my practice

float current_joint_position[3];
float current_joint_velocity[3];
float current_joint_torque[9];

void msgCallbackP(const manipulator::three::ConstPtr& msg)
{
    current_joint_position[0] = msg->a;
    current_joint_position[1] = msg->b;
    current_joint_position[2] = msg->c;
}

void msgCallbackV(const manipulator::three::ConstPtr& msg)
{
    current_joint_velocity[0] = msg->a;
    current_joint_velocity[1] = msg->b;
    current_joint_velocity[2] = msg->c;
}

void msgCallbackT(const manipulator::nine::ConstPtr& msg)
{
    current_joint_torque[0] = msg->a;
    current_joint_torque[1] = msg->b;
    current_joint_torque[2] = msg->c;
    current_joint_torque[3] = msg->d;
    current_joint_torque[4] = msg->e;
    current_joint_torque[5] = msg->f;
    current_joint_torque[6] = msg->g;
    current_joint_torque[7] = msg->h;
    current_joint_torque[8] = msg->i;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    ros::Publisher pub_jointp = nh.advertise<manipulator::three>("gazebo/manipulator_jointp", 100);
    ros::Publisher pub_jointv = nh.advertise<manipulator::three>("gazebo/manipulator_jointv", 100);
    ros::Publisher pub_jointt = nh.advertise<manipulator::three>("gazebo/manipulator_jointt", 100);
    ros::Subscriber sub_actp = nh.subscribe("manipulator_actp", 100, msgCallbackP);
    ros::Subscriber sub_actv = nh.subscribe("manipulator_actp", 100, msgCallbackV);
    ros::Subscriber sub_sentq = nh.subscribe("manipulator_sentq", 100, msgCallbackT);

    int case_;
    float pos_1, vel_1, tq_1;
    manipulator::three position, velocity, torque;
    while(ros::ok())
    {
      printf("1: position control //  2: velocity control  // 3: torque control: ");
      scanf("%d", &case_);
      if(case_ == 1){
          printf("position(degree): ");
          scanf("%f", &pos_1);
          pos_1 = pos_1*pi/180.0;

          position.a = pos_1;
          position.b = 0;
          position.c = 0;
          pub_jointp.publish(position);
      }
      else if(case_ == 2)
      {
          printf("velocity(degree/s): ");
          scanf("%f", &vel_1);
          vel_1 = vel_1*pi/180.0;

          velocity.a = vel_1;
          velocity.b = 0;
          velocity.c = 0;
          pub_jointv.publish(velocity);
      }
      else if(case_ == 3)
      {
          printf("torque(Nm): ");
          scanf("%f", &tq_1);

          torque.a = tq_1;
          torque.b = 0;
          torque.c = 0;
          pub_jointt.publish(torque);
      }
      else
      {
          printf("Error\n");
      }
     ros::spinOnce();
    }
}
