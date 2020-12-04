#include <ros/ros.h>
#include "sensor_msgs/JointState.h"

#define OBJ_DOF 3

int main(int argc, char** argv) {

  ros::init(argc, argv, "obj_motion_generator");

  ros::NodeHandle _n;

  ros::Publisher object_js_pub = _n.advertise<sensor_msgs::JointState>("joint_states", 10);

  sensor_msgs::JointState obj_joint_states;

  obj_joint_states.position.resize(OBJ_DOF);
  obj_joint_states.velocity.resize(OBJ_DOF);
  obj_joint_states.effort.resize(OBJ_DOF);
  obj_joint_states.name.resize(OBJ_DOF);

  for(size_t i = 0; i < OBJ_DOF; i++){
    obj_joint_states.position[i] = 0;
    obj_joint_states.velocity[i] = 0;
    obj_joint_states.effort[i] = 0;
    
  }
  
  obj_joint_states.name[0] = "joint_x";
  obj_joint_states.name[1] = "joint_y";
  obj_joint_states.name[2] = "joint_z";

  ros::Rate loop_rate(100);

  while(ros::ok()){

    obj_joint_states.header.stamp = ros::Time::now();

    object_js_pub.publish(obj_joint_states);
    // obj_joint_states.position[0] += 0.003;
    obj_joint_states.position[1] += 0.003;
    // obj_joint_states.position[2] += 0.003;

    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;
}