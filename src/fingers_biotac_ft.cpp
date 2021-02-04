/** 
 *  Copyright (c) 2020 Iason Batzianoulis  
 * 
 *  Developer:  Iason Batzianoulis
 *  email:      iasonbatz@gmail.com
 *  License:    GNU GPL v3
 * 
**/

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Float64.h"
#include <biotac_interface/biotacMsg.h>

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <iostream>


std::vector<Eigen::Vector3d> _fingersPosition;       // desired target position
std::vector<Eigen::Vector4d> _fingersOrientation;    // desired target orientation
Eigen::Vector3d _force;
Eigen::Vector3d _torque;

Eigen::Vector4d _position_errors;

double accepted_position_error = 0.05;
double deg_step = 1;
double finger_length = 0.135;


void updateFingersPose(const geometry_msgs::PoseArray& msg){

    /*
    *  callback function for listening to the desired pose of the fingertips
    *  
    */

    for (int fngr=0; fngr<(int)_fingersPosition.size(); fngr++){
        for (int i=0; i< msg.poses.size(); i++){
            _fingersPosition[fngr] << msg.poses[fngr].position.x, msg.poses[fngr].position.y, msg.poses[fngr].position.z;
            _fingersOrientation[fngr] << msg.poses[fngr].orientation.w, msg.poses[fngr].orientation.x, msg.poses[fngr].orientation.y, msg.poses[fngr].orientation.z;
        }
    }

}


void updateFT(const geometry_msgs::WrenchStamped &msg){
    _force << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z;

    _torque << msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z;
}




geometry_msgs::PoseArray getdesiredPose(std::vector<Eigen::Vector3d> desPose){
    geometry_msgs::PoseArray msg;

    msg.poses.resize(_fingersPosition.size());

    std::vector<Eigen::Vector3d> _tmp_fingersPosition = _fingersPosition;
    std::vector<Eigen::Vector4d> _tmp_fingersOrientation = _fingersOrientation;

    _tmp_fingersPosition[0] = desPose[0];
    _tmp_fingersPosition[1] = desPose[1];
    _tmp_fingersPosition[2] = desPose[2];


    for (size_t fngr=0; fngr<_fingersPosition.size(); fngr++){

        _position_errors[fngr] = (_fingersPosition[fngr] - desPose[fngr]).norm();

        // assign the position and orientation values to the message
        msg.poses[fngr].position.x = _tmp_fingersPosition[fngr](0);
        msg.poses[fngr].position.y = _tmp_fingersPosition[fngr](1);
        msg.poses[fngr].position.z = _tmp_fingersPosition[fngr](2);

        msg.poses[fngr].orientation.w = _fingersOrientation[fngr](0);
        msg.poses[fngr].orientation.x = _fingersOrientation[fngr](1);
        msg.poses[fngr].orientation.y = _fingersOrientation[fngr](2);
        msg.poses[fngr].orientation.z = _fingersOrientation[fngr](3);
    }

    return msg;
}


geometry_msgs::PoseArray getdesiredTorque(std::vector<Eigen::Vector3d> desTorq){
    geometry_msgs::PoseArray msg;

    msg.poses.resize(_fingersPosition.size());

    for (size_t fngr=0; fngr<_fingersPosition.size(); fngr++){

        // assign the position and orientation values to the message
        msg.poses[fngr].position.x = desTorq[fngr](0);
        msg.poses[fngr].position.y = desTorq[fngr](1);
        msg.poses[fngr].position.z = desTorq[fngr](2);
    }

    return msg;
}


bool checkPositionErrors(){
    bool t_r = true;
    for (size_t i=0; i<_position_errors.size(); i++){
        if (_position_errors[i]>accepted_position_error){
            t_r = t_r && false;
        }
    }
    return t_r;
}


int main(int argc, char** argv){


    _fingersPosition = std::vector<Eigen::Vector3d>(4);
    _fingersOrientation = std::vector<Eigen::Vector4d>(4);

    ros::init(argc, argv, "fingers_controller");

    ros::NodeHandle _n;

    ros::Publisher fingers_pub = _n.advertise<geometry_msgs::PoseArray>("/allegroHand_0/ft_pose_cmd", 1);
    ros::Publisher fingers_torque_pub = _n.advertise<geometry_msgs::PoseArray>("/allegroHand_0/ft_forque_cmd", 1);
    ros::Publisher force_pub = _n.advertise<std_msgs::Float64>("/ft_sensors/force", 1);
    ros::Publisher torque_pub = _n.advertise<std_msgs::Float64>("/ft_sensors/torque", 1);

    ros::Subscriber fingers_sub = _n.subscribe("/allegroHand_0/ft_pose", 1,  updateFingersPose);
    ros::Subscriber ft_sub = _n.subscribe("/ft_sensor/netft_data", 1,  updateFT);
    // ros::Subscriber ft_bias_sub = _n.subscribe()

    // open position
    Eigen::Vector3d open_index_target_pos(0.00, -0.054, 0.227);
    Eigen::Vector3d open_middle_target_pos(0.027, -0.00, 0.22);
    Eigen::Vector3d open_ring_target_pos(0.022, 0.053, 0.22);
    Eigen::Vector3d open_thumb_target_pos(0.022, 0.053, 0.22);

    // detect position
    // Eigen::Vector3d index_target_pos(0.11, -0.067, 0.12);

    // Eigen::Vector3d middle_target_pos(0.11, -0.012, 0.098);
    // Eigen::Vector3d ring_target_pos(0.098, 0.026, 0.073);
    // Eigen::Vector3d thumb_target_pos(0.022, 0.053, 0.22);
    
    std::vector<Eigen::Vector3d> open_desiredPos(4);
    open_desiredPos[0] = open_index_target_pos;
    open_desiredPos[1] = open_middle_target_pos;
    open_desiredPos[2] = open_ring_target_pos;
    open_desiredPos[3] = open_thumb_target_pos;

    std::vector<Eigen::Vector3d> desiredPos(4);

    desiredPos = open_desiredPos;

    std::vector<Eigen::Vector3d> appliedForce(4);
    std::vector<Eigen::Vector3d> zeroForce(4);

    appliedForce[0] = Eigen::Vector3d::Zero();
    appliedForce[1] = Eigen::Vector3d::Zero();
    appliedForce[2] = Eigen::Vector3d::Zero();
    appliedForce[3] = Eigen::Vector3d::Zero();

    zeroForce = appliedForce;

    // Eigen::Vector3d index_target_pos(0.06, -0.052, 0.144);

    // std::cout << "test\n";

    int c_con = 0;
    int step_counter = 0; 

    ros::Rate loop_rate(100);

    while(ros::ok()){


        geometry_msgs::PoseArray pub_msg, pub_torq_msg;

        switch (c_con)
        {
        case 0:
            pub_msg = getdesiredPose(open_desiredPos);
            if (!checkPositionErrors()){
                c_con++;
            }
            break;

        case 1:
        {
            Eigen::Vector3d tt(finger_length*sin((90-step_counter*deg_step)*M_PI/180), _fingersPosition[0](1), finger_length*cos((90-step_counter*deg_step)*M_PI/180));
            // std::cout << "tt: " << tt.transpose() << std::endl;
            desiredPos[0] = tt;
            pub_msg = getdesiredPose(desiredPos);
            if (_force.norm()>0.5){
                step_counter = 0;
                c_con++;
            }
            break;
        }

        case 2:
        {
            Eigen::Vector3d tt = _force;
            tt(0) += 0.2;
            std::cout << "tt: " << tt.transpose() << std::endl;
            appliedForce[0]= tt;
            if (_force.norm()>10){
                c_con++;
            }
            break;
        }

        default:
            pub_msg = getdesiredPose(open_desiredPos);
            pub_torq_msg = getdesiredTorque(zeroForce);
            break;
        }
        // if (!checkPositionErrors()){
        //     pub_msg = getdesiredPose(open_desiredPos);
        // }else{
        //     desiredPos[0] = open_index_target_pos;
            
        //     pub_msg = getdesiredPose(open_desiredPos);
        // }
        


        std_msgs::Float64 force_msg, torque_msg;
        force_msg.data = _force.norm();
        torque_msg.data = _torque.norm();
        
        std::cout << c_con << ": " << _position_errors.transpose() << std::endl;

        fingers_pub.publish(pub_msg);
        force_pub.publish(force_msg);
        torque_pub.publish(torque_msg);
        fingers_torque_pub.publish(pub_torq_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}