

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <iostream>


std::vector<Eigen::Vector3d> _fingersPosition;       // desired target position
std::vector<Eigen::Vector4d> _fingersOrientation;    // desired target orientation



void updateFingersPose(const geometry_msgs::PoseArray& msg){

    /*
    *  callback function for listening to the desired pose of the fingertips
    *  
    */
//    std::cout<< "in "
    for (int fngr=0; fngr<(int)_fingersPosition.size(); fngr++){
        for (int i=0; i< msg.poses.size(); i++){
            _fingersPosition[fngr] << msg.poses[fngr].position.x, msg.poses[fngr].position.y, msg.poses[fngr].position.z;
            _fingersOrientation[fngr] << msg.poses[fngr].orientation.w, msg.poses[fngr].orientation.x, msg.poses[fngr].orientation.y, msg.poses[fngr].orientation.z;
        }
    }

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


int main(int argc, char** argv){


    _fingersPosition = std::vector<Eigen::Vector3d>(4);
    _fingersOrientation = std::vector<Eigen::Vector4d>(4);

    ros::init(argc, argv, "fingers_controller");

    ros::NodeHandle _n;

    ros::Publisher fingers_pub = _n.advertise<geometry_msgs::PoseArray>("/allegroHand_0/ft_pose_cmd", 1);
    ros::Subscriber fingers_sub = _n.subscribe("/allegroHand_0/ft_pos", 1,  updateFingersPose);

    // open position
    Eigen::Vector3d index_target_pos(0.00, -0.054, 0.227);
    Eigen::Vector3d middle_target_pos(0.027, -0.00, 0.22);
    Eigen::Vector3d ring_target_pos(0.022, 0.053, 0.22);
    Eigen::Vector3d thumb_target_pos(0.022, 0.053, 0.22);

    // detect position
    // Eigen::Vector3d index_target_pos(0.11, -0.067, 0.12);

    // Eigen::Vector3d middle_target_pos(0.11, -0.012, 0.098);
    // Eigen::Vector3d ring_target_pos(0.098, 0.026, 0.073);
    // Eigen::Vector3d thumb_target_pos(0.022, 0.053, 0.22);
    
    std::vector<Eigen::Vector3d> desiredPos(4);
    desiredPos[0] = index_target_pos;
    desiredPos[1] = middle_target_pos;
    desiredPos[2] = ring_target_pos;
    desiredPos[3] = thumb_target_pos;

    // Eigen::Vector3d index_target_pos(0.06, -0.052, 0.144);

    // std::cout << "test\n";

    ros::Rate loop_rate(100);

    while(ros::ok()){


        geometry_msgs::PoseArray pub_msg = getdesiredPose(desiredPos);

        fingers_pub.publish(pub_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}