

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



geometry_msgs::PoseArray getdesiredPose(std::vector<Eigen::Vector3d> desPose, std::vector<Eigen::Vector4d> desOrient){
    geometry_msgs::PoseArray msg;

    msg.poses.resize(_fingersPosition.size());

    std::vector<Eigen::Vector3d> _tmp_fingersPosition = _fingersPosition;
    std::vector<Eigen::Vector4d> _tmp_fingersOrientation = _fingersOrientation;

    _tmp_fingersPosition[0] = desPose[0];
    _tmp_fingersPosition[1] = desPose[1];
    _tmp_fingersPosition[2] = desPose[2];
    _tmp_fingersPosition[3] = desPose[3];

    _tmp_fingersOrientation[0] = desOrient[0];
    _tmp_fingersOrientation[1] = desOrient[1];
    _tmp_fingersOrientation[2] = desOrient[2];
    _tmp_fingersOrientation[3] = desOrient[3];

    for (size_t fngr=0; fngr<_fingersPosition.size(); fngr++){

        // assign the position and orientation values to the message
        msg.poses[fngr].position.x = _tmp_fingersPosition[fngr](0);
        msg.poses[fngr].position.y = _tmp_fingersPosition[fngr](1);
        msg.poses[fngr].position.z = _tmp_fingersPosition[fngr](2);

        msg.poses[fngr].orientation.w = _tmp_fingersOrientation[fngr](0);
        msg.poses[fngr].orientation.x = _tmp_fingersOrientation[fngr](1);
        msg.poses[fngr].orientation.y = _tmp_fingersOrientation[fngr](2);
        msg.poses[fngr].orientation.z = _tmp_fingersOrientation[fngr](3);
    }

    return msg;
}

Eigen::Vector4d ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Eigen::Vector4d q;
    double w = cr * cp * cy + sr * sp * sy;
    double x = sr * cp * cy - cr * sp * sy;
    double y = cr * sp * cy + sr * cp * sy;
    double z = cr * cp * sy - sr * sp * cy;
    q << w, x, y, z;
    return q;
}


int main(int argc, char** argv){


    _fingersPosition = std::vector<Eigen::Vector3d>(4);
    _fingersOrientation = std::vector<Eigen::Vector4d>(4);

    ros::init(argc, argv, "fingers_controller");

    ros::NodeHandle _n;

    ros::Publisher fingers_pub = _n.advertise<geometry_msgs::PoseArray>("/allegroHand_0/ft_pose_cmd", 1);
    ros::Subscriber fingers_sub = _n.subscribe("/allegroHand_0/ft_pos", 1,  updateFingersPose);

    // open position
    Eigen::Vector3d index_target_pos(0.05, -0.054, 0.21);
    Eigen::Vector4d index_target_orient = ToQuaternion(0.0, 1.0, 0.0);

    Eigen::Vector3d middle_target_pos(0.05, -0.00, 0.21);
    Eigen::Vector4d middle_target_orient = ToQuaternion(0.0, 1.0, 0.0);
    
    Eigen::Vector3d ring_target_pos(0.05, 0.053, 0.21);
    Eigen::Vector4d ring_target_orient = ToQuaternion(0.0, 1.0, 0.0);
    
    Eigen::Vector3d thumb_target_pos(0.09, -0.10, 0.05);
    Eigen::Vector4d thumb_target_orient = ToQuaternion(0.0, 0.0, 0.0);

    std::vector<Eigen::Vector3d> open_position(4);
    open_position[0] = index_target_pos;
    open_position[1] = middle_target_pos;
    open_position[2] = ring_target_pos;
    open_position[3] = thumb_target_pos;

    std::vector<Eigen::Vector4d> open_orientation(4);
    open_orientation[0] = index_target_orient;
    open_orientation[1] = middle_target_orient;
    open_orientation[2] = ring_target_orient;
    open_orientation[3] = thumb_target_orient;

    // detect position
    index_target_pos = Eigen::Vector3d(0.10, -0.05, 0.14);
    index_target_orient = ToQuaternion(0.0, 120*3.14/180, 0.0);

    middle_target_pos = Eigen::Vector3d(0.08, 0.00, 0.15);
    middle_target_orient = ToQuaternion(0.0, 3.14/2, 0.0);

    ring_target_pos = Eigen::Vector3d(0.08, 0.05, 0.15);
    ring_target_orient = ToQuaternion(0.0, 90*3.14/180, 0.0);
    
    thumb_target_pos = Eigen::Vector3d(0.10, -0.05, 0.07);
    thumb_target_orient = ToQuaternion(0.0, -120*3.14/180, 3.14);

    std::vector<Eigen::Vector3d> detect_position(4);
    detect_position[0] = index_target_pos;
    detect_position[1] = middle_target_pos;
    detect_position[2] = ring_target_pos;
    detect_position[3] = thumb_target_pos;
    
    std::vector<Eigen::Vector4d> detect_orientation(4);
    detect_orientation[0] = index_target_orient;
    detect_orientation[1] = middle_target_orient;
    detect_orientation[2] = ring_target_orient;
    detect_orientation[3] = thumb_target_orient;


    std::vector<Eigen::Vector3d> desiredPos = open_position;
    desiredPos = detect_position;

    std::vector<Eigen::Vector4d> desiredOrient = open_orientation;
    desiredOrient=detect_orientation;

    // desiredPos[0] = index_target_pos;
    // desiredPos[1] = middle_target_pos;
    // desiredPos[2] = ring_target_pos;
    // desiredPos[3] = thumb_target_pos;

    // Eigen::Vector3d index_target_pos(0.06, -0.052, 0.144);

    // std::cout << "test\n";

    ros::Rate loop_rate(100);

    while(ros::ok()){


        geometry_msgs::PoseArray pub_msg = getdesiredPose(desiredPos, desiredOrient);

        fingers_pub.publish(pub_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}