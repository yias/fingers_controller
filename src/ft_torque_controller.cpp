

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

#include <thread>
#include <mutex>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/select.h>



const double T_PI = 3.14159265359;

std::vector<Eigen::Vector3d> _fingersPosition;       // desired target position
std::vector<Eigen::Vector4d> _fingersOrientation;    // desired target orientation

int kb_choice = 0;
bool isRunning;
std::mutex threadMutex;

int kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}


void checkKeyBoard(){
    
    int key;

    while(isRunning){
        if (kbhit()){
            key = getch();
            switch (key)
            {
            case 'o':
                kb_choice = 0;
                break;
            case 'c':
                kb_choice = 1;
                break;
            default:
                std::cout << "\nNot available choice!! \nType 'o' for \"zero torque\" or 'c' to apply torque " << std::endl;
                break;
            }
            std::cout << std::endl;
        }
    }
}


void updateFingersPose(const geometry_msgs::PoseArray& msg){

    /*
    *  callback function for listening to the real pose of the fingertips
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



geometry_msgs::PoseArray setDesiredPose(std::vector<Eigen::Vector3d> desPos, std::vector<Eigen::Vector4d> desOrient){
    geometry_msgs::PoseArray msg;

    msg.poses.resize(_fingersPosition.size());

    // std::vector<Eigen::Vector3d> _tmp_fingersPosition = _fingersPosition;
    // std::vector<Eigen::Vector4d> _tmp_fingersOrientation = _fingersOrientation;

    // _tmp_fingersPosition[0] = desPos[0];
    // _tmp_fingersPosition[1] = desPos[1];
    // _tmp_fingersPosition[2] = desPos[2];
    // _tmp_fingersPosition[3] = desPos[3];

    // _tmp_fingersOrientation[0] = desOrient[0];
    // _tmp_fingersOrientation[1] = desOrient[1];
    // _tmp_fingersOrientation[2] = desOrient[2];
    // _tmp_fingersOrientation[3] = desOrient[3];

    for (size_t fngr=0; fngr<_fingersPosition.size(); fngr++){

        // assign the position and orientation values to the message
        msg.poses[fngr].position.x = desPos[fngr](0);
        msg.poses[fngr].position.y = desPos[fngr](1);
        msg.poses[fngr].position.z = desPos[fngr](2);

        msg.poses[fngr].orientation.w = desOrient[fngr](0);
        msg.poses[fngr].orientation.x = desOrient[fngr](1);
        msg.poses[fngr].orientation.y = desOrient[fngr](2);
        msg.poses[fngr].orientation.z = desOrient[fngr](3);
    }

    return msg;
}

Eigen::Vector4d ToQuaternion(double yaw, double pitch, double roll) // yaw (Z-axis), pitch (Y-axis), roll (X-axis)
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


    std::thread kbThread;                       // the thread for getting input from keyboard

    _fingersPosition = std::vector<Eigen::Vector3d>(4);
    _fingersOrientation = std::vector<Eigen::Vector4d>(4);

    ros::init(argc, argv, "fingers_position_controller_example");

    ros::NodeHandle _n;

    ros::Publisher fingers_pub = _n.advertise<geometry_msgs::PoseArray>("/allegroHand_0/ft_torque_cmd", 1);
    ros::Subscriber fingers_sub = _n.subscribe("/allegroHand_0/ft_pose", 1,  updateFingersPose);

    geometry_msgs::PoseArray pub_msg;


    // open position
    Eigen::Vector3d index_torque(0.0, 0.0, 0.0);
    Eigen::Vector4d index_target_orient = ToQuaternion(0.0, 0.0, 0.0);

    Eigen::Vector3d middle_torque(0.0, 0.0, 0.0);
    Eigen::Vector4d middle_target_orient = ToQuaternion(0.0, 0.0, 0.0);
    
    Eigen::Vector3d ring_torque(0.0, 0.0, 0.0);
    Eigen::Vector4d ring_target_orient = ToQuaternion(0.0, 0.0, 0.0);
    
    Eigen::Vector3d thumb_torque(0.0, 0.0, 0.0);
    Eigen::Vector4d thumb_target_orient = ToQuaternion(0.0, 0.0, 0.0);

    std::vector<Eigen::Vector3d> zero_torque(4);
    zero_torque[0] = index_torque;
    zero_torque[1] = middle_torque;
    zero_torque[2] = ring_torque;
    zero_torque[3] = thumb_torque;

    std::vector<Eigen::Vector4d> open_orientation(4);
    open_orientation[0] = index_target_orient;
    open_orientation[1] = middle_target_orient;
    open_orientation[2] = ring_target_orient;
    open_orientation[3] = thumb_target_orient;

    // close configuration position
    index_torque = Eigen::Vector3d(-0.30, 0.2, 0.0);
    index_target_orient = ToQuaternion(0.0, 0.0, 0.0);

    middle_torque = Eigen::Vector3d(0.0, 0.0, 0.0);
    middle_target_orient = ToQuaternion(0.0, 0.0, 0.0);

    ring_torque = Eigen::Vector3d(0.0, 0.0, 0.0);
    ring_target_orient = ToQuaternion(0.0, 0.0, 0.0);
    
    thumb_torque = Eigen::Vector3d(0.0, 0.0, 0.0);
    thumb_target_orient = ToQuaternion(0.0, 0.0, 0.0);

    std::vector<Eigen::Vector3d> desired_torque(4);
    desired_torque[0] = index_torque;
    desired_torque[1] = middle_torque;
    desired_torque[2] = ring_torque;
    desired_torque[3] = thumb_torque;
    
    std::vector<Eigen::Vector4d> detect_orientation(4);
    detect_orientation[0] = index_target_orient;
    detect_orientation[1] = middle_target_orient;
    detect_orientation[2] = ring_target_orient;
    detect_orientation[3] = thumb_target_orient;


    // std::vector<Eigen::Vector3d> desiredPos = open_position;
    // desiredPos = detect_position;

    // std::vector<Eigen::Vector4d> desiredOrient = open_orientation;
    // desiredOrient=detect_orientation;

    ros::Rate loop_rate(100);

    isRunning = true;
    kbThread = std::thread(&checkKeyBoard);

    std::cout << "Type 'o' for \"zero torque\" or 'c' to apply torque " << std::endl;

    while(ros::ok()){
        
        if (kb_choice == 0){
            pub_msg = setDesiredPose(zero_torque, open_orientation);
        }
        if (kb_choice == 1){
            pub_msg = setDesiredPose(desired_torque, detect_orientation);
        }

        fingers_pub.publish(pub_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    isRunning = false;
    if(kbThread.joinable()){
        kbThread.join();
    }


    return 0;
}