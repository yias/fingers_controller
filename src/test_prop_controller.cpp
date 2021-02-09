/** 
 *  Copyright (c) 2021 Iason Batzianoulis
 *  
 * 
 *  Developer:  Iason Batzianoulis
 *  email:      iasonbatz@gmail.com
 *  License:    GNU GPL v3
 * 
**/

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "emg_interface/emg_msg.h"
#include "biotac_interface/biotacMsg.h"

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <numeric>
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
            case 'd':
                kb_choice = 2;
                break;
            case 'f':
                kb_choice = 3;
                break;
            case 'r':
                kb_choice = 4;
                break;
            default:
                std::cout << "\nNot available choice!! \nType 'o' for \"open\" or 'c' for \"close\" " << std::endl;
                break;
            }
            std::cout << std::endl;
        }
    }
}




int main(int argc, char** argv){



    ros::init(argc, argv, "fingers_position_controller_example");

    ros::NodeHandle _n;

    ros::Publisher emg_pub = _n.advertise<emg_interface::emg_msg>("/HandMaster/emg_interface", 1);
    ros::Publisher biotac_pub = _n.advertise<biotac_interface::biotacMsg>("/Handmanip/biotacInterface", 1);

    emg_interface::emg_msg pub_msg;
    biotac_interface::biotacMsg biotac_msg;

    pub_msg.Seq = 1;
    pub_msg.Rotation_Axis[0] = 1;
    pub_msg.Rotation_Axis[1] = 0;
    pub_msg.Rotation_Axis[2] = 0;
    pub_msg.Gain = 0;
    pub_msg.Fingers_Direction[0] = -1;
    pub_msg.Fingers_Direction[1] = -1;
    pub_msg.Fingers_Direction[2] = -1;
    pub_msg.Fingers_Direction[3] = -1;

    biotac_msg.bioTac1 = true;
    biotac_msg.bioTac2 = true;
    biotac_msg.bioTac3 = true;
    // biotac_msg.pDC = 0.0f;

    ros::Rate loop_rate(100);

    isRunning = true;
    std::thread kbThread;                       // the thread for getting input from keyboard
    kbThread = std::thread(&checkKeyBoard);

    std::cout << "Type 'o' for \"open\" or 'c' for \"close\" " << std::endl;

    while(ros::ok()){
        
        
        if (kb_choice == 0){
            pub_msg.Fingers_Direction[0] = -1;
            pub_msg.Fingers_Direction[1] = -1;
            pub_msg.Fingers_Direction[2] = -1;
            pub_msg.Fingers_Direction[3] = -1;
            biotac_msg.bioTac1 = false;
            biotac_msg.bioTac2 = false;
            biotac_msg.bioTac3 = false;
        }
        if (kb_choice == 1){
            pub_msg.Fingers_Direction[0] = 1;
            pub_msg.Fingers_Direction[1] = 1;
            pub_msg.Fingers_Direction[2] = 1;
            pub_msg.Fingers_Direction[3] = 1;
            biotac_msg.bioTac1 = false;
            biotac_msg.bioTac2 = false;
            biotac_msg.bioTac3 = false;
        }
        if (kb_choice == 2){
            pub_msg.Fingers_Direction[0] = 0;
            pub_msg.Fingers_Direction[1] = 0;
            pub_msg.Fingers_Direction[2] = 0;
            pub_msg.Fingers_Direction[3] = 0;
            biotac_msg.bioTac1 = false;
            biotac_msg.bioTac2 = false;
            biotac_msg.bioTac3 = false;
        }
        if (kb_choice == 3){
            pub_msg.Fingers_Direction[0] = 0;
            pub_msg.Fingers_Direction[1] = 0;
            pub_msg.Fingers_Direction[2] = 0;
            pub_msg.Fingers_Direction[3] = 0;
            pub_msg.Gain = 0.0;
            biotac_msg.bioTac1 = true;
            biotac_msg.bioTac2 = true;
            biotac_msg.bioTac3 = true;
        }
        if (kb_choice == 4){
            pub_msg.Fingers_Direction[0] = 0;
            pub_msg.Fingers_Direction[1] = 0;
            pub_msg.Fingers_Direction[2] = 0;
            pub_msg.Fingers_Direction[3] = 0;
            pub_msg.Gain = 1.0;
            biotac_msg.bioTac1 = true;
            biotac_msg.bioTac2 = true;
            biotac_msg.bioTac3 = true;
        }

        biotac_pub.publish(biotac_msg);

        emg_pub.publish(pub_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    isRunning = false;
    if(kbThread.joinable()){
        kbThread.join();
    }


    return 0;
}