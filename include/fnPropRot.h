/** 
 *  Copyright (c) 2021 Iason Batzianoulis
 *  
 * 
 *  Developer:  Iason Batzianoulis
 *  email:      iasonbatz@gmail.com
 *  License:    GNU GPL v3
 * 
 *  The objects of this class run a ros-node that listens to the contact status on the fingertips from bioTac sensors and the high-level command comming from an EMG decoder 
 *  for computing the torques on the fingertips for holding an object and/or apply a rotation to it. No control-loop is implemented on the generation of torques making this
 *  controller an open-loop controller. The main assumption of the contoller is that the object to grasp/rotate is a cuboid and the index and middle fingers approach the 
 *  object from the opposite object's side/face than the thumb.
 * 
 *  The initial and final poses of the fingertips, the number of fingertips equipped with the bioTac sensors, the maximum magnitude of the rotational torque and the applied
 *  force on each side of the object are defined in the file prop_rot_params.yaml, which loaded from the launcher and read from the node's initialization. 
 * 
 *  The ros-node has three subscribers to:
 *      1. listen to the pose of the fingertips (ros-message: geometry_msgs::PoseArray)
 *      2. listen to the contact status from the bioTac sensors (ros-message: biotac_interface::biotacMsg)
 *      3. listen to the high-level commands from an EMG-decoder (ros-message: emg_interface::emg_msg)
 * 
 *  The ros-node has two publishers to:
 *      1. publish the desired pose of the fingertips
 *      2. publish the desired torques for the fingertips
 *
 *  Regarding the bioTac message, the node acquires only the values of the boolean variables bioTac# that indicate the contact status (true: contact, false:no contact) and
 *  omits the pDC variable.
 *  Regarding the emg message, the node acquires the variable Fingers_Direction, which indicates command ( open(extend)/close(flex) ) for each finger (-1: open, 0: stop, 1: close,
 *  order of the finger is index, middle, pinky and thumb), and the variable Gain, the sign of it indicates the direction of rotation (>0 clockwise, <0 counter-clockwise) and the 
 *  absolute value of it corresponds to the percentage of the maximum magnitude of the rotational torque of the object. Hence, a greater absolute value of the Gain would mean a 
 *  greater magnintute of torque and a greater rotation angle.
 * 
 *  Once an emg-command is received, the fingertips would move towards their initial or final pose, as it is defined in the yaml file, according to the value of the finger's direction.
 *  If a contact is detected on a fingertip, the correspoding finger will stop moving. Note, the pinky finger is a slave to the contact status of the middle finger, meaning that if
 *  a contact is detected on the middle fingertip, both the middle finger and the pinky finger will stop moving. 
 * 
 *  If all the sensorized fingertips are in contact will the object and no rotation command is received (i.e., the Gain variable of the emg message is zero), the fingers will attemp 
 *  to press the object in the hope to secure a safe grasp. If a rotation command is received, the controller will generate the forces on the fingertips in order to rotate the object
 *  according to the value of the Gain vairable of the emg message as mentioned above.
 * 
 * 
**/


#ifndef __FNPROPROT_H__
#define __FNPROPROT_H__

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
// #include "fingers_controller/fingers_cmd.h"
#include "biotac_interface/biotacMsg.h"
#include "emg_interface/emg_msg.h"


#include <string>
#include <vector>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <signal.h>
#include <mutex>
#include <iostream>



class fnPropRot{

    int nb_fingers;                                         // number of fingers of the hand
    int nb_ft_tactile_sensors;                              // number of fingertips that have tactile sensors

    float max_torque;                                       // maximum absolute torque of the fingertips, in case of rotation

    float a_force;                                          // absolute force to be apply on each side of the object
    bool first_time_contact, update_vectors;                // indicators of the first time that all the fingertips are in contact with the object and if updating the vectors is required

    std::vector<Eigen::VectorXd> ftInitPosition;            // initial position of the fingertips
    std::vector<Eigen::VectorXd> ftInitOrientation;         // initial orientation of the fingertips

    std::vector<Eigen::VectorXd> ftFinalPosition;           // final position of the fingertips
    std::vector<Eigen::VectorXd> ftFinalOrientation;        // final orientation of the fingertips

    std::vector<Eigen::Vector3d> fingersCurrPosition;       // current fingertips position
    std::vector<Eigen::Vector4d> fingersCurrOrientation;    // current fingertips orientation

    std::vector<Eigen::Vector3d> targetsPosition;           // target position of the fingertips
    std::vector<Eigen::Vector4d> targetsOrientation;        // target orientation of the fingertips

    std::vector<Eigen::Vector3d> desiredTrq;                // desired torques of the fingertips

    
    Eigen::Vector3d IM;                                     // index-middle fingertips vector 
    Eigen::Vector3d IT;                                     // index-thumb fingertips vector
    Eigen::Vector3d projIT;                                 // projection of IT on the IM
    Eigen::Vector3d t_cm;                                   // centroid of the triangle described by the three fingertips (index, middle and thumb)
    Eigen::Vector3d cmT;                                    // centroid - thumb fingertip vector
    Eigen::Vector3d cmI;                                    // centroid - index fingertip vector
    Eigen::Vector3d cmM;                                    // centroid - middle fingertip vector
    
    float d1;                                               // norm of the projection
     

    std::vector<bool> ft_contact;                           // a vector to contain the contact occurrence on the fingertips 
    std::vector<float> external_cmd;                        // a vector to contain the external commands

    ros::NodeHandle nh;                                     // ros node handler
    ros::Rate loopRate;                                     // loop rate (Hz)

    // ros publishers
    ros::Publisher ft_pose_pub;                             // publisher of the desired fingers pose (position + orientation)
    ros::Publisher ft_trq_pub;                              // publisher of the desired torque

    // ros messages
    geometry_msgs::PoseArray pose_msg;                      // message for the desired pose of the fingertips
    geometry_msgs::PoseArray trq_msg;                       // message for the desired torque of the fingertips

    // ros subscribers
    ros::Subscriber fingers_pose_sub;                       // listener of the current pose of the fingers
    ros::Subscriber biotac_sub;                             // listener of the biotac sensors on the fingertips, it expects a vector of 3 integers each one corresponds to the detection of contact for each fingertip (0: no contact, 1:contact) 
    ros::Subscriber ext_cmd_sub;                            // listener of external commands for the action of the fingers (e.g., commands from an EMG decoder)

    double initTime;                                        // ros time when the node started
    bool n_stop;                                            // checking for CTRL+C event

    std::mutex th_mutex;                                    // mutex variable for controlling the message publishing

    static fnPropRot* me;                                   // Pointer on the instance


    // callback functions

    void updateFingersPose(const geometry_msgs::PoseArray& msg);            // callback function for updating the current pose of the fingertips
    // void updateExternalCmd(const fingers_contoller::fingers_cmd &msg);      // callback function for updating the command to the fingers 
    void updateExternalCmd(const emg_interface::emg_msg &msg);              // callback function for updating the command to the fingers 
    void updateContactStatus(const biotac_interface::biotacMsg &msg);       // collback fucntion for updating the constact status

	static void stopNodeCallback(int sig);                                  // callback function for stopping the node

    // operation methods
    int parse_params();                                                     // parsing parameters from yaml file
    int setDesPos();                                                        // setting the desired pose of the fingertips 
    int setFtPose(int ft_nb);                                               // setting the desired pose of the ft_nb fingertip according to the external commands and the input fron the tactile sensors
    int setDesTrq();                                                        // setting the desired torque to the fingertips according to the external commands and the input fron the tactile sensors

    int publish();                                                          // publishing the messages

    Eigen::Vector4d ToQuaternion(double yaw, double pitch, double roll);    // convertion form Euler angles to Quartenions, yaw (Z-axis), pitch (Y-axis), roll (X-axis)

    public:

    // fnPropRot();                                                        // empty constructor
    fnPropRot(ros::NodeHandle &n, double frequency);                        // constructor accepting the ros handler and the loop rate

    bool init();                                                            // initialization of the node

    int run();                                                              // running the methods of the node in a loop

    ~fnPropRot(){};                                                         // destructor

};


#endif