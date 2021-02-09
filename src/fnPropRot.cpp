/** 
 *  Copyright (c) 2021 Iason Batzianoulis
 *  
 * 
 *  Developer:  Iason Batzianoulis
 *  email:      iasonbatz@gmail.com
 *  License:    GNU GPL v3
 * 
**/

#include "fnPropRot.h"

fnPropRot* fnPropRot::me = NULL;

const double T_PI = 3.14159265359;


fnPropRot::fnPropRot(ros::NodeHandle &n, double frequency):nh(n), loopRate(frequency){

    max_torque = -1;
    nb_fingers = -1;
    nb_ft_tactile_sensors = -1;
    a_force = -1;
}

bool fnPropRot::init(){
    /**
     *  node initialization
     * 
     *  - parsing the parameters from the server
     *  - setting the size of the vectors according to the number of fingers
     *  - settinp up the ros publishers, subcribers and messages
     *
     *  outputs:
     *      true if everything is done properly, false otherwise 
     * 
     */

    // parsing the parameters from the parameter server
    if (parse_params()<0){
        std::cout << "[proportional rotation node] Correct the parameters in the yaml file" << std::endl;
        return false;
    }


    // setting the size of vectors
    fingersCurrPosition = std::vector<Eigen::Vector3d>(nb_fingers);
    fingersCurrOrientation = std::vector<Eigen::Vector4d>(nb_fingers);

    targetsPosition = std::vector<Eigen::Vector3d>(nb_fingers);
    targetsOrientation = std::vector<Eigen::Vector4d>(nb_fingers);

    desiredTrq = std::vector<Eigen::Vector3d>(nb_fingers);

    ft_contact = std::vector<bool>(nb_ft_tactile_sensors, false);
    external_cmd = std::vector<float>(nb_fingers + 1, 0);

    // setting up the publishers
    ft_pose_pub = nh.advertise<geometry_msgs::PoseArray>("/allegroHand_0/ft_pose_cmd", 1);
    ft_trq_pub = nh.advertise<geometry_msgs::PoseArray>("/allegroHand_0/ft_torque_cmd", 1);

    // setting up the subscribers
    fingers_pose_sub = nh.subscribe("/allegroHand_0/ft_pose", 1, &fnPropRot::updateFingersPose, this,ros::TransportHints().reliable().tcpNoDelay());
    ext_cmd_sub = nh.subscribe("/HandMaster/emg_interface", 1, &fnPropRot::updateExternalCmd, this,ros::TransportHints().reliable().tcpNoDelay());
    biotac_sub = nh.subscribe("/Handmanip/biotacInterface", 1, &fnPropRot::updateContactStatus, this,ros::TransportHints().reliable().tcpNoDelay());

    // setting the size of poses of the messages
    pose_msg.poses.resize(nb_fingers);
    trq_msg.poses.resize(nb_fingers);


    // signal(SIGINT, fnPropRot::stopNodeCallback);

    first_time_contact = false;
    update_vectors = false;

    n_stop = false;

    return true;
    
}

int fnPropRot::parse_params(){


    // set the names of the ros-parameters
    std::vector<std::string> paramNames(7);
    paramNames[0] = "ft_init_positions";
    paramNames[1] = "ft_init_orientations";
    paramNames[2] = "ft_final_positions";
    paramNames[3] = "ft_final_orientations";
    paramNames[4] = "/nb_tactile_sensors";
    paramNames[5] = "/max_torque";
    paramNames[6] = "/a_force";

    // parse the initial and final positions and orientations of the fingertips
    std::vector <std::vector<Eigen::VectorXd> > params_values(4);

    for (size_t paramNb=0; paramNb<4; paramNb++){
        
        // set an xml variable to hold the parameter
        XmlRpc::XmlRpcValue ros_param;

        // load the parameter from the server
        ros::param::get(paramNames[paramNb], ros_param);

        // check if the parameter is an array
        ROS_ASSERT(ros_param.getType() == XmlRpc::XmlRpcValue::TypeArray);

        // check if the number of fingers is constant among the parameters
        if (nb_fingers < 0){
            nb_fingers = (int)ros_param.size();
        }else{
            int t_nb_fingers = (int)ros_param.size();
            if (t_nb_fingers != nb_fingers){
                std::cout << "[proportional rotation node] The number of fingers (rows) of the parameter \"" << paramNames[paramNb] << "\" does not match the number of fingers (rows) of the previous parameters" << std::endl;
                return - 10*paramNb - 1;
            }
        }

        params_values[paramNb] = std::vector<Eigen::VectorXd>(nb_fingers);

        // parse the parameters to the vector
        for (size_t nb_rows=0; nb_rows < (size_t)nb_fingers; nb_rows++){
            // get the size of the row
            size_t vec_size = ros_param[nb_rows].size();

            // check if the row has exact 3 elements
            if (vec_size != 3){
                std::cout << "[proportional rotation node] The coordinates at the row " << nb_rows + 1 << " of the parameter \"" << paramNames[paramNb] << "\" are not equal to 3" << std::endl;
                return - 10*paramNb - 2;
            }
            

            if (paramNames[paramNb].find("position") != std::string::npos){
                params_values[paramNb][nb_rows] = Eigen::Vector3d();
                params_values[paramNb][nb_rows] << ros_param[nb_rows][0], ros_param[nb_rows][1], ros_param[nb_rows][2];
            }

            if (paramNames[paramNb].find("orientation") != std::string::npos){
                params_values[paramNb][nb_rows] = Eigen::Vector4d();
                params_values[paramNb][nb_rows] = ToQuaternion(ros_param[nb_rows][0], ros_param[nb_rows][1], ros_param[nb_rows][2]);
            }
        }

    }

    // assign the parsed parameters to the class' member-variables
    ftInitPosition = params_values[0];
    ftInitOrientation = params_values[1];
    ftFinalPosition = params_values[2];
    ftFinalOrientation = params_values[3];

    // parse the rest of the parameters
    // number of tactile sensors in the fingertips
    if (!nh.getParam(paramNames[4].c_str(), nb_ft_tactile_sensors)){
        std::cout << "[proportional rotation node] The parameter \"" << paramNames[4] << "\" is not found" << std::endl;
        return -2;
    }

    // maximum torque
    if (!nh.getParam(paramNames[5].c_str(), max_torque)){
        std::cout << "[proportional rotation node] The parameter \"" << paramNames[5] << "\" is not found" << std::endl;
        return -3;
    }

    // applied force
    if (!nh.getParam(paramNames[6], a_force)){
        std::cout << "[proportional rotation node] The parameter \"" << paramNames[6] << "\" is not found" << std::endl;
        return -4;
    }

    return 0;

}



int fnPropRot::run(){
    /*
    *  main function to call the operational methods for running the node
    * 
    *  setting the desired targets and torques and pubishes the messages until an interruption from the keyboard (ctl+c)
    *  
    */

    initTime = ros::Time::now().toSec();

    while(ros::ok()){  // !n_stop  && 

        th_mutex.lock();

        // set the desired target
        setDesPos();

        // set the desired torque
        setDesTrq();

        // publish the data
        publish();

        th_mutex.unlock();

        ros::spinOnce();
        loopRate.sleep();
    }
    
    publish();
    ros::spinOnce();
    loopRate.sleep();
    ros::shutdown();

    return 0;
}


void fnPropRot::stopNodeCallback(int sig){
    /*
    *  callback function for monitoring ctrl+c
    * 
    *  if ctl+c is press on the keyboard, set the control variable to true
    *  
    */
	me->n_stop = true;
}


void fnPropRot::updateFingersPose(const geometry_msgs::PoseArray& msg){
    /*
    *  callback function for listening to the real pose of the fingertips
    *  
    */

    for (int fngr=0; fngr<(int)fingersCurrPosition.size(); fngr++){
        fingersCurrPosition[fngr] << msg.poses[fngr].position.x, msg.poses[fngr].position.y, msg.poses[fngr].position.z;
        fingersCurrOrientation[fngr] << msg.poses[fngr].orientation.w, msg.poses[fngr].orientation.x, msg.poses[fngr].orientation.y, msg.poses[fngr].orientation.z;
    }

}


void fnPropRot::updateExternalCmd(const emg_interface::emg_msg &msg){

    /*
    *  callback function for listening to the expernal commands for the action of fingers
    *  
    */

   for (int i=0; i<nb_fingers; i++){
       external_cmd[i] = (float)msg.Fingers_Direction[i];
   }
   
   external_cmd[4] = (float)msg.Gain;
    
}


void fnPropRot::updateContactStatus(const biotac_interface::biotacMsg &msg){

    /*
    *  callback function for listening to the contact status from fingertips tactile sensors
    *  
    */

   ft_contact[0] = msg.bioTac1;
   ft_contact[1] = msg.bioTac2;
   ft_contact[2] = msg.bioTac3;

   if(std::accumulate(ft_contact.begin(), ft_contact.end(), 0) == 3 ){
       if(!first_time_contact){
           first_time_contact = true;
           update_vectors = true;
       }else{
           update_vectors = false;
       }
   }else{
        first_time_contact = false;
        update_vectors = false;
   }

}


Eigen::Vector4d fnPropRot::ToQuaternion(double yaw, double pitch, double roll) // yaw (Z-axis), pitch (Y-axis), roll (X-axis)
{
    /**
     *  convertion from Euler angles to quartenions
     * 
     *  inputs:
     *      yaw:    rotation around the Z-axes [degrees]
     *      pitch:  rotation around the Y-axis [degrees]
     *      roll:   rotation around the X-axis [degrees]
     *
     *  output:
     *      an Eigen Vector of 4 elements corresponding to the quartenion coordinates w, v, y, z
     */

    // convert to radians
    yaw = yaw * T_PI / 180;
    pitch = pitch * T_PI / 180;
    roll = roll * T_PI / 180;

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


int fnPropRot::setDesPos(){
    /**
     *  setting the desired pose for all the fingertips
     * 
     */

    for (int i=0; i < nb_fingers; i++){
        setFtPose(i);
    }

    return 0;

}

int fnPropRot::setFtPose(int ft_nb){
    /**
     *  setting the desired pose of one fingertip according to the external commands and the input fron the tactile sensors
     * 
     *  input:
     *      ft_nb: the number of the fingertip
     * 
     * 
     */

    int ft_sensor_nb = (ft_nb > nb_ft_tactile_sensors) ? ft_nb -1 : ft_nb;

    if (ft_contact[ft_sensor_nb]){
        targetsPosition[ft_nb] = fingersCurrPosition[ft_nb];
        targetsOrientation[ft_nb] = fingersCurrOrientation[ft_nb];
    }else{

        int state = (external_cmd[ft_nb] < -0.5) ? -1 : 0;
        state = (external_cmd[ft_nb] < 0.5) ? state : 1;

        switch (state) {
        case -1:
            targetsPosition[ft_nb] = ftInitPosition[ft_nb];
            targetsOrientation[ft_nb] = ftInitOrientation[ft_nb];
            break;
        case 0:
            targetsPosition[ft_nb] = fingersCurrPosition[ft_nb];
            targetsOrientation[ft_nb] = fingersCurrOrientation[ft_nb];
            break;
        case 1:
            targetsPosition[ft_nb] = ftFinalPosition[ft_nb];
            targetsOrientation[ft_nb] = ftFinalOrientation[ft_nb];
            break;
        }
    }

    return 0;

}


int fnPropRot::setDesTrq(){
    /**
     *  computing the desired torques of the fingertips
     *  
     * 
     *  if there is a free motion of the fingers (no contact has been occured),
     *  the torques will be set to zero
     * 
     *  The main assumption is that the object to grasp/rotate is a cuboid and 
     *  the index and middle fingers approach the object from the opposite 
     *  object's side/face than the thumb.
     *  
     *  If all fingers are in contact with the object and no object rotation 
     *  is required, then the torques are set so that each fingertip apply a 
     *  force towards the centroid of the triangle described by the three 
     *  fingertips. The magnitudes the overall applied forces on each side
     *  are set to be equal to each other but with opposite directions.
     * 
     *  If an object rotation is required, the forces to be applied are assigned
     *  based on the overall torque, and the contribution to the rotation is 
     *  destributed equally to the overall forces on each side. Although the 
     *  thumb contibutes to half the applied rotational torque regardless of the
     *  direction of the rotation, the contribution weight of the index and middle
     *  finger depends on the direction of the rotation. In the case of clockwise 
     *  rotation, the middle finger contributes more than the index, and in the case 
     *  of counter-clockwise rotation, the index finger contributes more that the
     *  middle finger. 
     * 
     * 
     */


    // set zero values for the free motion condition
    for (int fngr=0; fngr < nb_fingers; fngr++){
        desiredTrq[fngr] << 0.0, 0.0, 0.0;
    }


    // if all three fingertips are in contact with the object
    if(std::accumulate(ft_contact.begin(), ft_contact.end(), 0) == 3 ){

        if (update_vectors){
            // update the vectors only once, when the first contact of all the fingertips occurs

            // index-middle fingertips vector 
            IM = fingersCurrPosition[1] - fingersCurrPosition[0];

            // index-thumb fingertips vector
            IT = fingersCurrPosition[1] - fingersCurrPosition[3];

            // projection of IT on the IM
            projIT = IM * (IM.dot(IT) / IM.norm());

            // norm of the projection
            float d1 = (float)projIT.norm();

            // centroid of the triangle described by the three 
            // fingertips (index, middle and thumb)
            t_cm = (fingersCurrPosition[0] + fingersCurrPosition[1] + fingersCurrPosition[3]) / 3;

            
            // centroid - thumb fingertip vector 
            cmT = t_cm - fingersCurrPosition[3];

            // centroid - index fingertip vector 
            cmI = t_cm - fingersCurrPosition[0];

            // centroid - middle fingertip vector 
            cmM = t_cm - fingersCurrPosition[1];
        }


        // check if an object rotation is required
        if (external_cmd[4] != 0){
            // object rotation is required

            // overall torque
            Eigen::Vector3d trq = IM.cross(IT);

            trq = max_torque * external_cmd[4] * trq / trq.norm();


            // torque applied from the thumb (half of the overall torque)
            desiredTrq[3] = cmT.cross(trq) / 2;


            // percentage of remained torque due to the other two fingers
            float p_index, p_middle;

            // set the torques on the other two fingertips according to the direction of rotation
            if (external_cmd[4] > 0){
                // clockwise
                p_middle = std::abs(IM.norm() - d1)/IM.norm();
                p_index = 1 - p_middle;
                desiredTrq[0] = p_index * (t_cm - (cmI - cmM)/2).cross(trq/2);
                // desiredTrq[1] = p_middle * (t_cm - (cmI - cmM)/2).cross(trq/2);
                desiredTrq[1] = desiredTrq[3];
            }else{
                // counter-clockwise
                p_index = std::abs(IM.norm() - d1)/IM.norm();
                p_middle = 1 - p_index;
                // desiredTrq[0] = p_index * ((cmI - cmM)/2 - t_cm).cross(trq/2);
                desiredTrq[0] = desiredTrq[3];
                desiredTrq[1] = p_middle * ((cmI - cmM)/2 - t_cm).cross(trq/2);
            }


        }else{

            // if no rotation is required, compute the forces to be applied 
            // to the object from each side

            Eigen::Matrix3d rotMat = Eigen::Matrix3d::Zero(3,3);

            rotMat = Eigen::AngleAxisd(T_PI/2,Eigen::Vector3d::UnitY());

            desiredTrq[3]  = - ( rotMat * ( (cmI / cmI.norm()) + (cmM / cmM.norm()) ) ) * (double)a_force;

            // the applied force on the opposite side is equal in magnitude but oposite to the one applied by the thumb
            desiredTrq[0] = - desiredTrq[3]/2;
            desiredTrq[1] = - desiredTrq[3]/2;

        }
        
        // leave the pinky finger in the same position 
        desiredTrq[2] << 0.0, 0.0, 0.0;
    }

    return 0;
}


int fnPropRot::publish(){
    /**
     *  update and publish all the messages
     * 
     */



    for (int fngr=0; fngr < nb_fingers; fngr++){

        // assign the position and orientation values to the pose message
        pose_msg.poses[fngr].position.x = targetsPosition[fngr](0);
        pose_msg.poses[fngr].position.y = targetsPosition[fngr](1);
        pose_msg.poses[fngr].position.z = targetsPosition[fngr](2);

        pose_msg.poses[fngr].orientation.w = targetsOrientation[fngr](0);
        pose_msg.poses[fngr].orientation.x = targetsOrientation[fngr](1);
        pose_msg.poses[fngr].orientation.y = targetsOrientation[fngr](2);
        pose_msg.poses[fngr].orientation.z = targetsOrientation[fngr](3);


        // assign the desired torque to the torque message
        trq_msg.poses[fngr].position.x = desiredTrq[fngr](0);
        trq_msg.poses[fngr].position.y = desiredTrq[fngr](1);
        trq_msg.poses[fngr].position.z = desiredTrq[fngr](2);

        trq_msg.poses[fngr].orientation.w = targetsOrientation[fngr](0);
        trq_msg.poses[fngr].orientation.x = targetsOrientation[fngr](1);
        trq_msg.poses[fngr].orientation.y = targetsOrientation[fngr](2);
        trq_msg.poses[fngr].orientation.z = targetsOrientation[fngr](3);
    }

    // publish the messages


    ft_pose_pub.publish(pose_msg);

    ft_trq_pub.publish(trq_msg);

    return 0;
}


int main (int argc, char **argv)
{
    float frequency = 100.0f;

    ros::init(argc,argv, "object_proportional_rotation_controller");
    ros::NodeHandle n;
    std::shared_ptr<fnPropRot> ft_controller = std::make_shared<fnPropRot>(n, frequency);

    if (!ft_controller->init()){
        std::cout << "[proportional rotation node] Node initialization failed!" << std::endl;
        ROS_INFO("[proportional rotation node] Shutting down the node");
        return -1;
    }else{
        ft_controller->run();
    }
    return 0;
}