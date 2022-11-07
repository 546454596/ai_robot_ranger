
#ifndef _AIBRAIN_H
#define _AIBRAIN_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <ros/callback_queue.h>

//#include "opencv/cv.hpp"

#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Path.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/OpticalFlowRad.h"
#include "ai_robot_msgs/set_destination.h"

#include "PIDController.h"
#include "pid_fuzzy.h"
#include "ADRC.h"
#include "postrajectory.h"
#include "assistMath.h"
#include <Eigen/Dense>

//the control target
#include "posControl.h"
#include "../findpath/findpathsrm.h"
#include "../pointtopoint/PointToPoint.h"

//task number define
#define HOVER_TASK 0
#define TAKEOFF_TASK 1
#define LAND_TASK 2
#define MOVE_TASK 3
#define FINDPATH_TASK 4
#define POINTTOPOINT_TASK 5
#define FAR_POINTTOPOINT_TASK 6

class AIBrain
{
public:
    AIBrain(ros::NodeHandle *_nh);
    ~AIBrain();

    void think();
private:
    ros::NodeHandle *nh;
    PosControl poseCtrl;

    //find path
    FindPathSRM fpsrm;

    //point to point
    PointToPoint ptop;

    //pose now
    float poseXYZ_now[3], velXYZ_now[3], roll_now, pitch_now, yaw_now, qwxyz_now[4];
    //pose target
    float poseXYZ_target[3], yaw_target;

    ros::Time pidTime;

    //wether remote controller has input
    bool isControllerInput;
    //what shall be done, see task number define
    int task_lastnumber, task_number;

    //for hover
    bool isStableHover;
    double hoverStartTime;

    //for storing controller input
    double forbackward_ctrl, leftright_ctrl, updown_ctrl, turnlr_ctrl;

    //for find path
    //0--not started, 1--find path done, 2--on the move, 3--to destiny and should end
    uint8_t state_findpath;
    //regard slam as body frame to use assitMath
    //yawslam + yawworldslam = yawworld
    float Rworldslam[9], Tworldslam[3], yawworldslam;
    //destination position
    float poseXYZ_destination[3];

    //for point to point
    uint8_t state_ptop;

    //for far point to point
    uint8_t state_farptop;
    nav_msgs::Path farptop_path;
    int point_i_farptop, pausetime, cntdown;

    //for test mode of find path
    bool isTestModeOfFindPath;

    //subscriber for controller
    ros::Subscriber joy_sub;
    /*
    ros::Subscriber btn1_sub;  //default for takeoff
    ros::Subscriber btn2_sub;  //for find path
    ros::Subscriber btn3_sub;  //default for land
    ros::Subscriber btn4_sub;  //for point to point
    ros::Subscriber btn5_sub;  //for test mode of find path, this mode is under human control mode
    ros::Subscriber btn6_sub;
    ros::Subscriber btn7_sub;
    ros::Subscriber btn8_sub;
    ros::Subscriber btnstart_sub;
    ros::Subscriber btnback_sub;
    ros::Subscriber btnhome_sub;
    ros::Subscriber axes0lr_sub;
    ros::Subscriber axes0ud_sub;
    ros::Subscriber axes1lr_sub;
    ros::Subscriber axes1ud_sub;
    */
    //subscriber for pose,vel
    ros::Subscriber pose_sub;
    ros::Subscriber vel_sub;
    //subscriber for find path
    ros::Subscriber qtSlamWorld;
    //subscriber for destination topic from rviz
    ros::Subscriber move_base_goal_sub;
    //subscriber for point to point
    ros::Subscriber ptop_sub;
    //subscriber for far point to point
    ros::Subscriber farptop_sub;
    //subscriber for slam pose
    ros::Subscriber slampose_sub;

    //service for set destination for find path
    ros::ServiceServer setDestination_ser;

    //add initialize function here
    void initSubscriber();
    void initService();

    //task function here
    void hoverTASK(double _dt);
    void takeoffTASK();
    void landTASK();
    void controllerTASK(double _dt);
    void findpath(double _dt);
    void pointToPointTASK(double _dt);
    void farPointToPointTASK(double _dt);

    void givePosToFindpath();

    //call back for subscriber for controller
    void btn1Callback(const std_msgs::Bool::ConstPtr &_msg);
    void btn3Callback(const std_msgs::Bool::ConstPtr &_msg);
    void axes0lrCallback(const std_msgs::Float64::ConstPtr &_msg);
    void axes0udCallback(const std_msgs::Float64::ConstPtr &_msg);
    void axes1lrCallback(const std_msgs::Float64::ConstPtr &_msg);
    void axes1udCallback(const std_msgs::Float64::ConstPtr &_msg);
    //call back for subscriber for pose,vel
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg);
    void velCallback(const geometry_msgs::TwistStamped::ConstPtr &_msg);
    //call back for subscriber for qtSlamWorld
    void qtslamworldCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg);
    //call back for subscriber for move_base_goal
    void movebasegoalCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg);
    //point to point subscriber
    void ptopCallback(const nav_msgs::Path::ConstPtr &_msg);
    //far point to point subscriber
    void farptopCallback(const nav_msgs::Path::ConstPtr &_msg);

    //find path
    void btn2Callback(const std_msgs::Bool::ConstPtr &_msg);
    //point to point
    void btn4Callback(const std_msgs::Bool::ConstPtr &_msg);
    //test mode for find path
    void btn5Callback(const std_msgs::Bool::ConstPtr &_msg);
    //one topic for all joystick input
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);

    //set destination service call back
    bool setdestinationCallback(ai_robot_msgs::set_destination::Request &req, ai_robot_msgs::set_destination::Response &res);
};

#endif
