#ifndef VICON_H
#define VICON_H

#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <ros/callback_queue.h>
#include "geometry_msgs/TransformStamped.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

struct ViconPoint {
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    double roll;
    double pitch;
    double yaw;
};

class Vicon
{
public:
    Vicon(ros::NodeHandle *n):
        isConvertInit(false),
        isViconInit(false),
        isSetQbw(false),
        startConvert(false),
        isposedata(false),
        isveldata(false),
        isViconUpdate(false),
        isMarkersUpdate(false),
        timeBetweenFrame(0.01){
        nh=*n;
        setup();
    }

    double dataTime,start_time;
    double frameCount,frameMarkersCount,frameCountStart,frameMarkersCountStart,frameCountLast;
    double timeBetweenFrame;
    double qbw[4];

    ros::Subscriber vicondata_sub;
    ros::Publisher viconToWorld_pub;
    ViconPoint state;
    geometry_msgs::Transform viconToWorldTransform;

    double qbw_start[4],qWFromV[4],Rwv[9];
    float translation[3],pose_last[3],vel_last[3];
    bool isConvertInit,startConvert,isposedata,isveldata,isViconUpdate,isMarkersUpdate,isSetQbw,isViconInit;


    //**********KF for vicon velocity**********
    float viconKF_vx,viconKF_vy,viconKF_vz;
    float kfP,kfQ,kfR,kfKk;
    //**********KF for vicon acceleration******
    float viconKF_ax,viconKF_ay,viconKF_az;
    float acckfP,acckfQ,acckfR,acckfKk;
    //*****************************************

    void qtoEuler(double& _roll,double& _pitch,double& _yaw,double q[]);

    void getqInworld(float q[]);


private:
    ros::NodeHandle nh;

    void setup();
    void viconDataCallback(const geometry_msgs::TransformStamped::ConstPtr &msg);
    void qmultiplyq(double q1[],double q2[],double q_result[]);
    void getRotation(double q[],double R[]);
    void transform_world_from_vicon(const double a_vicon[3],double a_world[3]);
};

#endif // Vicon_H
