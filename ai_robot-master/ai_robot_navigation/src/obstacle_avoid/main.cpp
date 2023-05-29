/*
 * this version is a stand alone obstacle avoidance module. when you publish a
 * local target to it, it starts to move using odometry.
 * this is a convience version of obstacle avoidance module.
 * if you want an efficient obstacle avoidance module, use the p3atObsAvoid.h
 * directly in you program.
*/

#include "p3atObsAvoid.h"
#include "sensor_msgs/Joy.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "../brain/Pioneer3AT.h"

using namespace std;

//global
float targetP[3]={0,0,0};        //x,y,z of target point, in odom frame
float targetOrien[4]={0,0,0,1};  //w,x,y,z of target point, in odom frame
float targetR[9]={1,0,0,0,1,0,0,0,1};
float odomPose[3]={0,0,0};       //body pose in odom frame
float odomOrien[4]={0,0,0,1};    //body pose in odom frame
float odomR[9]={1,0,0,0,1,0,0,0,1};
bool setFinalOrien=false;
bool setLocalTarget=false;
bool stopFlag=false;
p3atObstacleAvoid* ptrpoa;
Pioneer3AT* ptrctrl;
vector<Eigen::Vector3d> vec_target_map;
int num_stored_target=1;
Eigen::Vector3d target_map;

//the input msg is in body frame, transform to odom frame to store
//in case robot moves.
void targetPCallback(const geometry_msgs::PoseConstPtr &msg){
    if(stopFlag){
        ROS_INFO("[OBS]obstacle avoid is lock, press Y to unlock!");
    }
    float tmp[3], tmpR[9];
    tmp[0] = msg->position.x;
    tmp[1] = msg->position.y;
    tmp[2] = msg->position.z;
    qToRotation(odomOrien, tmpR);
    transform_NWUworld_from_body(tmp[0], tmp[1], tmp[2], targetP[0], targetP[1],
            targetP[2], tmpR, odomPose);
    targetOrien[0] = msg->orientation.w;
    targetOrien[1] = msg->orientation.x;
    targetOrien[2] = msg->orientation.y;
    targetOrien[3] = msg->orientation.z;
    if(fabs(targetOrien[0])+fabs(targetOrien[1])+fabs(targetOrien[2])+
            fabs(targetOrien[3]) < 0.1){
        setFinalOrien = false;
        targetOrien[0] = 0;
        targetOrien[1] = 0;
        targetOrien[2] = 0;
        targetOrien[3] = 0;
    }
    else{
        setFinalOrien = true;
        float tmpRbt[9],tmpRob[9];
        qToRotation(targetOrien, tmpRbt);
        qToRotation(odomOrien, tmpRob);
        RmultiR(tmpRob, tmpRbt, targetR);
        rotationToQ(targetOrien, targetR);
    }
//    cout<<targetP[0]<<","<<targetP[1]<<","<<targetP[2]<<"--"<<targetOrien[0]<<","
//        <<targetOrien[1]<<","<<targetOrien[2]<<","<<targetOrien[3]<<endl;
    setLocalTarget = true;
}
// this callback is available when obtaining targetP_odom from brainnavi visual system
void targetodomCallback(const geometry_msgs::PoseConstPtr &msg){
    if(stopFlag){
        ROS_INFO("[OBS]obstacle avoid is lock, press Y to unlock!");
    }
    Eigen::Vector3d tmptarmap(msg->position.x, msg->position.y, msg->position.z);
    if(vec_target_map.size() == 0){
        vec_target_map.push_back(tmptarmap);
    }
    else{
        // judge whether the same as the last one
        Eigen::Vector3d dellastthis = tmptarmap - vec_target_map[vec_target_map.size()-1];
        if(dellastthis.norm() > 0.0){
            // add this to vector
            vec_target_map.push_back(tmptarmap);
            if(vec_target_map.size() > num_stored_target){
                // only store last 5
                vec_target_map.erase(vec_target_map.begin());
            }
        }
    }
    target_map << 0,0,0;
    for(int i=0; i<vec_target_map.size(); ++i){
        target_map += vec_target_map[i];
    }
    target_map = target_map / vec_target_map.size();

    targetP[0] = target_map[0];
    targetP[1] = target_map[1];
    targetP[2] = target_map[2];
    targetOrien[0] = msg->orientation.w;
    targetOrien[1] = msg->orientation.x;
    targetOrien[2] = msg->orientation.y;
    targetOrien[3] = msg->orientation.z;
    if(fabs(targetOrien[0])+fabs(targetOrien[1])+fabs(targetOrien[2])+
       fabs(targetOrien[3]) < 0.1){
        setFinalOrien = false;
        targetOrien[0] = 0;
        targetOrien[1] = 0;
        targetOrien[2] = 0;
        targetOrien[3] = 0;
    }
    else{
        setFinalOrien = true;
    }
    setLocalTarget = true;
}

void odomCallback(const nav_msgs::OdometryConstPtr & msg){
    odomPose[0] = msg->pose.pose.position.x;
    odomPose[1] = msg->pose.pose.position.y;
    odomPose[2] = msg->pose.pose.position.z;
    odomOrien[0] = msg->pose.pose.orientation.w;
    odomOrien[1] = msg->pose.pose.orientation.x;
    odomOrien[2] = msg->pose.pose.orientation.y;
    odomOrien[3] = msg->pose.pose.orientation.z;
    qToRotation(odomOrien, odomR);
}

void ekfposeCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg)
{
    odomPose[0] = _msg->pose.position.x;
    odomPose[1] = _msg->pose.position.y;
    odomPose[2] = _msg->pose.position.z;
    odomOrien[0] = _msg->pose.orientation.w;
    odomOrien[1] = _msg->pose.orientation.x;
    odomOrien[2] = _msg->pose.orientation.y;
    odomOrien[3] = _msg->pose.orientation.z;
    qToRotation(odomOrien, odomR);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr &msg){
    if(msg->buttons[0] > 0.5){
        //btn A
        stopFlag = true;
        setLocalTarget = false;
        ptrpoa->reset();
        ptrctrl->move(0,0);
    }
    else if(msg->buttons[3] > 0.5){
        //btn Y
        stopFlag = false;
    }
}


//add by xj
void stoptaskCallback(const std_msgs::String::ConstPtr &msg){
        stopFlag = true;
        setLocalTarget = false;
        ptrpoa->reset();
        ptrctrl->move(0,0);
	
}

void obsAbortCallback(const std_msgs::String::ConstPtr &msg)
{
	
	ptrpoa->reset();
	stopFlag = false;
	setLocalTarget = false;
	printf("+++++++++++++++++++++++++++++++++++\n");
}


int main(int argc, char** argv){
    ros::init(argc, argv,"obsavoid");
    ros::NodeHandle nh;
    p3atObstacleAvoid poa(nh);
    ptrpoa = &poa;
    Pioneer3AT ctrl(nh);
    ptrctrl = &ctrl;
    ros::Subscriber joy_sub = nh.subscribe("/joy", 1, joyCallback);
    //add by xj
    ros::Subscriber joy_sub2 = nh.subscribe("/remote/joy",1,joyCallback);
	ros::Subscriber stop_task = nh.subscribe("/remote/stoptask",1,stoptaskCallback);
	ros::Subscriber obs_abort = nh.subscribe("/obs_abort", 1, obsAbortCallback);
    ros::Subscriber odom_sub = nh.subscribe("/RosAria/pose", 1, odomCallback);
    //ros::Subscriber ekfpose_sub = nh.subscribe("/est_pose", 1, ekfposeCallback);
    ros::Subscriber tarP_sub = nh.subscribe("/ai_robot/findpath/targetP", 1, targetPCallback);
    ros::Subscriber targetodom_sub = nh.subscribe("/targetP_odom", 1, targetodomCallback);

    ros::Rate loop(50);
    float Plt[3], Olt[4], Rlt[9], vxc = 0, rzc = 0;
    ptrctrl->move(0,0);
    ROS_INFO("[OBS]waiting odom!");
    ros::topic::waitForMessage<nav_msgs::Odometry>("/RosAria/pose", nh);
    ROS_INFO("[OBS]Obstacle avoid wake up!");
    while (ros::ok()){
        if(!stopFlag && setLocalTarget){
            transform_body_from_NWUworld(Plt[0], Plt[1], Plt[2], targetP[0],
                    targetP[1], targetP[2], odomR, odomPose);
            if(setFinalOrien){
                RtmultiR(odomR, targetR, Rlt);
                rotationToQ(Olt, Rlt);
                //cout<<Olt[0]<<","<<Olt[1]<<","<<Olt[2]<<","<<Olt[3]<<endl;
            }
            else{
                Olt[0] = 0;
                Olt[1] = 0;
                Olt[2] = 0;
                Olt[3] = 0;
            }
            if(poa.gotoLocalTarget(vxc, rzc, Plt, Olt, true)){
                ctrl.move(vxc,rzc);
            }
            else{
                ctrl.move(0,0);
                ptrpoa->reset();
                setLocalTarget = false;
            }
        }
        else{
            poa.pubSafeZone();
        }
        ros::spinOnce();
        loop.sleep();
    }
    ros::shutdown();
    return 0;
}
