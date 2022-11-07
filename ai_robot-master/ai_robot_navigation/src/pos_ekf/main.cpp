#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include "vicon.h"
#include <std_msgs/Bool.h>

#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include "mavros_msgs/OpticalFlowRad.h"
#include <std_msgs/Float32.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
//#include <ardrone_autonomy/navdata_altitude.h>
//#include <ardrone_autonomy/Navdata.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Transform.h>

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <deque>

#include "pose_ekf.h"
#include <algorithm>

#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/MagneticModel.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <fstream>

//#include <../../include/assistMath.h>
#include "assistMath.h"

#define SIMULATION 0
#define ARDRONE 0
#define PIONEER3AT 1

#define ORB 0
#define CARTO 1

#define PI 3.1415926

std::string drift_path;
std::string geoidsPath;

fstream drift_file;

boost::shared_ptr<GeographicLib::Geoid> geoid;

using namespace std;
using namespace Eigen;

int USE_GPS=0 ;
int USE_VICON =1;

bool getSLAMRTdone=0;
bool imuok =  false;
float Rwslam[9]={1,0,0, 0,1,0, 0,0,1}, Twslam[3]={0,0,0}, q_bworld[4] = {0,0,0,1},
q_worldfromslam[4] = {0,0,0,1}, R_bworld[9] = {1,0,0, 0,1,0, 0,0,1},
rol_world=0, pit_world=0, yaw_world=0, pos_gtworld[3]={0,0,0}, deltaYaw=0;
float errCamToDroneCenter = 0.2;
int cnt_slam=0, cntd_slam=10; //only after cntd_slam slam results it will calculate slam
bool brain_ok = false;
float p3at_speed_modify = 1.0;
//slamtec's yaw
bool isfirstslamtecsub = true;
float err_slamtec_imu = 0, yaw_slamtec_mod = 0;
int cnt_slamtecyaw = 0;

enum SensorType
{
  IMU,
  FIX,
  FIX_VELOCITY,
  SONAR,
  OPTICALFLOW,
};
Pose_ekf pose_ekf;
Vicon *_vicon;
ros::Publisher pose_pub;
ros::Publisher vel_pub;
ros::Publisher vicon_pose_pub;
ros::Publisher vicon_vel_pub;
ros::Publisher pub_pose_debug;
ros::Publisher pub_vel_debug ;
ros::Publisher pub_px4pose_debug;
ros::Publisher pub_px4vel_debug;
ros::Publisher test_slam_pub;
ros::Publisher test_gt_pub;
ros::Publisher slam_pos_pub;
ros::Publisher QTworldslam_pub;

nav_msgs::Path path_msg;
ros::Publisher pub_path;

deque< pair<double, geometry_msgs::PoseStamped> > slam_q;
deque< pair<double, sensor_msgs::Imu> > imu_q;
deque< pair<double, Vector2d> >vel_q;
deque< pair<double, double> >height_q;//ardrone_autonomy::navdata_altitude
deque< pair<double, geometry_msgs::PoseStamped> >correct_pose_q;


void publish_pose(Pose_ekf pose_ekf)
{
  geometry_msgs::PoseStamped pose;
  geometry_msgs::TwistStamped velocity;
  Quaterniond q;
  Vector3d p, v, bw, ba;
  pose_ekf.getState(q, p, v, ba);
  //float _q[4]={q.w(), q.x(), q.y(), q.z()};
  q_bworld[0] = q.w();
  q_bworld[1] = q.x();
  q_bworld[2] = q.y();
  q_bworld[3] = q.z();
  qToEuler(rol_world, pit_world, yaw_world, q_bworld);
  qToRotation(q_bworld, R_bworld);
  pos_gtworld[0] = p(0);
  pos_gtworld[1] = p(1);
  pos_gtworld[2] = p(2);
  pose.header.stamp = ros::Time(pose_ekf.get_time());
  pose.header.frame_id = "/world";

  velocity.header.stamp = ros::Time(pose_ekf.get_time());
  velocity.header.frame_id = "/world";
  velocity.twist.angular.x=0;
  velocity.twist.angular.y=0;
  velocity.twist.angular.z=0;

  pose.pose.orientation.w = _vicon->qbw[0];
  pose.pose.orientation.x = _vicon->qbw[1];
  pose.pose.orientation.y = _vicon->qbw[2];
  pose.pose.orientation.z = _vicon->qbw[3];//q is NWU frame
  pose.pose.position.x = _vicon->state.x;
  pose.pose.position.y =  _vicon->state.y;
  pose.pose.position.z =  _vicon->state.z;//publish postion as NWU frame
  vicon_pose_pub.publish(pose);

  velocity.twist.linear.x= _vicon->state.vx;
  velocity.twist.linear.y=_vicon->state.vy;
  velocity.twist.linear.z=_vicon->state.vz;//publish as NWU frame
  vicon_vel_pub.publish(velocity);
  if(USE_VICON){
      pose.pose.orientation.w = _vicon->qbw[0];
      pose.pose.orientation.x = _vicon->qbw[1];
      pose.pose.orientation.y = _vicon->qbw[2];
      pose.pose.orientation.z = _vicon->qbw[3];//q is NWU frame
      pose.pose.position.x = _vicon->state.x;
      pose.pose.position.y = _vicon->state.y;
      pose.pose.position.z =  _vicon->state.z;//publish postion as NWU frame
      pose_pub.publish(pose);

      velocity.twist.linear.x= _vicon->state.vx;
      velocity.twist.linear.y=_vicon->state.vy;
      velocity.twist.linear.z=_vicon->state.vz;//publish as NWU frame
      vel_pub.publish(velocity);
  }else{
      pose.pose.orientation.w = q.w();
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();//q is NWU frame
      pose.pose.position.x = p(0);
      pose.pose.position.y = p(1);
      pose.pose.position.z = p(2);//publish postion as NWU frame
      pose_pub.publish(pose);

      velocity.twist.linear.x=v(0);
      velocity.twist.linear.y=v(1);
      velocity.twist.linear.z=v(2);//publish as NWU frame
      vel_pub.publish(velocity);
  }

  geometry_msgs::Vector3 data;
  data.x=pose.pose.position.x;
  data.y=pose.pose.position.y;
  data.z=pose.pose.position.z;
  pub_pose_debug.publish(data);
  data.x=velocity.twist.linear.x;
  data.y=velocity.twist.linear.y;
  data.z=velocity.twist.linear.z;
  pub_vel_debug.publish(data);
}


bool loadModels() 
{
  
  if (!geoid) {
    try {
      geoid = boost::make_shared<GeographicLib::Geoid>("egm84-15", geoidsPath.c_str());
    }
    catch (GeographicLib::GeographicErr &e) {
      ROS_ERROR("[POS_EKF]Failed to load geoid. Reason: %s", e.what());
      return false;
    }
  }
  
  return true;
}


bool processSensorData()
{
  if(imu_q.empty() || (imu_q.back().first - imu_q.front().first) < 0.05 ) return false;

  static int imu_cnt = 0;//correct with acc every 10 times
  //find the first com sensor
  double t[8] = {DBL_MAX};
  for(int i = 0; i < 8; i++) t[i] = DBL_MAX;
  if(!imu_q.empty()) t[0] = imu_q.front().first;
  if(!vel_q.empty()) t[1] = vel_q.front().first;
  if(!height_q.empty()) t[2] = height_q.front().first;
  if(!correct_pose_q.empty()) t[3] = correct_pose_q.front().first;
  if(!slam_q.empty()) t[4] = slam_q.front().first;

  int min_id;

  //for(int i = 0; i < 5; i++) cout << i << " " << t[i] << "  ";
  min_id = min_element(t, t + 8) - t;
  //cout << "min_id: " << min_id << "  min_t: " << t[min_id] << endl;
  if(t[min_id] == DBL_MAX) return false;
  
  if(min_id == 0)//imu
  {
        //cout << "size: " << imu_q.size() << endl;
        double t = imu_q.front().first;
        sensor_msgs::Imu msg = imu_q.front().second;
        Vector3d acc;
        Vector4d q;
        float q_world[4];
        acc(0) = msg.linear_acceleration.x;
        acc(1) = msg.linear_acceleration.y;
        acc(2) = msg.linear_acceleration.z;
        q(0) = msg.orientation.w;
        q(1) = msg.orientation.x;
        q(2) = msg.orientation.y;
        q(3) = msg.orientation.z;
        /*
         if(!isfirstslamtecsub){
            float _q[4]={float(msg.orientation.x), float(msg.orientation.w),
                         float(msg.orientation.y), float(msg.orientation.z)}, ro, pi, ya;
            qToEuler(ro, pi, ya, _q);
            //replace yaw with slamtec's modify yaw
            //eulerToQ(ro , pi, yaw_slamtec_mod, _q);
            eulerToQ(0,0,yaw_slamtec_mod, _q);
            q(0) = _q[0];
            q(1) = _q[1];
            q(2) = _q[2];
            q(3) = _q[3];
        }
        else */if(getSLAMRTdone){//false
            q_world[0]=q_worldfromslam[0];
            q_world[1]=q_worldfromslam[1];
            q_world[2]=q_worldfromslam[2];
            q_world[3]=q_worldfromslam[3];
            q(0)=q_worldfromslam[0];
            q(1)=q_worldfromslam[1];
            q(2)=q_worldfromslam[2];
            q(3)=q_worldfromslam[3];
//cout << "q_worldfromslam:"<<endl<<q_worldfromslam[1]<<endl<<q_worldfromslam[2]<<endl<<q_worldfromslam[3]<<endl<<q_worldfromslam[0]<<endl;
        }
        else{
        q_world[0]=msg.orientation.w;
        q_world[1]=msg.orientation.x;
        q_world[2]=msg.orientation.y;
        q_world[3]=msg.orientation.z;
//float dq[4]={q_world[0]-q_worldfromslam[0], q_world[1]-q_worldfromslam[1], q_world[2]-q_worldfromslam[2], q_world[3]-q_worldfromslam[3]};
//cout << "error qxyzw:"<<endl<<dq[1]<<endl<<dq[2]<<endl<<dq[3]<<endl<<dq[0]<<endl;
        }

        pose_ekf.predict(q, acc, t);
        imu_cnt++;
        if(!(_vicon->isSetQbw)) _vicon->getqInworld(q_world);
        if(imu_cnt % 10 == 0) pose_ekf.correct_gravity(acc, t);
        imu_q.pop_front();
        
    } else if(min_id == 1)//vel
    {
      double t = vel_q.front().first;
      Vector2d opt_velocity = vel_q.front().second;
      pose_ekf.correct_opt_velocity(opt_velocity, t);
      vel_q.pop_front();

    }else if(min_id == 2) //height
    {
      double t = height_q.front().first;
      //ardrone_autonomy::navdata_altitude msg = height_q.front().second;
      double sonar_height = height_q.front().second;//msg.altitude_raw*1.0/1000;
      pose_ekf.correct_sonar_height(sonar_height, t);
      height_q.pop_front();
      
    }else if(min_id == 3)//correct pose
    {
        double t = correct_pose_q.front().first;
        geometry_msgs::PoseStamped msg = correct_pose_q.front().second;
        Vector2d d_position;
        Vector4d d_q;
        d_position(0)=msg.pose.position.x;
        d_position(1)=msg.pose.position.y;
        d_q(0)=msg.pose.orientation.w;
        d_q(1)=msg.pose.orientation.x;
        d_q(2)=msg.pose.orientation.y;
        d_q(3)=msg.pose.orientation.z;
        pose_ekf.correct_posecorrect(d_position, d_q, t);
        correct_pose_q.pop_front();
    }
  else if(min_id == 4)//correct slam pose
  {
      double _t = slam_q.front().first;
      geometry_msgs::PoseStamped msg = slam_q.front().second;
      Vector3d slam_position;
      slam_position(0) = msg.pose.position.x;
      slam_position(1) = msg.pose.position.y;
      slam_position(2) = msg.pose.position.z;
      pose_ekf.correct_slam_pose(slam_position, _t);
      slam_q.pop_front();
  }

  return true;
}

void slamCallback(const geometry_msgs::PoseStampedPtr& slam_msg)
{
  if(!imuok){
    return;
  }
    //test
    float rol,pit,yaw,_q[4],postmp[3];
    if(ORB){
//    geometry_msgs::Vector3 data;
    _q[0]=float(slam_msg->pose.orientation.w);
    _q[1]=float(slam_msg->pose.orientation.z);
    _q[2]=float(-slam_msg->pose.orientation.x);
    _q[3]=float(-slam_msg->pose.orientation.y);
//    data.x=rol;
//    data.y=pit;
//    data.z=yaw;
    //test_slam_pub.publish(data);

    //get slam position and change into nwu
    postmp[0] = slam_msg->pose.position.z;
    postmp[1] = -slam_msg->pose.position.x;
    postmp[2] = -slam_msg->pose.position.y;
    //cout << "slam call back" <<endl;
    }
    else if(CARTO){
    _q[0]=slam_msg->pose.orientation.w;
    _q[1]=slam_msg->pose.orientation.x;
    _q[2]=slam_msg->pose.orientation.y;
    _q[3]=slam_msg->pose.orientation.z;
    postmp[0] = slam_msg->pose.position.x;
    postmp[1] = slam_msg->pose.position.y;
    postmp[2] = slam_msg->pose.position.z;
    }
    qToEuler(rol,pit,yaw,_q);
    cnt_slam = 0;

    if(getSLAMRTdone)
    {
        //then transform into drone world frame
        geometry_msgs::PoseStamped _slam_msg;
        float pos[3];
        transform_NWUworld_from_body(postmp[0], postmp[1], postmp[2],
                                     pos[0], pos[1], pos[2],
                                     Rwslam, Twslam);
        //yaw += deltaYaw;
        float Rbodyworld[9], Rbodyslam[9];
        qToRotation(_q, Rbodyslam);
        RmultiR(Rwslam, Rbodyslam, Rbodyworld);
        rotationToQ(_q, Rbodyworld);
        qToEuler(rol,pit,yaw,_q);
        _slam_msg.pose.position.x = pos[0] - errCamToDroneCenter*cos(yaw);
        _slam_msg.pose.position.y = pos[1] - errCamToDroneCenter*sin(yaw);
        _slam_msg.pose.position.z = pos[2] + errCamToDroneCenter*sin(pit);
        //eulerToQ(rol, pit, yaw, _q);
        q_worldfromslam[0] = _q[0];
        q_worldfromslam[1] = _q[1];
        q_worldfromslam[2] = _q[2];
        q_worldfromslam[3] = _q[3];
//cout << "q_worldfromslam:"<<endl<<q_worldfromslam[1]<<endl<<q_worldfromslam[2]<<endl<<q_worldfromslam[3]<<endl<<q_worldfromslam[0]<<endl;
        _slam_msg.pose.orientation.w = _q[0];
        _slam_msg.pose.orientation.x = _q[1];
        _slam_msg.pose.orientation.y = _q[2];
        _slam_msg.pose.orientation.z = _q[3];
        //cout << "pub slam pos after modify" <<endl;
        slam_pos_pub.publish(_slam_msg);
        double t = slam_msg->header.stamp.toSec();//ros::Time::now().toSec();
        slam_q.push_back(make_pair(t, _slam_msg) );
    }
    else
    {
        if(cntd_slam > 0)
        {
            --cntd_slam;
            return;
        }
        //if(isnan(pos_gtworld[0])) return;
        //calculate the R T from slam_nwu to drone_world
        float pos_worldtmp[3], R_bodyworld[9], R_bodyslam[9];
        Eigen::Matrix<float, 4, 4> Tslamworld, Tbodyworld, Tbodyslam;
        qToRotation(q_bworld, R_bodyworld);
        Tbodyworld << R_bodyworld[0], R_bodyworld[1], R_bodyworld[2], pos_gtworld[0],
                R_bodyworld[3], R_bodyworld[4], R_bodyworld[5], pos_gtworld[1],
                R_bodyworld[6], R_bodyworld[7], R_bodyworld[8], pos_gtworld[2],
                0, 0, 0, 1;
//        cout << "Tbodyorld{" << Tbodyworld(0,0) <<", "<< Tbodyworld(0,1) <<", "<< Tbodyworld(0,2)<<endl
//                << Tbodyworld(1,0) <<", "<< Tbodyworld(1,1) <<", "<< Tbodyworld(1,2)<<endl
//                << Tbodyworld(2,0) <<", "<< Tbodyworld(2,1) <<", "<< Tbodyworld(2,2)<<"}"<<endl;
        qToRotation(_q, R_bodyslam);
//        cout << "Rbodyslam{" << R_bodyslam[0] <<", "<< R_bodyslam[1] <<", "<< R_bodyslam[2]<<endl
//                << R_bodyslam[3] <<", "<< R_bodyslam[4] <<", "<< R_bodyslam[5]<<endl
//                << R_bodyslam[6] <<", "<< R_bodyslam[7] <<", "<< R_bodyslam[8]<<"}"<<endl;
        Tbodyslam << R_bodyslam[0], R_bodyslam[1], R_bodyslam[2], postmp[0],
                R_bodyslam[3], R_bodyslam[4], R_bodyslam[5], postmp[1],
                R_bodyslam[6], R_bodyslam[7], R_bodyslam[8], postmp[2],
                0, 0, 0, 1;
//        cout << "Tbodyslam{" << Tbodyslam(0,0) <<", "<< Tbodyslam(0,1) <<", "<< Tbodyslam(0,2)<<endl
//                << Tbodyslam(1,0) <<", "<< Tbodyslam(1,1) <<", "<< Tbodyslam(1,2)<<endl
//                << Tbodyslam(2,0) <<", "<< Tbodyslam(2,1) <<", "<< Tbodyslam(2,2)<<"}"<<endl;
        Tslamworld =  Tbodyworld * Tbodyslam.inverse();
//        deltaYaw = yaw_world - yaw;
//        Rwslam[0] = cos(deltaYaw);
//        Rwslam[4] = cos(deltaYaw);
//        Rwslam[1] = -sin(deltaYaw);
//        Rwslam[3] = sin(deltaYaw);
//        transform_NWUworld_from_body(postmp[0],postmp[1],postmp[2],
//                pos_worldtmp[0], pos_worldtmp[1], pos_worldtmp[2],
//                Rwslam, Twslam);//from slam to world
//        Twslam[0] = pos_gtworld[0] - pos_worldtmp[0];
//        Twslam[1] = pos_gtworld[1] - pos_worldtmp[1];
//        Twslam[2] = pos_gtworld[2] - pos_worldtmp[2];
//        cout << "{" << Tslamworld(0,0) <<", "<< Tslamworld(0,1) <<", "<< Tslamworld(0,2)<<endl
//                << Tslamworld(1,0) <<", "<< Tslamworld(1,1) <<", "<< Tslamworld(1,2)<<endl
//                << Tslamworld(2,0) <<", "<< Tslamworld(2,1) <<", "<< Tslamworld(2,2)<<"}"<<endl;
        Rwslam[0] = Tslamworld(0,0);
        Rwslam[1] = Tslamworld(0,1);
        Rwslam[2] = Tslamworld(0,2);
        Rwslam[3] = Tslamworld(1,0);
        Rwslam[4] = Tslamworld(1,1);
        Rwslam[5] = Tslamworld(1,2);
        Rwslam[6] = Tslamworld(2,0);
        Rwslam[7] = Tslamworld(2,1);
        Rwslam[8] = Tslamworld(2,2);
        Twslam[0] = Tslamworld(0,3);
        Twslam[1] = Tslamworld(1,3);
        Twslam[2] = Tslamworld(2,3);
        float _qq[4];
        //pub it
        rotationToQ(_qq, Rwslam);
        geometry_msgs::PoseStamped msg;
        msg.pose.position.x = Twslam[0];
        msg.pose.position.y = Twslam[1];
        msg.pose.position.z = Twslam[2];
        msg.pose.orientation.w = _qq[0];
        msg.pose.orientation.x = _qq[1];
        msg.pose.orientation.y = _qq[2];
        msg.pose.orientation.z = _qq[3];
        ROS_INFO("[POS_EKF]slam frame ok!");
        //sleep(1);
        QTworldslam_pub.publish(msg);
        
        getSLAMRTdone = true;
        cntd_slam = 10;
    }
}

void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
//    double t = ros::Time::now().toSec();
    imu_q.push_back(make_pair(t, *imu_msg) );
    if(!imuok){
      imuok = true;
    }
}

void slamtecYawCallback(const std_msgs::Float32ConstPtr &msg){
    cnt_slamtecyaw = 0;
    if(isfirstslamtecsub){
        if(imu_q.empty()){
            return;
        }
        isfirstslamtecsub = false;
        float qimu[4], ro, pi, ya;
        sensor_msgs::Imu _imu = imu_q[imu_q.size()-1].second;
        qimu[0] = _imu.orientation.w;
        qimu[1] = _imu.orientation.x;
        qimu[2] = _imu.orientation.y;
        qimu[3] = _imu.orientation.z;
        qToEuler(ro, pi, ya, qimu);
        err_slamtec_imu = msg->data - ya;
    }
    else{
        yaw_slamtec_mod = msg->data - err_slamtec_imu;
        if(yaw_slamtec_mod > PI){
            yaw_slamtec_mod -= 2*PI;
        }
        else if(yaw_slamtec_mod < -PI){
            yaw_slamtec_mod += 2*PI;
        }
    }
}

void odometryCallback(const nav_msgs::OdometryConstPtr & msg)
{

}

//void navaltdataCallback(const ardrone_autonomy::navdata_altitudeConstPtr &msg)
//{
//    double t = ros::Time::now().toSec();
//    double _m = msg->altitude_raw*1.0/1000;
//    height_q.push_back(make_pair(t, _m));
//  //cout << "sonar; ";
//}

//void navdataCallback(const ardrone_autonomy::NavdataConstPtr &msg)
//{
//    double t = ros::Time::now().toSec();
//    Vector2d velocity;
//    velocity(0) =msg->vx*1.0/1000; //convert to nwu_body frame
//    velocity(1) =msg->vy*1.0/1000;
//    vel_q.push_back(make_pair(t, velocity));

//}

/**
 * correct pose by image information
 * give the correct value os position
 * then use ekf to fusion data
 * /
**/
void correctPoseDataCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
//    double t = msg->header.stamp.toSec();

    double t = ros::Time::now().toSec();
    correct_pose_q.push_back(make_pair(t, *msg));

}

/**
 * vision hover :correct pose by image information
 * give the correct value or position
 * then use ekf to fusion data
 * /
**/
void hoverCorrectPoseDataCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
//    double t = msg->header.stamp.toSec();

    double t = ros::Time::now().toSec();

    Vector2d d_position;
    Vector4d d_q;

    d_position(0)=msg->pose.position.x;
    d_position(1)=msg->pose.position.y;
    d_q(0)=msg->pose.orientation.w;
    d_q(1)=msg->pose.orientation.x;
    d_q(2)=msg->pose.orientation.y;
    d_q(3)=msg->pose.orientation.z;
    pose_ekf.correct_posecorrect(d_position, d_q, t);

}

#if SIMULATION
//test
void gtposeCallback(const geometry_msgs::PoseConstPtr &msg)
{
//    geometry_msgs::PoseStamped pose;
//    if(isnan(gtPosLast[0])){
//        pose.pose=*msg;
//        gtPosLast[0]=msg->position.x;
//        gtPosLast[1]=msg->position.y;
//        gtPosLast[2]=msg->position.z;
//    }
//    else{
//        pose.pose.orientation=msg->orientation;
//        pose.pose.position.x+=msg->position.x-gtPosLast[0];
//        pose.pose.position.y+=msg->position.y-gtPosLast[1];
//        pose.pose.position.z+=msg->position.z-gtPosLast[2];
//        gtPosLast[0]=msg->position.x;
//        gtPosLast[1]=msg->position.y;
//        gtPosLast[2]=msg->position.z;
//    }

//    pose.header.seq++;
//    pose.header.stamp=ros::Time::now();
//    pose_pub.publish(pose);

    //test
    geometry_msgs::Vector3 data;
    float rol,pit,yaw,_q[4]={msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z};
    qToEuler(rol,pit,yaw,_q);
    data.x=rol;
    data.y=pit;
    data.z=yaw;
    rol_world=rol;
    pit_world=pit;
    yaw_world=yaw;
    pos_gtworld[0] = msg->position.x;
    pos_gtworld[1] = msg->position.y;
    pos_gtworld[2] = msg->position.z;
    test_gt_pub.publish(data);
    //3 seconds no slam reset the cnt to 0, use gt
    if(++cnt_slam > 300)
    {
        if(getSLAMRTdone == true)
        {
            getSLAMRTdone = false;
            Rwslam[0] = 1;
            Rwslam[4] = 1;
            Rwslam[1] = 0;
            Rwslam[3] = 0;
            Twslam[0] = 0;
            Twslam[1] = 0;
            Twslam[2] = 0;
            ROS_INFO("[POS_EKF]reuse gt pose vel.");
        }
    }

    if(!getSLAMRTdone)  //use this as slam for control when no slam
    {
        geometry_msgs::PoseStamped _slam_msg;
        _slam_msg.pose.position.x = msg->position.x;
        _slam_msg.pose.position.y = msg->position.y;
        _slam_msg.pose.position.z = msg->position.z;
        _slam_msg.pose.orientation.w = _q[0];
        _slam_msg.pose.orientation.x = _q[1];
        _slam_msg.pose.orientation.y = _q[2];
        _slam_msg.pose.orientation.z = _q[3];
//        //cout << "pub slam pos after modify" <<endl;
//        slam_pos_pub.publish(_slam_msg);
        double t = ros::Time::now().toSec();
        slam_q.push_back(make_pair(t, _slam_msg) );
    }
//    data.x=msg->position.x;
//    data.y=msg->position.y;
//    data.z=msg->position.z;
//    pub_gtpose_debug.publish(data);

}
//test
void gtvelCallback(const geometry_msgs::TwistPtr &msg)
{
//    geometry_msgs::TwistStamped velocity;
//    velocity.twist=*msg;
//    vel_pub.publish(velocity);
    double t = ros::Time::now().toSec();
    Vector2d velocity;
    velocity(0) = msg->linear.x*cos(yaw_world) + msg->linear.y*sin(yaw_world); //convert to nwu_body frame
    velocity(1) = -msg->linear.x*sin(yaw_world) + msg->linear.y*cos(yaw_world);
    vel_q.push_back(make_pair(t, velocity));

//    geometry_msgs::Vector3 data;
//    data.x=velocity.twist.linear.x;
//    data.y=velocity.twist.linear.y;
//    data.z=velocity.twist.linear.z;
//    pub_vel_debug.publish(data);
//    data.x=velocity.twist.linear.x;
//    data.y=velocity.twist.linear.y;
//    data.z=velocity.twist.linear.z;
//    pub_gtvel_debug.publish(data);
}

#endif



void p3atPoseCallback(const nav_msgs::OdometryConstPtr & msg)
{
    static double last_speed_time=ros::Time::now().toSec();
    double t = msg->header.stamp.toSec();//ros::Time::now().toSec();
    double dt = t - last_speed_time;
    last_speed_time = t;
    //cout<<"time:"<<dt<<endl;
    if(dt > 1) return;
    Vector2d velocity;
    velocity(0)=0;
    velocity(1)=0;
//    Vector2d velori;
//    velori << msg->twist.twist.linear.x, msg->twist.twist.linear.y;
//    Matrix<double, 2, 2> Rbw;
//    Rbw << R_bworld[0], R_bworld[1], R_bworld[3], R_bworld[4];
//    velocity = Rbw * velori;
//    velocity(0) = msg->twist.twist.linear.x * cos(yaw_world); //convert to nwu_body frame
//    velocity(1) = msg->twist.twist.linear.x * sin(yaw_world);
    //give body frame vel     ?|| dt < 0.008
    if(false){//(fabs(msg->twist.twist.angular.z)>0.01 ) && (dt>0.001)){//more accurate vel for vx,vy
        double vx = msg->twist.twist.linear.x / p3at_speed_modify,
                rz = msg->twist.twist.angular.z / p3at_speed_modify;
        double r = fabs(vx/rz);
        double d = sqrt(2*r*r*(1-cos(rz*dt)));
        velocity(0) = d * cos(rz*dt/2) / dt;
        velocity(1) = d * sin(rz*dt/2) / dt;
    }
    else{
        velocity(0) = msg->twist.twist.linear.x / p3at_speed_modify;
        velocity(1) = msg->twist.twist.linear.y / p3at_speed_modify;
    }
    //cout<<"test----"<<endl<<"dt="<<dt<<", velocity="<<velocity(0)<<", "<<velocity(1)<<endl;
//cout<<"velcb:"<<velocity(0)<<","<<velocity(1)<<endl;
    vel_q.push_back(make_pair(t, velocity));
    //lock height on plane ground, if not in a plane ground, diable it
    if(true){
        double _height = 1.0;
        height_q.push_back(make_pair(t, _height));
    }
}

void mavrosHeightCallback(const sensor_msgs::RangeConstPtr &msg)
{
    double t = ros::Time::now().toSec();
    double _m = msg->range;
    height_q.push_back(make_pair(t, _m));
}

void brainokCallback(const std_msgs::BoolConstPtr &msg){
    brain_ok = msg->data;
}

int main (int argc, char **argv) 
{
  ros::init(argc, argv, "pose_estimator");
  ros::NodeHandle n;
  _vicon=new Vicon(&n);

  pub_pose_debug = n.advertise<geometry_msgs::Vector3>("/debug/pose", 10);
  pub_vel_debug = n.advertise<geometry_msgs::Vector3>("/debug/vel", 10);
  pub_path = n.advertise<nav_msgs::Path>("path", 10);
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/est_pose", 10);
  vel_pub = n.advertise<geometry_msgs::TwistStamped>("/est_vel", 10);
  vicon_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/vicon/est_pose", 10);
  vicon_vel_pub = n.advertise<geometry_msgs::TwistStamped>("/vicon/est_vel", 10);
  QTworldslam_pub = n.advertise<geometry_msgs::PoseStamped>("/qtworldslam",1);

  path_msg.header.frame_id = "world";

  //test
  test_slam_pub = n.advertise<geometry_msgs::Vector3>("/slam_rpy",10);
  test_gt_pub = n.advertise<geometry_msgs::Vector3>("/gt_rpy",10);
  slam_pos_pub = n.advertise<geometry_msgs::PoseStamped>("/slam_modify_pose",10);

#if SIMULATION
  ros::Subscriber sub_imu = n.subscribe("/drone/imu", 100, imuCallback);
  ros::Subscriber pose_sub = n.subscribe("/drone/gt_pose", 1, gtposeCallback);
  ros::Subscriber vel_sub = n.subscribe("/drone/gt_vel", 1, gtvelCallback);
#endif
  
#if ARDRONE
  ros::Subscriber sub_imu = n.subscribe("/ardrone/imu", 100, imuCallback);
  ros::Subscriber sub_navdata = n.subscribe("ardrone/navdata", 100, navdataCallback);
  ros::Subscriber sub_navaltdata = n.subscribe("/ardrone/navdata_altitude", 100, navaltdataCallback);
  ros::Subscriber sub_odometrydata = n.subscribe("/ardrone/odometry", 100, odometryCallback);
#endif

#if PIONEER3AT
  ros::Subscriber sub_imu = n.subscribe("/mavros/imu/data", 1, imuCallback);
  ros::Subscriber sub_p3atpose = n.subscribe("/RosAria/pose", 1, p3atPoseCallback);
  //ros::Subscriber sub_mavrosheight = n.subscribe("/mavros/px4flow/ground_distance", 10, mavrosHeightCallback);
  ros::Subscriber sub_slamtecyaw = n.subscribe("/slamtec/yaw", 5, slamtecYawCallback);
#endif

  ros::Subscriber brainok_sub=n.subscribe("/brain/ok", 1, brainokCallback);
  ros::Subscriber slam_sub=n.subscribe("/slam/pose", 1, slamCallback);
  ros::Subscriber correctPoseData_sub=n.subscribe("/autoreturn/correctpose", 1,correctPoseDataCallback);
  ros::Subscriber hoverCorrectPoseData_sub=n.subscribe("/visionhover/correctpose", 1,hoverCorrectPoseDataCallback);

  n.getParam("use_gps",USE_GPS);
  n.getParam("use_vicon",USE_VICON);
  n.getParam("drift_path",drift_path);
  n.getParam("geoidsPath",geoidsPath);
  n.getParam("errCamToDroneCenter", errCamToDroneCenter);
  n.getParam("wheel_type", p3at_speed_modify);
  drift_file.open(drift_path.c_str());

  bool ret = loadModels();
  if(!ret) return -1;

  ros::Rate loop_rate(100);
  sleep(1);
  ROS_INFO("[POS_EKF]pos_ekf start!");
  while(ros::ok())
  {
    ros::spinOnce();
    //if(!brain_ok) continue;

    //cnt = 0;
    if(cnt_slam < 100){
        ++cnt_slam;
    }
    else if(cnt_slam < 101)
    {
        ++cnt_slam;
        getSLAMRTdone = false;
        Twslam[0] = 0;
        Twslam[1] = 0;
        Twslam[2] = 0;
        ROS_INFO("[POS_EKF]miss slam data!");
    }
    if(cnt_slamtecyaw < 50){
        ++cnt_slamtecyaw;
    }
    else if(cnt_slamtecyaw < 51){
        ++cnt_slamtecyaw;
        isfirstslamtecsub = true;
        ROS_INFO("[POS_EKF]miss lidar yaw");
    }
    while(processSensorData()){}//++cnt;
    publish_pose(pose_ekf);
    //cout << "process data " <<cnt<< " times"<<endl;
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}

