
/*This code is used to plan the trajectory of the robot  
*/

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>

#include <geometry_msgs/Twist.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace tf;

double x, y, z, ww, zz, hh, ii, Aww, Azz, Ahh, Aii;
double theta;
Eigen::Vector3d t;

float x_current;
float y_current;

float normeNextGoal;

class quaternion_ros
{
public:
  float w;
  float x;
  float y;
  float z;

  quaternion_ros();

  void toQuaternion(float pitch, float roll, float yaw);
};

void quaternion_ros::toQuaternion(float pitch, float roll, float yaw)
{

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  w = cy * cr * cp + sy * sr * sp;
  x = cy * sr * cp - sy * cr * sp;
  y = cy * cr * sp + sy * sr * cp;
  z = sy * cr * cp - cy * sr * sp;
}

quaternion_ros::quaternion_ros()
{
  w = 1;
  x = 0;
  y = 0;
  z = 0;
}

class Path_planned
{
public:
  struct Goal
  {
    float x;
    float y;
    bool visited;
  };

  vector<Goal> Path;

  Path_planned();
  //Path_planned(float x, float y, bool visited);
  void addGoal(float X, float Y, bool visit);
};

Path_planned::Path_planned()
{
}

//Path_planned(float x, float y, bool visited)

void Path_planned::addGoal(float X, float Y, bool visit)
{
  Path_planned::Goal newGoal;
  newGoal.x = X;
  newGoal.y = Y;
  newGoal.visited = visit;
  Path.push_back(newGoal);
}

Path_planned planned_path;
nav_msgs::Path passed_path;
geometry_msgs::PoseStamped p;
ros::Publisher pub_passed_path;
void pose_callback(const nav_msgs::Odometry &poses)
{ //里程计回调函数,用来计算当前机器人位置与前面目标点的距离,判断是否要发新的幕摆点
  x_current = poses.pose.pose.position.x;
  y_current = poses.pose.pose.position.y;
  passed_path.header = poses.header;

  p.header = poses.header;
  p.pose = poses.pose.pose;
  passed_path.poses.emplace_back(p);
  pub_passed_path.publish(passed_path);
}

int taille_last_path = 0;
bool new_path = false;

//接受规划的路径
void path_callback(const nav_msgs::Path &path)
{
  //注意为了rviz显示方便 路径一直在发,但是这里只用接受一次就好,当规划的路径发生变化时候再重新装载
  if ((planned_path.Path.size() == 0) || (path.poses.size() != taille_last_path))
  {
    planned_path.Path.clear();
    new_path = true;
    for (int i = 0; i < path.poses.size(); i++)
    {
      planned_path.addGoal(path.poses[i].pose.position.x, path.poses[i].pose.position.y, false);

      cout << path.poses[i].pose.position.x << " " << path.poses[i].pose.position.y << endl;
    }
    cout << "Recv path size:" << path.poses.size() << endl;
    taille_last_path = path.poses.size();
  }
}

// int **count_antonin(char *)

int main(int argc, char *argv[])
{
  srand(time(0));
  ros::init(argc, argv, "next_goal");
  ros::NodeHandle next_goal;
  ros::Subscriber sub1 = next_goal.subscribe("/odom", 1000, pose_callback);
  ros::Subscriber sub2 = next_goal.subscribe("/path_planning_node/cleaning_plan_nodehandle/cleaning_path", 1000, path_callback);

  ros::Publisher pub1 = next_goal.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
  ros::Publisher pub2 = next_goal.advertise<geometry_msgs::Pose>("/ai_robot/findpath/targetP", 1000);
  pub_passed_path = next_goal.advertise<nav_msgs::Path>("/clean_robot/passed_path", 1000);

  tf::TransformListener listener;

  ros::Rate loop_rate(10);

  geometry_msgs::PoseStamped goal_msgs;
  geometry_msgs::Pose local_goal_msgs;
  int count = 0;
  double angle;
  bool goal_reached = false;
  tf::Quaternion q_map2base;
  Eigen::Matrix3d rotationMatrix;
  Eigen::Matrix3d rotationMatrix_next_goal;
  Eigen::Matrix3d rotationMatrix_next_goal_local;
  Eigen::Matrix4d transformMatrix_base;
  Eigen::Matrix4d transformMatrix_base_inverse;
  Eigen::Matrix4d transformMatrix_next_goal;
  Eigen::Matrix4d transformMatrix_next_goal_local;


  //获取发送下一个点的阈值
  if (!next_goal.getParam("/NextGoal/tolerance_goal", normeNextGoal))
  {
    ROS_ERROR("Please set your tolerance_goal");
    return 0;
  }
  ROS_INFO("tolerance_goal=%f", normeNextGoal);

  while (ros::ok())
  {
    ros::spinOnce();

    tf::StampedTransform transform;
    try {
        listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    t[0] = transform.getOrigin().x();
    t[1] = transform.getOrigin().y();
    t[2] = 0;
    q_map2base = transform.getRotation();
    Eigen::Quaterniond rotation_q(q_map2base.w(), q_map2base.x(), q_map2base.y(), q_map2base.z());
    rotationMatrix = rotation_q.toRotationMatrix();
    transformMatrix_base << rotationMatrix(0,0), rotationMatrix(0,1), rotationMatrix(0,2), x,
                       rotationMatrix(1,0), rotationMatrix(1,1), rotationMatrix(1,2), y,
                       rotationMatrix(2,0), rotationMatrix(2,1), rotationMatrix(2,2), z,
                       0,0,0,1;

    transformMatrix_base_inverse << rotationMatrix.transpose(), -rotationMatrix.transpose()*t, 0, 0, 0, 1;

    if (new_path)
    {
      count = 0;
      new_path = false;
    }
    //当前处理的点
    cout << " count : " << count << endl;
    if (!planned_path.Path.empty())
    {
      //当前距离达到了
      if (sqrt(pow(x_current - planned_path.Path[count].x, 2) + pow(y_current - planned_path.Path[count].y, 2)) <= normeNextGoal)
      {
        count ++;
        goal_reached = false;
      }
      if (goal_reached == false)
      {
        goal_msgs.header.frame_id = "odom";
        goal_msgs.header.stamp = ros::Time::now();
        goal_msgs.pose.position.x = planned_path.Path[count].x;
        goal_msgs.pose.position.y = planned_path.Path[count].y;
        goal_msgs.pose.position.z = 0;
        if (count < planned_path.Path.size())
        {//计算发布的yaw，不过还有bug 但是不影响使用，yaw不会产生太大影响
          angle = atan2(planned_path.Path[count + 1].y - planned_path.Path[count].y, planned_path.Path[count + 1].x - planned_path.Path[count].x);
        }
        else
        {
          angle = atan2(planned_path.Path[0].y - planned_path.Path[count].y, planned_path.Path[0].x - planned_path.Path[count].x);
        }
        cout << angle << endl;
        quaternion_ros q;
        q.toQuaternion(0, 0, float(angle));
        goal_msgs.pose.orientation.w = q.w;
        goal_msgs.pose.orientation.x = q.x;
        goal_msgs.pose.orientation.y = q.y;
        if (planned_path.Path[count].x < planned_path.Path[count + 1].x)
        {
          goal_msgs.pose.orientation.z = 0;
        }
        if (planned_path.Path[count].x > planned_path.Path[count + 1].x)
        {
          goal_msgs.pose.orientation.z = 2;
        }

        Eigen::Quaterniond rotation_q_next_goal(goal_msgs.pose.orientation.w, goal_msgs.pose.orientation.x, goal_msgs.pose.orientation.y, goal_msgs.pose.orientation.z);
        rotationMatrix_next_goal = rotation_q_next_goal.toRotationMatrix();
        transformMatrix_next_goal << rotationMatrix_next_goal(0,0), rotationMatrix_next_goal(0,1), rotationMatrix_next_goal(0,2), goal_msgs.pose.position.x,
                                     rotationMatrix_next_goal(1,0), rotationMatrix_next_goal(1,1), rotationMatrix_next_goal(1,2), goal_msgs.pose.position.y,
                                     rotationMatrix_next_goal(2,0), rotationMatrix_next_goal(2,1), rotationMatrix_next_goal(2,2), goal_msgs.pose.position.z,
                                     0,0,0,1;

        transformMatrix_next_goal_local = transformMatrix_base_inverse * transformMatrix_next_goal;
        rotationMatrix_next_goal_local << transformMatrix_next_goal_local(0,0), transformMatrix_next_goal_local(0,1), transformMatrix_next_goal_local(0,2),
                                          transformMatrix_next_goal_local(1,0), transformMatrix_next_goal_local(1,1), transformMatrix_next_goal_local(1,2),
                                          transformMatrix_next_goal_local(2,0), transformMatrix_next_goal_local(2,1), transformMatrix_next_goal_local(2,2);
        local_goal_msgs.position.x = transformMatrix_next_goal_local(0,3);
        local_goal_msgs.position.y = transformMatrix_next_goal_local(1,3);
        local_goal_msgs.position.z = transformMatrix_next_goal_local(2,3);
        Eigen::Quaterniond q_local(rotationMatrix_next_goal_local);
        local_goal_msgs.orientation.w = q_local.w();
        local_goal_msgs.orientation.x = q_local.x();
        local_goal_msgs.orientation.y = q_local.y();
        local_goal_msgs.orientation.z = q_local.z();

        cout << " NEW GOAL " << endl;
        cout << " x = " << planned_path.Path[count].x << " y = " << planned_path.Path[count].y << endl;
        goal_reached = true;
        pub1.publish(goal_msgs);
        pub2.publish(local_goal_msgs);
      }

      cout << x_current << " " << y_current << endl;
      //当前
      cout << planned_path.Path[count].x << " " << planned_path.Path[count].y << endl;
      //目标
      cout << " DISTANCE : " << sqrt((x_current - planned_path.Path[count].x) * (x_current - planned_path.Path[count].x) + (y_current - planned_path.Path[count].y) * (y_current - planned_path.Path[count].y)) << endl;
      // 距离公式
    }
    loop_rate.sleep();
  }
  return 0;
}
