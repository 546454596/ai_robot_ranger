#ifndef FINDPATHSRM_H
#define FINDPATHSRM_H

#include <iostream>
#include <cstdlib>
#include <time.h>
#include <fstream>

#include "Octree.hpp"

#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include "assistMath.h"

using namespace std;

class PathNode
{
public:
    PathNode(float _x, float _y, float _z, int _id);
    ~PathNode();
    void setxyzid(float _x, float _y, float _z, int _id);
    void addLink(int _id_link, float _dist);
    void removeLink(int _id_link);
    //for A star or Dijkstra
    float distTilNow, distToEnd, distTotal;
    int id_fromWhere;
    //link between this node and the other node
    vector<int> id_link;
    //store distance between node
    vector<float> dist_link;
    //flag represent wether the node is dealed
    bool isDealed;
    float x, y, z;
private:
    //location in the queue
    int id_self;
};

class FindPathSRM
{
public:
    FindPathSRM(ros::NodeHandle &_n, float _startx, float _starty, float _startz,
                float _endx, float _endy, float _endz, bool _isview);
    FindPathSRM(ros::NodeHandle &_n, bool _isview);
    ~FindPathSRM();

    //find path, before findpath, make sure start point and end point are set
    bool findPath();
    //display point cloud
    void display();
    //reset
    void resetAll(float _startx, float _starty, float _startz,
                  float _endx, float _endy, float _endz);
    //get target speed acording to position now,
    //if return true, get v; if false, to destination and get target pose
    //input vx,vy,vz should be vel_now initially
    bool getTargetSpeed(float _nowx, float _nowy, float _nowz, float _nowyaw,
                     float &vx, float &vy, float &vz, float &vyaw);
    void setPosForVisual(float _nowx, float _nowy, float _nowz, float _nowyaw);
    double getShortestPathLength();
    // reload graph
    void reloadGraph(string filepath);
private:
    ros::NodeHandle nh;
    ros::Subscriber restart_sub, reloadmap_sub;
    ros::Publisher targetP_pub;

    //the start and end position
    pcl::PointXYZRGB startP, endP;
    pcl::PointXYZRGB nowP, nowtarP, nowvelP, nowdirP, nowfviP, nowleftP, nowupP;
    float Rwc[9];
    int pathid_near_node;
    bool isNewTar;
    float velXYZ_now[3];
    //nearest node from start/end position
    int id_startNode, id_endNode;
    //target point id when move along path
    int idp_targetNode;
    //id of node along path, link is (end position)<-(path.begin)<-(path.end)<-(start position)
    vector<int> path;
    vector<pcl::PointXYZ> path_point;
    //keyframe position, the last one is destination
    vector<PathNode> pNode;
    //from keyframehandle for view, keyPosPC<->denkeyfPC, keyPosLink<->denkeyfLine
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPosPC;
    vector< vector<int> > keyPosLink;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapPC;
    unibn::Octree<pcl::PointXYZRGB> mapOct;
    //unibn::Octree<pcl::PointXYZRGB> keyPosOct;
    float mappointSparse;
    //for pcl viewer
    int v1;
    bool isview, isDealView, isFirstView;
    //for time
    int64_t time_begin, time_end;
    //for calculate time consume of calculate potantial
    double time_calpotan_total, time_maxcalpo;
    int cnt_calpotan;

    //for calculate potantial
    /*obstacle
     * potantial = 1 / (1 + exp(distance - adist)*bdist)
     *start/end
     * potantial = 1 / (1 + exo(distance - adist*halfdist)*bdist)
     */
    float astartdist, bstartdist, aenddist, benddist, aobsdist, bobsdist, halfStartEndDist;

    //for calDpotantial2
    int kseg, ksegnow;
    float loneseg;

    //log file
    ofstream mylog;
    string _logfile, _denKeyfPos, _denKeyfPosRelation, _MapPointsPos, _aParamForKeyframeHandle;

    int justReplan;

    bool isAllFileExit;
    //read file and reconstruct node graph
    bool reconstructGraph();
    //read key pos
    bool readPoint();
    //read link between key pos
    bool readLink();
    //read map point for view
    bool readMapPoint();
    //read parameters from file
    bool readParams();
    //init pcl viewer
    void initPclViewer();
    void initService();
    //for i=12345, change str(line00000) to str(line12345)
    void setstring(string &str, int k);
    //
    void restartNavCallback(const geometry_msgs::Pose::ConstPtr msg);
    // reload map from the path of msg
    void reloadCB(const std_msgs::StringConstPtr& msg);
    // clear data before reload
    void clearall();

    //find start and end node
    template <typename Distance>
    bool findStartEndNode();
    //a star
    template <typename Distance>
    bool astar();
    //insert and sort path node in queue by distTotal, large at begin, small at end
    void insertSortByDistTotal(int _id, vector<int> &_nodeQueue);

    //calculate derivation of potantial
    void calDPotantial2(float _nowx, float _nowy, float _nowz,
                        float _tarx, float _tary, float _tarz,
                        float &vx, float &vy, float &vz);

    //calculate yaw according to v
    void calYaw(float vx, float vy, float vz, float &yaw);
    //return yaw speed
    float calYawSpeed(float yaw_tar, float yaw_now);

    //for test
    //show node
    void testShowNode();
    //show queue
    void testShowQueue(vector<int> &vec);

    //pub target point for obstacle avoid
    void pubTargetP();
};


#endif
