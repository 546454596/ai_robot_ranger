#include "findpathsrm.h"

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Map Viewer"));
const unibn::OctreeParams& mapOctParams = unibn::OctreeParams(30, false, 0.02f);
//const unibn::OctreeParams& keyPosOctParams = unibn::OctreeParams(10, false, 0.02f);


PathNode::PathNode(float _x, float _y, float _z, int _id):
    x(_x),y(_y),z(_z),id_self(_id), id_fromWhere(-1), isDealed(false),
    distTilNow(-1), distToEnd(-1), distTotal(-1)
{}

PathNode::~PathNode()
{}

void PathNode::setxyzid(float _x, float _y, float _z, int _id)
{
    x = _x;
    y = _y;
    z = _z;
    id_self = _id;
}

void PathNode::addLink(int _id_link, float _dist)
{
    id_link.push_back(_id_link);
    dist_link.push_back(_dist);
}

void PathNode::removeLink(int _id_link)
{
    for(int i=0; i<id_link.size(); ++i){
        if(id_link[i] == _id_link){
            id_link.erase(id_link.begin()+i);
            dist_link.erase(dist_link.begin()+i);
        }
    }
}

FindPathSRM::FindPathSRM(ros::NodeHandle &_n, float _startx, float _starty, float _startz,
                         float _endx, float _endy, float _endz, bool _isview):
    nh(_n),
    isAllFileExit(false),
    startP(200,0,200), endP(100, 50, 50), id_startNode(-1), id_endNode(-1), isview(_isview),
    mappointSparse(5.0), isDealView(0), isNewTar(0), isFirstView(1),
    idp_targetNode(0), Rwc{1,0,0,0,1,0,0,0,1}, justReplan(0),
    astartdist(0.5), bstartdist(0.05), aenddist(1.5), benddist(0.05), aobsdist(1), bobsdist(2),
    keyPosPC(new pcl::PointCloud<pcl::PointXYZRGB>),
    mapPC(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    startP.x = _startx;
    startP.y = _starty;
    startP.z = _startz;
    endP.x = _endx;
    endP.y = _endy;
    endP.z = _endz;
    nowP.x = 0;
    nowP.y = 0;
    nowP.z = 0;
    nowtarP.x = 0;
    nowtarP.y = 0;
    nowtarP.z = 0;
    nowvelP.x = 0;
    nowvelP.y = 0;
    nowvelP.z = 0;
    nowdirP.x = 0;
    nowdirP.y = 0;
    nowdirP.z = 0;
    nh.getParam("logfile", _logfile);
    nh.getParam("denKeyfPos", _denKeyfPos);
    nh.getParam("denKeyfPosRelation", _denKeyfPosRelation);
    nh.getParam("MapPointsPos", _MapPointsPos);
    nh.getParam("aParamForKeyframeHandle", _aParamForKeyframeHandle);
//    if(!reconstructGraph()){return;}
    mylog.open(_logfile);
    initPclViewer();
    reconstructGraph();
    initService();
//    mapOct.initialize(mapPC->points, mapOctParams);
    //keyPosOct.initialize(keyPosPC->points, keyPosOctParams);
}

FindPathSRM::FindPathSRM(ros::NodeHandle &_n, bool _isview):
    nh(_n),
    isAllFileExit(false),
    startP(0,0,0), endP(0, 0, 0), id_startNode(-1), id_endNode(-1), isview(_isview),
    mappointSparse(5.0), isDealView(0), isNewTar(0), isFirstView(1),
    idp_targetNode(0), Rwc{1,0,0,0,1,0,0,0,1}, justReplan(0),
    ksegnow(0), kseg(0),
    astartdist(0.5), bstartdist(0.05), aenddist(1.5), benddist(0.05), aobsdist(1), bobsdist(2),
    keyPosPC(new pcl::PointCloud<pcl::PointXYZRGB>),
    mapPC(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    nh.getParam("logfile", _logfile);
    nh.getParam("denKeyfPos", _denKeyfPos);
    nh.getParam("denKeyfPosRelation", _denKeyfPosRelation);
    nh.getParam("MapPointsPos", _MapPointsPos);
    nh.getParam("aParamForKeyframeHandle", _aParamForKeyframeHandle);
//    if(!reconstructGraph()){return;}
    initPclViewer();
    reconstructGraph();
    nowP.x = 0;
    nowP.y = 0;
    nowP.z = 0;
    nowtarP.x = 0;
    nowtarP.y = 0;
    nowtarP.z = 0;
    nowdirP.x = 0;
    nowdirP.y = 0;
    nowdirP.z = 0;
    initService();
    mylog.open(_logfile);
//    mapOct.initialize(mapPC->points, mapOctParams);
    //keyPosOct.initialize(keyPosPC->points, keyPosOctParams);
}

FindPathSRM::~FindPathSRM()
{
    mylog.close();
}

void FindPathSRM::initService()
{
    reloadmap_sub = nh.subscribe("/topo/savedone", 1, &FindPathSRM::reloadCB, this);
    restart_sub = nh.subscribe("/ai_robot/restart_nav", 1, &FindPathSRM::restartNavCallback, this);
    targetP_pub = nh.advertise<geometry_msgs::Pose>("/ai_robot/findpath/targetP",1);
}

double FindPathSRM::getShortestPathLength()
{
    return pNode[id_endNode].distTilNow;
}

bool FindPathSRM::findPath()
{
    if(!isAllFileExit)
    {
        cout << "[FINDPATH]some files are not exits" <<endl;
        return 0;
    }
    //testShowNode();
    int64_t start1=0,end1=0;
    start1 = cv::getTickCount();
    if(findStartEndNode<unibn::L2Distance<pcl::PointXYZRGB> >())
    {
        end1 = cv::getTickCount();
        double _t = 1000*double(end1 - start1)/cv::getTickFrequency();
        // cout << "search start end node time:" << _t <<" ms."<< endl;
        start1 = end1;
        mylog << _t << " ";
        // mylog << "-----start new find path task-----" <<endl;
        if(!astar<unibn::L2Distance<pcl::PointXYZRGB> >()){
            mylog << 0 << endl;
            // fail to find path
            return 0;
        }
    }
    else
    {
        end1 = cv::getTickCount();
        double _t = 1000*double(end1 - start1)/cv::getTickFrequency();
        mylog << _t << " ";
        cout << "Start/End position is not reachable!" << endl;
        return 0;
    }
    end1 = cv::getTickCount();
    double _t = 1000*double(end1 - start1)/cv::getTickFrequency();
    mylog << _t << endl;
    // cout << "search path time:" << _t <<" ms."<< endl;
    // mylog << "search path time:" << _t << " ms." << endl;
    return 1;
}

void FindPathSRM::setPosForVisual(float _nowx, float _nowy, float _nowz, float _nowyaw)
{
    nowfviP.x = _nowx;
    nowfviP.y = _nowy;
    nowfviP.z = _nowz;
    nowdirP.z = _nowz;
    Rwc[0] = cos(_nowyaw);
    Rwc[1] = -sin(_nowyaw);
    Rwc[3] = sin(_nowyaw);
    Rwc[4] = cos(_nowyaw);
    nowdirP.x = _nowx + 2*Rwc[0];//cos(_nowyaw);
    nowdirP.y = _nowy + 2*Rwc[3];//sin(_nowyaw);
    nowleftP.z = _nowz;
    nowleftP.x = _nowx - 1.5*Rwc[3];//sin(_nowyaw);
    nowleftP.y = _nowy + 1.5*Rwc[0];//cos(_nowyaw);
    nowupP.x = _nowx;
    nowupP.y = _nowy;
    nowupP.z = _nowz + 1.5;
    if(justReplan>0){
        --justReplan;
    }
}

void FindPathSRM::display()
{
if(isAllFileExit)
{
    if(isFirstView)
    {
        viewer->addLine(nowfviP, nowdirP, 200, 0, 0, "linedir", v1);
        viewer->addLine(nowfviP, nowleftP, 0, 0, 170, "lineleft", v1);
        viewer->addLine(nowfviP, nowupP, 0, 0, 170, "lineup", v1);
        viewer->addLine(nowfviP, nowfviP, 200, 0, 170, "linenode", v1);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 7, "linedir", v1);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 7, "lineleft", v1);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 7, "lineup", v1);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "linenode", v1);
        viewer->setCameraPosition(nowfviP.x-2*(nowdirP.x-nowfviP.x)-2*Rwc[1], nowfviP.y-2*(nowdirP.y-nowfviP.y)+2*Rwc[0], nowfviP.z+15,
                                  nowdirP.x, nowdirP.y, nowdirP.z,
                                  0, 0, 1);
        isFirstView = false;
    }
    else
    {
        viewer->removeShape("linedir", v1);
        viewer->addLine(nowfviP, nowdirP, 200, 0, 0, "linedir", v1);
        viewer->removeShape("lineleft", v1);
        viewer->addLine(nowfviP, nowleftP, 0, 0, 170, "lineleft", v1);
        viewer->removeShape("lineup", v1);
        viewer->addLine(nowfviP, nowupP, 0, 0, 170, "lineup", v1);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 7, "linedir", v1);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 7, "lineleft", v1);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 7, "lineup", v1);
        if(path_point.size()>0){
            viewer->removeShape("linenode", v1);
            viewer->addLine(nowfviP, path_point[pathid_near_node], 20, 0, 17, "linenode", v1);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "linenode", v1);
        }
    }
    if(!isDealView && isview)
    {
        //display map point and dense keyframe position
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pchmapPC(mapPC);
        viewer->addPointCloud<pcl::PointXYZRGB> (mapPC, pchmapPC, "origin map point cloud", v1);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "origin map point cloud", v1);

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pchkeyPosPC(keyPosPC);
        viewer->addPointCloud<pcl::PointXYZRGB> (keyPosPC, pchkeyPosPC, "keyframe pos", v1);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keyframe pos", v1);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr startendPC(new pcl::PointCloud<pcl::PointXYZRGB>);
        startendPC->push_back(startP);
        startendPC->push_back(endP);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pchsePC(startendPC);
        viewer->addPointCloud<pcl::PointXYZRGB> (startendPC, pchsePC, "startend", v1);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "startend", v1);

        viewer->addCoordinateSystem (1.0, "origin cloud", v1);
        //add line of keyframe pos
        string li("line00000");
        for(int i=0; i<keyPosLink.size(); ++i)
        {
            setstring(li, i);
            viewer->addLine(keyPosPC->points[keyPosLink[i][0]], keyPosPC->points[keyPosLink[i][1]], 0, 255, 0, li, v1);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, li, v1);
        }

        //add path line
        if(path.size()>=2)
        {
            string pli("path00000");
            //cout << "Path(node number):";
            for(int j=0; j<=path.size()-2; ++j)  //the first one is end point and last one is start point
            {
                //cout << path[j] << "<-";
                setstring(pli, j);
                viewer->addLine(path_point[j], path_point[j+1], 200, 0, 200, pli, v1);
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, pli, v1);
            }
            cout << path[path.size()-1] << endl;
        }

        viewer->addLine(nowP, nowtarP, 0, 200, 25, "nowline", v1);
        viewer->addLine(nowP, nowvelP, 30, 0, 100, "velline", v1);

        isDealView = true;
    }

    if(isNewTar)
    {
        viewer->removeShape("nowline", v1);
        viewer->addLine(nowP, nowtarP, 0, 200, 25, "nowline", v1);
        isNewTar = false;
    }
    viewer->removeShape("velline", v1);
    viewer->addLine(nowP, nowvelP, 30, 0, 100, "velline", v1);

    //while(isview)
    {
        viewer->spinOnce (1);
    }
}
}

void FindPathSRM::resetAll(float _startx, float _starty, float _startz,
                           float _endx, float _endy, float _endz)
{
    startP.x = _startx;
    startP.y = _starty;
    startP.z = _startz;
    endP.x = _endx;
    endP.y = _endy;
    endP.z = _endz;
    idp_targetNode = 0;
    cnt_calpotan = 0;
    time_calpotan_total = 0;
    time_maxcalpo = 0;
    ksegnow = 0;
    kseg = 0;
    id_endNode = -1;
    id_startNode = -1;
    pathid_near_node = 0;

    if(isAllFileExit)
    {
        pNode[pNode.size()-1].x = _endx;
        pNode[pNode.size()-1].y = _endy;
        pNode[pNode.size()-1].z = _endz;
        pNode[pNode.size()-2].x = _startx;
        pNode[pNode.size()-2].y = _starty;
        pNode[pNode.size()-2].z = _startz;
        path.clear();
        path_point.clear();
    if(isDealView)
    {
        viewer->removeAllShapes(v1);
        viewer->removeAllPointClouds(v1);
        isDealView = 0;
    }

    for(int i=0; i<pNode.size(); ++i)
    {
        pNode[i].isDealed = false;
        pNode[i].distTilNow = -1;
    }
    }
}

bool FindPathSRM::getTargetSpeed(float _nowx, float _nowy, float _nowz, float _nowyaw,
                     float &vx, float &vy, float &vz, float &vyaw)
{
    int64_t start1=0,end1=0;
    start1 = cv::getTickCount();
    pcl::PointXYZRGB tmp;
    nowP.x = _nowx;
    nowP.y = _nowy;
    nowP.z = _nowz;
    tmp.x = _nowx;
    tmp.y = _nowy;
    tmp.z = _nowz;
    velXYZ_now[0] = vx;
    velXYZ_now[1] = vy;
    velXYZ_now[2] = vz;
    float _dist = sqrt(pow(_nowx-path_point[idp_targetNode].x, 2) +
                       pow(_nowy-path_point[idp_targetNode].y, 2));// +pow(_nowz-path_point[idp_targetNode].z, 2)
    float yaw_tar;
    //update which node drone is in
    if(pathid_near_node>1){
        float nodedist = (pow(_nowx-path_point[pathid_near_node].x, 2) +
                pow(_nowy-path_point[pathid_near_node].y, 2));// + pow(_nowz-path_point[pathid_near_node].z, 2)
        float nextnodedist = (pow(_nowx-path_point[pathid_near_node-1].x, 2) +
                pow(_nowy-path_point[pathid_near_node-1].y, 2));// + pow(_nowz-path_point[pathid_near_node-1].z, 2)
        if(nextnodedist < nodedist){
            --pathid_near_node;
        }
    }
    if(idp_targetNode >= 0)
    {
        pcl::PointXYZRGB tar;
        if(ksegnow > (kseg-1)) //cal next segment path
        {
            if(idp_targetNode == 0){
                if(_dist < 0.5){
                    vx = endP.x;
                    vy = endP.y;
                    vz = endP.z;
                    vyaw = _nowyaw;
                    cout << "adverage calculate potantial time:" << time_calpotan_total/cnt_calpotan
                         << " ms, max calculate time:" << time_maxcalpo <<" ms."<< endl;
                    mylog << "to target" << endl << endl;
                    time_maxcalpo = 0;
                    cnt_calpotan = 0;
                    time_calpotan_total = 0;
                    return false;
                }
                else{
                    tar.x = endP.x;
                    tar.y = endP.y;
                    tar.z = endP.z;
                }
            }
            else{
                --idp_targetNode;
                ksegnow = 0;
                _dist = sqrt(pow(path_point[idp_targetNode].x - path_point[idp_targetNode+1].x,2)
                        + pow(path_point[idp_targetNode].y - path_point[idp_targetNode+1].y,2));//+ pow(path_point[idp_targetNode].z - path_point[idp_targetNode+1].z,2)
                kseg = _dist / loneseg;
                tar.x = path_point[idp_targetNode+1].x;
                tar.y = path_point[idp_targetNode+1].y;
                tar.z = path_point[idp_targetNode+1].z;
            }
            mylog << "~~~~next segment:" << idp_targetNode << " node. "
                  << path_point[idp_targetNode].x << ","
                  << path_point[idp_targetNode].y << "," << path_point[idp_targetNode].z
                  << "---kseg:" << kseg <<endl;
        }
        else //cal this segment path
        {
            ++ksegnow;
            tar.x = (path_point[idp_targetNode].x*ksegnow + path_point[idp_targetNode+1].x*(kseg-ksegnow)) / kseg;
            tar.y = (path_point[idp_targetNode].y*ksegnow + path_point[idp_targetNode+1].y*(kseg-ksegnow)) / kseg;
            tar.z = (path_point[idp_targetNode].z*ksegnow + path_point[idp_targetNode+1].z*(kseg-ksegnow)) / kseg;
            _dist = sqrt(pow(_nowx-tar.x, 2) +
                       pow(_nowy-tar.y, 2));// + pow(_nowz-tar.z, 2)
            //if blocked, back to former point|| ksegnow > (kseg)
            if(_dist > 3  || mapOct.isBlock<unibn::L2Distance<pcl::PointXYZRGB> >(tmp, tar, 1.2*mappointSparse))
            {
                --ksegnow;
                //if(ksegnow < 0) ksegnow = 0;
                tar.x = (path_point[idp_targetNode].x*ksegnow + path_point[idp_targetNode+1].x*(kseg-ksegnow)) / kseg;
                tar.y = (path_point[idp_targetNode].y*ksegnow + path_point[idp_targetNode+1].y*(kseg-ksegnow)) / kseg;
                tar.z = (path_point[idp_targetNode].z*ksegnow + path_point[idp_targetNode+1].z*(kseg-ksegnow)) / kseg;
            }
            mylog << "to:" << tar.x <<","<< tar.y <<","<< tar.z << "---ksegnow:"<< ksegnow <<endl;
        }
        calDPotantial2(_nowx, _nowy, _nowz, tar.x, tar.y, tar.z, vx, vy, vz);
        calYaw(vx, vy, vz, yaw_tar);
        vyaw = calYawSpeed(yaw_tar, _nowyaw);
        nowtarP.x = tar.x;
        nowtarP.y = tar.y;
        nowtarP.z = tar.z;
        isNewTar = true;
    }

    end1 = cv::getTickCount();
    double _ti = 1000*double(end1 - start1)/cv::getTickFrequency();
    pubTargetP();
    //double _ti = ((double)(time_end - time_begin) / CLOCKS_PER_SEC);
    if(_ti > time_maxcalpo){
        time_maxcalpo = _ti;
    }
    time_calpotan_total += _ti;
    ++cnt_calpotan;

    nowvelP.x = nowP.x + vx*2;
    nowvelP.y = nowP.y + vy*2;
    nowvelP.z = nowP.z + vz*2;

    mylog << "target position:" << pNode[path[idp_targetNode]].x << ","
          << pNode[path[idp_targetNode]].y << "," << pNode[path[idp_targetNode]].z
          << ",position now:" << _nowx << "," << _nowy << "," << _nowz << endl
          << "target speed:" << vx << "," << vy << "," << vz
          << " = " << sqrt(vx*vx + vy*vy) << " ==yaw now:"
          << _nowyaw << ", yaw tar:" << yaw_tar << ", yaw speed:" << vyaw << endl;
    viewer->setCameraPosition(nowfviP.x-2*(nowdirP.x-nowfviP.x)-2*Rwc[1], nowfviP.y-2*(nowdirP.y-nowfviP.y)+2*Rwc[0], nowfviP.z+15,
                                  nowdirP.x, nowdirP.y, nowdirP.z,
                                  0, 0, 1);
    return true;
}

void FindPathSRM::pubTargetP(){
    //publish target pose in body frame
    geometry_msgs::Pose tarp_msg;
    float twc[3]={nowP.x,nowP.y,nowP.z}, tarp_frombody[3];
    //float rrwc[9]={Rwc[0],-Rwc[1],0, -Rwc[3],Rwc[4],0, 0,0,1};
    transform_body_from_NWUworld(tarp_frombody[0], tarp_frombody[1], tarp_frombody[2],
            nowtarP.x,nowtarP.y,nowtarP.z, Rwc, twc);
    //cout<<".........."<<nowtarP.x<<","<<nowtarP.y<<","<<nowtarP.z<<endl;
    //cout<<"[fp]rt:"<<twc[0]<<","<<twc[1]<<","<<twc[2]<<"; "<<Rwc[0]<<","<<Rwc[1]
    //   <<","<<Rwc[3]<<","<<Rwc[4]<<endl;
    //cout<<"local target:"<<tarp_frombody[0]<<","<<tarp_frombody[1]<<","<<tarp_frombody[2]<<endl;
    tarp_msg.position.x = tarp_frombody[0];
    tarp_msg.position.y = tarp_frombody[1];
    tarp_msg.position.z = tarp_frombody[2];
    float taryawp_frombody[3], _q[4], tardir[3] = {path_point[idp_targetNode].x,path_point[idp_targetNode].y,path_point[idp_targetNode].z};
    transform_body_from_NWUworld(taryawp_frombody[0], taryawp_frombody[1], taryawp_frombody[2],
            tardir[0], tardir[1], tardir[2], Rwc, twc);
    float tar_yaw = atan2(taryawp_frombody[1]-tarp_frombody[1], taryawp_frombody[0]-tarp_frombody[0]);
    eulerToQ(0,0, tar_yaw, _q);
    //cout<<"taryaw in bodyframe is "<<tar_yaw<<endl;
    if(idp_targetNode > 0){
    	tarp_msg.orientation.w = _q[0];
    	tarp_msg.orientation.x = _q[1];
   		tarp_msg.orientation.y = _q[2];
    	tarp_msg.orientation.z = _q[3];
    	targetP_pub.publish(tarp_msg);
    }
    else{
    	tarp_msg.orientation.w = 0;
    	tarp_msg.orientation.x = 0;
    	tarp_msg.orientation.y = 0;
    	tarp_msg.orientation.z = 0;
    	// if(tarp_msg.position.x > 0.8){
    	// 	if(tarp_msg.position.x < 0.9)
    	// 	{
    	// 		tarp_msg.position.y = 0;
    	// 		tarp_msg.position.z = 0;
    	// 	}
    		targetP_pub.publish(tarp_msg);  //for autolabor
    	// }
    }
}

//for cal potantial
typedef pair<uint32_t, float> PAIR;
bool cmp_by_value(const PAIR& lhs, const PAIR& rhs) {
    return lhs.second < rhs.second;
}

void FindPathSRM::calDPotantial2(float _nowx, float _nowy, float _nowz,
                    float _tarx, float _tary, float _tarz,
                    float &vx, float &vy, float &vz)
{
    float _dx = _tarx - _nowx;
    float _dy = _tary - _nowy;
    float _dL = sqrt(pow(_dx,2) + pow(_dy,2));
    float vtar;
    if(_dL > 2)
    {
        vtar = 0.5;
    }
    else
    {
        vtar = _dL / 4;
    }
    float _vo = vtar / _dL;
    vx = _vo * _dx;
    vy = _vo * _dy;
    //_dz = _tarz - _nowz;
    if(_dx > 0)
    {
        vx = fabs(vx);
    }
    else
    {
        vx = -fabs(vx);
    }

    if(_dy > 0)
    {
        vy = fabs(vy);
    }
    else
    {
        vy = -fabs(vy);
    }

    float _dz = _tarz - _nowz;
    if(fabs(_dz) > 0.3)
    {
        vz = 0.3 * fabs(_dz) / _dz;
    }
    else
    {
        vz = _dz;
    }
}

void FindPathSRM::calYaw(float vx, float vy, float vz, float &yaw)
{
    float L = sqrt(pow(vx,2) + pow(vy,2));
    if(vy > 0)
    {
        yaw = acos(vx / L);
    }
    else
    {
        yaw = -acos(vx / L);
    }
}

float FindPathSRM::calYawSpeed(float yaw_tar, float yaw_now)
{
    float yErr = yaw_tar - yaw_now;
    if(yErr > 3.1415)
    {
        yErr -= 2*3.1415;
    }
    else if(yErr < -3.1415)
    {
        yErr += 2*3.1415;
    }

    return yErr;
}

bool FindPathSRM::reconstructGraph()
{
    clearall();
    cout << "Get start position(" << startP.x << ","
         << startP.y << "," << startP.z << "), end position("
         << endP.x << "," << endP.y << "," << endP.z << ")." << endl;
    if(readPoint() && readLink() && readMapPoint())
    {
        readParams();
        isAllFileExit = true;
        mapOct.initialize(mapPC->points, mapOctParams);
        return true;
    }
    else
    {
        isAllFileExit = false;
        return false;
    }
}

bool FindPathSRM::readPoint()
{
//    cout<< "_denKeyfPos"<<_denKeyfPos;
    std::ifstream in(_denKeyfPos);
    if(!in){
        printf("[findpath]no key position file!\n");
        return false;
    }
    std::string line;
    boost::char_separator<char> sep(" ");
    pcl::PointXYZRGB pt;
    //float minx=3000, miny=3000,maxx=-3000,maxy=-3000, minz=3000, maxz=-3000;
    PathNode pn(0,0,0,0);
    int i=0;
    // read point cloud from "freiburg format"
    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();
        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
        if (tokens.size() != 3) continue;
        pt.x = boost::lexical_cast<float>(tokens[0]);
        pt.y = boost::lexical_cast<float>(tokens[1]);
        pt.z = boost::lexical_cast<float>(tokens[2]);
        pt.r = 0;
        pt.g = 0;
        pt.b = 255;

        keyPosPC->push_back(pt);

        pn.setxyzid(pt.x, pt.y, pt.z, i);
        pNode.push_back(pn);
        ++i;
    }
    pNode.push_back(pn);
    pNode.push_back(pn);  //add two to the last for destination and start position

    std::cout << "read key pos----->" << std::endl
//              << "  x range (" << minx << "," << maxx << ")" << std::endl
//              << "  y range (" << miny << "," << maxy << ")" << std::endl
//              << "  z range (" << minz << "," << maxz << ")" << std::endl
              << "  total " << keyPosPC->size() << " key pos." << endl;
    in.close();

    return true;
}

bool FindPathSRM::readLink()
{
    std::ifstream in(_denKeyfPosRelation);
    if(!in){
        printf("[findpath]no link file!\n");
        return false;
    }
    std::string line;
    boost::char_separator<char> sep(" ");
    int i,j;
    vector<int> tmp(2);
    float distmp;

    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();
        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
        if (tokens.size() != 2) continue;
        i = boost::lexical_cast<float>(tokens[0]);
        j = boost::lexical_cast<float>(tokens[1]);
        tmp[0] = i;
        tmp[1] = j;
        keyPosLink.push_back(tmp);
        distmp = unibn::L2Distance<pcl::PointXYZRGB>::sqrt(
                    unibn::L2Distance<pcl::PointXYZRGB>::compute(
                        keyPosPC->points[i],keyPosPC->points[j]));
                /*sqrt(pow((keyPosPC->points[i].x-keyPosPC->points[j].x),2)
                    + pow((keyPosPC->points[i].y-keyPosPC->points[j].y),2)
                    + pow((keyPosPC->points[i].z-keyPosPC->points[j].z),2));*/

        pNode[i].addLink(j, distmp);
        pNode[j].addLink(i, distmp);
    }
    cout << "read link------>" << endl
         << "  total " << keyPosLink.size() << " link." << endl;
    in.close();

    return true;
}

bool FindPathSRM::readMapPoint()
{
    std::ifstream in(_MapPointsPos);
    if(!in){
        printf("[findpath]no mappoint file!\n");
        return false;
    }
    std::string line;
    boost::char_separator<char> sep(" ");
    pcl::PointXYZRGB pt;
    // read point cloud from "freiburg format"
    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();
        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
        if (tokens.size() != 3) continue;
        pt.x = boost::lexical_cast<float>(tokens[0]);
        pt.y = boost::lexical_cast<float>(tokens[1]);
        pt.z = boost::lexical_cast<float>(tokens[2]);
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;

        mapPC->push_back(pt);
    }

    std::cout << "read map points----->" << std::endl
              << "  total " << mapPC->size() << " map points." << endl;
    in.close();
    return true;
}

bool FindPathSRM::readParams()
{
    cv::FileStorage fs(_aParamForKeyframeHandle, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        cout << "can not find parameter file aParamForKeyframeHandle.yaml" << endl
             << "default parameters are used!" << endl;
        return false;
    }

    fs["mappointSparse"] >> mappointSparse;
    fs["astartdist"] >> astartdist;
    fs["bstartdist"] >> bstartdist;
    fs["aenddist"] >> aenddist;
    fs["benddist"] >> benddist;
    fs["aobsdist"] >> aobsdist;
    fs["bobsdist"] >> bobsdist;
    fs["loneseg"] >> loneseg;
    cout << "read parameters----->" << endl
         << "  mappointSparse:" << mappointSparse << endl
         << "  astartdist:" << astartdist << endl
         << "  bstartdist:" << bstartdist << endl
         << "  aenddist:" << aenddist << endl
         << "  benddist:" << benddist << endl
         << "  aobsdist:" << aobsdist << endl
         << "  bobsdist:" << bobsdist << endl
         << "  loneseg:" << loneseg << endl;
    return true;
}

void FindPathSRM::initPclViewer()
{
    viewer->initCameraParameters();
    viewer->createViewPort(0.0,0.0,1.0,1.0,v1);
    viewer->setBackgroundColor (255, 255, 255, v1);
    viewer->setCameraPosition(-2, 0, 15, 2, 0, 0, 0, 0, 1);
}

void FindPathSRM::setstring(string &str, int k)
{
    int t = 0, i = 0, size = str.size();
    char b = '0';
    while(k>0)
    {
        t = k%10;
        k = k/10;
        b = b + t;
        str.insert(str.end()-i-1, b);
        str.erase(size-i,1);
        b = b - t;
        ++i;
    }
}

template <typename Distance>
bool FindPathSRM::findStartEndNode()
{
    float mindistS = 10000, mindistE = 10000, tmpdist;
    for(int i=0; i<keyPosPC->size(); ++i)
    {
        tmpdist = Distance::compute(keyPosPC->points[i], startP);
        if(tmpdist<mindistS && !mapOct.isBlock<Distance >(startP, keyPosPC->points[i], mappointSparse))
        {
            id_startNode = i;
            mindistS = tmpdist;
        }

        tmpdist = Distance::compute(keyPosPC->points[i], endP);
        if(tmpdist<mindistE && !mapOct.isBlock<Distance >(endP, keyPosPC->points[i], mappointSparse))
        {
            id_endNode = i;
            mindistE = tmpdist;
        }
    }

    //cout << "start node id:" << id_startNode << ", end node id:" << id_endNode << endl;
    if(id_startNode<0 || id_endNode<0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

template <typename Distance>
bool FindPathSRM::astar()
{
    vector<int> nodeQueue;
    //first node
    pNode[id_startNode].id_fromWhere = id_startNode;
    pNode[id_startNode].distTilNow = 0;
    pNode[id_startNode].distToEnd = Distance::sqrt(
                Distance::compute(
                    keyPosPC->points[id_startNode], keyPosPC->points[id_endNode]));
    pNode[id_startNode].distTotal = pNode[id_startNode].distTilNow +
            pNode[id_startNode].distToEnd;
    pNode[id_startNode].isDealed = true;
    nodeQueue.push_back(id_startNode);

    int id_now, id_next, dealtime=0;
    float dtn_tmp;
    bool valid_path = false;
    while(!nodeQueue.empty())
    {
        if((++dealtime)>2000)
        {
            cout << "Path not found" << endl;
            break;
        }

        id_now = nodeQueue[nodeQueue.size()-1];

        if(id_now == id_endNode)
        {
//            while(id_now != id_startNode)
//            {
//                path.push_back(id_now);
//                id_now = pNode[id_now].id_fromWhere;
//            }
//            path.push_back(id_startNode);
//            cout << "Find path done!" << endl;
            valid_path = true;
            break;
        }
        nodeQueue.pop_back();

        for(int i=0; i<pNode[id_now].id_link.size(); ++i)
        {
            id_next = pNode[id_now].id_link[i];

            if(id_next == pNode[id_now].id_fromWhere) continue;

            if(!pNode[id_next].isDealed)
            {
                pNode[id_next].distTilNow = pNode[id_now].distTotal
                        + pNode[id_now].dist_link[i];
                pNode[id_next].id_fromWhere = id_now;
                pNode[id_next].distToEnd = 0;/*Distance::sqrt(
                        Distance::compute(
                            keyPosPC->points[id_startNode], keyPosPC->points[id_endNode]));*/
                pNode[id_next].distTotal = pNode[id_next].distToEnd
                        + pNode[id_next].distTilNow;
                pNode[id_next].isDealed = true;

                insertSortByDistTotal(id_next, nodeQueue);
            }
            else
            {
                dtn_tmp = pNode[id_now].distTotal + pNode[id_now].dist_link[i];
                if(dtn_tmp < pNode[id_next].distTilNow)
                {
                    pNode[id_next].distTilNow = dtn_tmp;
                    pNode[id_next].id_fromWhere = id_now;
                    pNode[id_next].distTotal = pNode[id_next].distToEnd
                            + pNode[id_next].distTilNow;

                    int j=0;
                    //erase from queue and readd into queue
                    for(j=0; j<nodeQueue.size(); ++j)
                    {
                        if(nodeQueue[j] == id_next)
                        {
                            nodeQueue.erase(nodeQueue.begin()+j);
                            insertSortByDistTotal(id_next, nodeQueue);
                            break;
                        }
                    }
                }
            }
        }
    }
    mylog << getShortestPathLength() << " ";
    if(!valid_path){
        cout<<"no valid path. maybe topomap is broken."<<endl;
        return valid_path;
    }
    //find all nodes
    pcl::PointXYZ tmpP;
    path.push_back(pNode.size()-1);  //push destination first
    tmpP.x = pNode[pNode.size()-1].x;
    tmpP.y = pNode[pNode.size()-1].y;
    tmpP.z = pNode[pNode.size()-1].z;
    path_point.push_back(tmpP);
    id_now = id_endNode;
    while(id_now != id_startNode)
    {
        path.push_back(id_now);
        tmpP.x = pNode[id_now].x;
        tmpP.y = pNode[id_now].y;
        tmpP.z = pNode[id_now].z;
        path_point.push_back(tmpP);
        id_now = pNode[id_now].id_fromWhere;
    }
    path.push_back(id_startNode);
    path.push_back(pNode.size()-2);  //push start point

    tmpP.x = pNode[id_startNode].x;
    tmpP.y = pNode[id_startNode].y;
    tmpP.z = pNode[id_startNode].z;
    path_point.push_back(tmpP);
    tmpP.x = pNode[pNode.size()-2].x;
    tmpP.y = pNode[pNode.size()-2].y;
    tmpP.z = pNode[pNode.size()-2].z;
    path_point.push_back(tmpP);

    //remove the too near node from start/end point
    pcl::PointXYZRGB startorend, nearone;
    startorend.x = path_point[0].x;
    startorend.y = path_point[0].y;
    startorend.z = path_point[0].z;
    nearone.x = path_point[2].x;
    nearone.y = path_point[2].y;
    nearone.z = path_point[2].z;
    if(path_point.size()>=3 && !mapOct.isBlock<Distance >(startorend, nearone, 1.2*mappointSparse)){
        path_point.erase(path_point.begin()+1);
        path.erase(path.begin()+1);
    }
    startorend.x = path_point[path_point.size()-1].x;
    startorend.y = path_point[path_point.size()-1].y;
    startorend.z = path_point[path_point.size()-1].z;
    nearone.x = path_point[path_point.size()-3].x;
    nearone.y = path_point[path_point.size()-3].y;
    nearone.z = path_point[path_point.size()-3].z;
    if(path_point.size()>=3 && !mapOct.isBlock<Distance >(startorend, nearone, 1.2*mappointSparse)){
        path_point.erase(path_point.begin()+path_point.size()-2);
        path.erase(path.begin()+path.size()-2);
//        nearone.x = path_point[path_point.size()-3].x;
//        nearone.y = path_point[path_point.size()-3].y;
//        nearone.z = path_point[path_point.size()-3].z;
    }

    idp_targetNode = path.size() - 1;
    pathid_near_node = path.size() - 1; //which path node drone is near
    // cout << "Find path done!" << endl;
    // for(int i=0;i<path.size();++i){
    //     cout<<path[i]<<"("<<path_point[i].x<<","<<path_point[i].y<<","<<path_point[i].z<<")"<<"<<<";
    // }
    // cout<<endl;
    justReplan = 500;
    return valid_path;
}

//insert and sort path node in queue by distTotal, large at begin, small at end
void FindPathSRM::insertSortByDistTotal(int _id, vector<int> &_nodeQueue)
{
    if(_nodeQueue.empty())
    {
        _nodeQueue.push_back(_id);
    }
    else if(pNode[_id].distTotal > pNode[_nodeQueue[0]].distTotal)
    {
        _nodeQueue.insert(_nodeQueue.begin(), _id);
    }
    else if(pNode[_id].distTotal < pNode[_nodeQueue[_nodeQueue.size()-1]].distTotal)
    {
        _nodeQueue.push_back(_id);
    }
    else
    {
        int _start=0, _end=_nodeQueue.size()-1, half=0;
        while((_end-_start)>1)
        {
            half = (_end+_start) / 2;
            if(pNode[_id].distTotal > pNode[_nodeQueue[half]].distTotal)
            {
                _end = half;
            }
            else
            {
                _start = half;
            }
        }
        _nodeQueue.insert(_nodeQueue.begin()+_end, _id);
    }
}

void FindPathSRM::testShowNode()
{
    for(int i=0; i<pNode.size(); ++i)
    {
        cout << "~~node number:" << i << endl << " links:";
        for(int j=0; j<pNode[i].id_link.size(); ++j)
        {
            cout << pNode[i].id_link[j] << " ";
        }
        cout << endl;
    }
}

void FindPathSRM::testShowQueue(vector<int> &vec)
{
    cout << "queue:";
    for(int i=0; i<vec.size(); ++i)
    {
        cout << vec[i] << " ";
    }
    cout << endl;
    cout << "distTotal:";
    for(int i=0; i<vec.size(); ++i)
    {
        cout << pNode[vec[i]].distTotal <<" ";
    }
    cout <<endl;
}

void FindPathSRM::restartNavCallback(const geometry_msgs::Pose::ConstPtr msg){
    cout<<"[fp]get restart message"<<endl;
    if(id_endNode < 0){
        return ;
    }
    //if(justReplan > 0)
    //    return;
    if(pathid_near_node<1){
        //res.set_ok = false;
        ROS_INFO("[FINDPATH]near destinaton and not replaning.");
        return ;//false;
    }
    //remenber to change from body frame to world frame
    if(msg->orientation.w < 0){
        //no safezone
        mylog << ros::Time::now().toSec() <<endl<<
                 ">>>>>>>>>replan"<<endl;
    }
    else if(msg->orientation.x > 0){
        //(vel modify and targetP is blocked) && msg->orientation.y > 0
        // turn into wrong path so replan
        // not finished
        float T[3]{nowfviP.x, nowfviP.y, nowfviP.z};
        float modifytar[3];
        transform_NWUworld_from_body(msg->position.x, msg->position.y, msg->position.z,
                                     modifytar[0], modifytar[1], modifytar[2], Rwc, T);
        //cal angle of origin_target-posnow-modify target, if angle>80degree, the origin
        //road might be blocked
        float vec1[3] = {modifytar[0]-nowfviP.x, modifytar[1]-nowfviP.y, modifytar[2]-nowfviP.z},
              vec2[3] = {pNode[path[idp_targetNode]].x-nowfviP.x, pNode[path[idp_targetNode]].y-nowfviP.y, pNode[path[idp_targetNode]].z-nowfviP.z};
        float cos_angle = (vec1[0]*vec2[0]+vec1[1]*vec2[1]+vec1[2]*vec2[2])
                / (sqrt(vec1[0]*vec1[0]+vec1[1]*vec1[1]+vec1[2]*vec1[2])*sqrt(vec2[0]*vec2[0]+vec2[1]*vec2[1]+vec2[2]*vec2[2]));
        if(cos_angle < 0.17){}
        else{
            return;
        }
    }
    else{
        return;
    }
    /**/
    ROS_INFO("[FINDPATH]start replan.");
    int removenode1 = path[pathid_near_node], removenode2 = path[pathid_near_node-1];
    float dist = sqrt(pow((pNode[removenode1].x - pNode[removenode2].x),2)
                     +pow((pNode[removenode1].y - pNode[removenode2].y),2)
                     +pow((pNode[removenode1].z - pNode[removenode2].z),2));
    pNode[path[pathid_near_node]].removeLink(path[pathid_near_node-1]);
    cout<<"near path node is:"<<pathid_near_node<<","<<pNode[path[pathid_near_node]].x<<","<<pNode[path[pathid_near_node]].y<<","<<pNode[path[pathid_near_node]].z<<endl;
    pNode[path[pathid_near_node-1]].removeLink(path[pathid_near_node]);
    int tmpidendnode = id_endNode;
    int tmppathidnear = path[pathid_near_node];
    resetAll(nowP.x, nowP.y, nowP.z, endP.x, endP.y, endP.z);
    id_endNode = tmpidendnode;
    id_startNode = tmppathidnear;

    if(astar<unibn::L2Distance<pcl::PointXYZRGB> >()){
        ROS_INFO("[FINDPATH]replan ok.");
    }
    else{
        ROS_INFO("[FINDPATH]replan fail.");
    }

    //add removed node back?
    pNode[removenode1].addLink(removenode2, dist);
    pNode[removenode2].addLink(removenode1, dist);
    
    //res.set_ok = true;
    //return true;
}

void FindPathSRM::reloadCB(const std_msgs::StringConstPtr& msg){
    // _denKeyfPos = msg->data+"node.txt";
    // _denKeyfPosRelation = msg->data+"edge.txt";
    _denKeyfPos = msg->data+"denKeyfPos.txt";
    _denKeyfPosRelation = msg->data+"denKeyfPosRelation.txt";
    _MapPointsPos = msg->data+"MapPointsPos.txt";
    cout << "loading new topomap from " << msg->data << endl;
    if(reconstructGraph()){
        ROS_INFO("[FINDPATH]reload map done.");
    }
    else{
        ROS_WARN("[FINDPATH]unknown error while reloading map.");
    }
}

void FindPathSRM::reloadGraph(string filepath)
{
    _denKeyfPos = filepath+"denKeyfPos.txt";
    _denKeyfPosRelation = filepath+"denKeyfPosRelation.txt";
    _MapPointsPos = filepath+"MapPointsPos.txt";
    cout << "loading new topomap from " << filepath << endl;
    if(reconstructGraph()){
        ROS_INFO("[FINDPATH]reload map done.");
    }
    else{
        ROS_WARN("[FINDPATH]unknown error while reloading map.");
    }
}

void FindPathSRM::clearall(){
    resetAll(startP.x, startP.y, startP.z, endP.x, endP.y, endP.z);
    keyPosPC->clear();
    keyPosLink.clear();
    mapPC->clear();
    pNode.clear();
//    isDealView = false;
}
