
#include "AIBrain.h"

AIBrain::AIBrain(ros::NodeHandle *_nh):
    nh(_nh), isControllerInput(false), pidTime(ros::Time::now().toSec()),
    isStableHover(true), poseCtrl(_nh),
    isTestModeOfFindPath(false),
    yawworldslam(0), ptop(*_nh), poseXYZ_target{0,0,0},
    fpsrm(*_nh, true), state_findpath(0), state_ptop(0)
{
    poseXYZ_destination[0] = 0;
    poseXYZ_destination[1] = 0;
    poseXYZ_destination[2] = 2;
    //init param for zone1
    Tworldslam[0] = 0;//4.06;
    Tworldslam[1] = 0;//2.95;
    Tworldslam[2] = 0;//2.73;
    float _q[4] = {1,0,0,0};//{0.766,0,0,0.643};
    qToRotation(_q, Rworldslam);
    qToEuler(yawworldslam, yawworldslam, yawworldslam, _q);
//    Rworldslam[0] = 1;
//    Rworldslam[4] = 1;
//    Rworldslam[8] = 1;
    initSubscriber();
    initService();
}

AIBrain::~AIBrain()
{}

void AIBrain::think()
{
    double dt = ros::Time::now().toSec() - pidTime.toSec();
    pidTime = ros::Time::now();
//    if(isControllerInput)
//    {
//        controllerTASK(dt);
//    }
//    else
    {
        givePosToFindpath();
        fpsrm.display();
        poseCtrl.setState(poseXYZ_now, velXYZ_now, yaw_now);
        switch(task_number)
        {
        case HOVER_TASK:hoverTASK(dt);break;
        case TAKEOFF_TASK:takeoffTASK();break;
        case LAND_TASK:landTASK();break;
        case MOVE_TASK:controllerTASK(dt);break;
        case FINDPATH_TASK:findpath(dt);break;
        case POINTTOPOINT_TASK:pointToPointTASK(dt);break;
        case FAR_POINTTOPOINT_TASK:farPointToPointTASK(dt);break;
        default:hoverTASK(dt);break;
        }
    }
}

void AIBrain::hoverTASK(double _dt)
{
    if(task_lastnumber != task_number)
    {
        ROS_INFO("[AIBRAIN]hover!");
        task_lastnumber = task_number;
        hoverStartTime = ros::Time::now().toSec();
    }

    if(!isStableHover)
    {
        if(sqrt((pow(velXYZ_now[0],2) + pow(velXYZ_now[1],2)) < 0.3))
        {
            isStableHover = true;
            poseXYZ_target[0] = poseXYZ_now[0];
            poseXYZ_target[1] = poseXYZ_now[1];
            poseXYZ_target[2] = poseXYZ_now[2];
            yaw_target = yaw_now;
            poseCtrl.doMove(_dt,0,0,0,0);
        }
        else
        {
            poseCtrl.brake(_dt);
//            poseXYZ_target[0] = poseXYZ_now[0] - velXYZ_now[0]*_dt;
//            poseXYZ_target[1] = poseXYZ_now[1] - velXYZ_now[1]*_dt;
//            poseXYZ_target[2] = poseXYZ_now[2] - velXYZ_now[2]*_dt;
        }
    }
    else
    {
        poseCtrl.doHover(_dt, poseXYZ_target, yaw_target, poseXYZ_now, velXYZ_now, yaw_now);
    }

    if(isTestModeOfFindPath){
        findpath(_dt);
    }

    //display position in slam
    float posSlam[3];
    transform_body_from_NWUworld(posSlam[0], posSlam[1], posSlam[2],
            poseXYZ_target[0], poseXYZ_target[1], poseXYZ_target[2], Rworldslam, Tworldslam);
    printf("target pos(%f,%f,%f)(%f,%f,%f)\r", poseXYZ_target[0], poseXYZ_target[1], poseXYZ_target[2],posSlam[0], posSlam[1], posSlam[2]);
}

void AIBrain::takeoffTASK()
{
    if(task_lastnumber != task_number)
    {
        ROS_INFO("[AIBRAIN]takeoff!");
        task_lastnumber = task_number;
    }
    yaw_target = yaw_now;
    poseXYZ_target[0] = poseXYZ_now[0];
    poseXYZ_target[1] = poseXYZ_now[1];
    poseXYZ_target[2] = 1.0;
    isStableHover = true;
    poseCtrl.takeOff();
    task_number = HOVER_TASK;
}

void AIBrain::landTASK()
{
    if(task_lastnumber != task_number)
    {
        ROS_INFO("[AIBRAIN]land!");
        task_lastnumber = task_number;
    }
    poseCtrl.land();
    task_number = HOVER_TASK;
}

void AIBrain::controllerTASK(double _dt)
{
    if(task_lastnumber != task_number)
    {
        ROS_INFO("[AIBRAIN]move!");
        task_lastnumber = task_number;
    }
    poseCtrl.doMove(_dt, forbackward_ctrl, leftright_ctrl, updown_ctrl, turnlr_ctrl);
    if(isTestModeOfFindPath){
        findpath(_dt);
    }
    if(fabs(forbackward_ctrl)+fabs(leftright_ctrl)+fabs(updown_ctrl)+fabs(turnlr_ctrl) < 0.01){
        task_number = HOVER_TASK;
    }
}

void AIBrain::pointToPointTASK(double _dt)
{
    if(task_lastnumber != task_number)
    {
        ROS_INFO("[AIBRAIN]start point to point!");
        //state_ptop = 0;
        task_lastnumber = task_number;
    }

    if(state_ptop == 1){
        float _v[4], _T[3] = {0,0,0};
        float posSlam[3];
        transform_body_from_NWUworld(posSlam[0], posSlam[1], posSlam[2],
            poseXYZ_now[0], poseXYZ_now[1], poseXYZ_now[2], Rworldslam, Tworldslam);
        if(ptop.getTargetSpeed(posSlam[0], posSlam[1], posSlam[2], yaw_now-yawworldslam,
                               _v[0], _v[1], _v[2], _v[3])){
            transform_NWUworld_from_body(_v[0], _v[1], _v[2],
                    posSlam[0], posSlam[1], posSlam[2], Rworldslam, _T);
            //poseCtrl.doMoveInWorld(_dt, posSlam[0], posSlam[1], posSlam[2], _v[3]);
        }
        else{
            state_ptop = 2;
            hoverTASK(_dt);
        }
    }
    else if(state_ptop == 0){
        if(!ptop.readWayPoints()){
            state_ptop = 2;
        }
        else
        {
            state_ptop = 1;
        }
        hoverTASK(_dt);
    }
    else if(state_ptop == 2){
        state_ptop = 0;
        task_number = HOVER_TASK;
        ROS_INFO("[AIBRAIN]to destination!");
        hoverTASK(_dt);
    }
}

void AIBrain::farPointToPointTASK(double _dt){
    if(task_lastnumber != task_number)
    {
        ROS_INFO("[AIBRAIN]start far point to point!");
        task_lastnumber = task_number;
        std_msgs::Int16 msg;
        msg.data = 1;
        //finish_pub.publish(msg);
    }
    if(cntdown > 0){
        --cntdown;
        printf("\rwait count down %.0f second.", 1.0*cntdown/50);
        hoverTASK(_dt);
        return;
    }
    if(state_farptop == 1){
        if(state_findpath == 3){
            //next point
            ++point_i_farptop;
            if(point_i_farptop >= farptop_path.poses.size()){
                state_farptop = 2;
                return;
            }
            if(point_i_farptop != 0){
                cntdown = pausetime;
            }
            isStableHover = false;
            ROS_INFO("[FARPTOP]to the %d point and pause %f second.",point_i_farptop, pausetime*1.0/50);
            transform_NWUworld_from_body(farptop_path.poses[point_i_farptop].pose.position.x,
                    farptop_path.poses[point_i_farptop].pose.position.y,
                    farptop_path.poses[point_i_farptop].pose.position.z,
                    poseXYZ_destination[0], poseXYZ_destination[1], poseXYZ_destination[2],
                    Rworldslam, Tworldslam);
            state_findpath = 0;
        }
        findpath(_dt);
    }
    else if(state_farptop == 0){
        point_i_farptop = -1;
        state_findpath = 3;
        state_farptop = 1;
    }
    else if(state_farptop == 2){
        state_farptop = 0;
        farptop_path.poses.clear();
        task_number = HOVER_TASK;
        isStableHover = false;
        std_msgs::Int16 msg;
        msg.data = 0;
        //finish_pub.publish(msg);
        ROS_INFO("[AIBRAIN]far point to point task done!");
        hoverTASK(_dt);
    }
}

void AIBrain::findpath(double _dt)
{
    if(isTestModeOfFindPath && state_findpath == 0){
        ROS_INFO("[AIBRAIN]start find path in test mode, please control with joystick!");
    }
    else if(task_lastnumber != task_number)
    {
        ROS_INFO("[AIBRAIN]start find path!");
        state_findpath = 0;
        task_lastnumber = task_number;
    }

    float posSlam[3], posDesSlam[3];
    transform_body_from_NWUworld(posSlam[0], posSlam[1], posSlam[2],
            poseXYZ_now[0], poseXYZ_now[1], poseXYZ_now[2], Rworldslam, Tworldslam);
    transform_body_from_NWUworld(posDesSlam[0], posDesSlam[1], posDesSlam[2],
            poseXYZ_destination[0], poseXYZ_destination[1], poseXYZ_destination[2], Rworldslam, Tworldslam);

    if(state_findpath == 0)  //not started yet
    {
        fpsrm.resetAll(posSlam[0], posSlam[1], posSlam[2],
                posDesSlam[0], posDesSlam[1], posDesSlam[2]);
        ROS_INFO("[FINDPATH]start findpath.");
        if(fpsrm.findPath())
        {
            state_findpath = 2;
            ROS_INFO("[FINDPATH]findpath done.");
            //fpsrm.display();
        }
        else
            state_findpath = 3;

        //cout << "[brain]find path done" <<endl;
        if(!isTestModeOfFindPath){
            hoverTASK(_dt);  //to avoid no control input to drone
        }
    }
    else if(state_findpath == 1)  //find path done
    {
        if(!isTestModeOfFindPath){
            hoverTASK(_dt);
        }
    }
    else if(state_findpath == 2)   //on the move
    {        
        //fpsrm.display();
        float _v[4];
        float _T[3] = {0,0,0};
        transform_body_from_NWUworld(_v[0], _v[1], _v[2],
                velXYZ_now[0], velXYZ_now[1], velXYZ_now[2], Rworldslam, _T);
        //vx,vy,vz,vyaw, if at destination, targetx, targety, targetz, targetyaw
//        Eigen::Matrix<float, 3, 3> Rslamworld, Rbodyworld, Rbodyslam;
//        Rslamworld << Rworldslam[0] << Rworldslam[1] << Rworldslam[2]
//                << Rworldslam[3] << Rworldslam[4] << Rworldslam[5]
//                << Rworldslam[6] << Rworldslam[7] << Rworldslam[8];
//        float Rbw_tmp[9];
//        qToRotation(qwxyz_now, Rbw_tmp);
//        Rbodyslam << Rbw_tmp[0] << Rbw_tmp[1] << Rbw_tmp[2]
//                << Rbw_tmp[3] << Rbw_tmp[4] << Rbw_tmp[5]
//                << Rbw_tmp[6] << Rbw_tmp[7] << Rbw_tmp[8];
//        Rbodyslam = Rbodyworld * Rslamworld.inverse();
        //the yaw may nolonger be used
        if(fpsrm.getTargetSpeed(posSlam[0], posSlam[1], posSlam[2], yaw_now-yawworldslam,
                             _v[0], _v[1], _v[2], _v[3]))
        {
            //cout << "[brain]running along path" <<endl;

            //speed just need rotate
            transform_NWUworld_from_body(_v[0], _v[1], _v[2],
                    posSlam[0], posSlam[1], posSlam[2], Rworldslam, _T);
//            cout << "get target speed:" << posSlam[0] << "," << posSlam[1] << ","
//                 << posSlam[2] << "," << _v[3] <<endl;
//            if(!isTestModeOfFindPath){
//                poseCtrl.doMoveInWorld(_dt, posSlam[0], posSlam[1], posSlam[2], _v[3]);
//            }
        }
        else
        {
            //cout << "[brain]to end" <<endl;
            state_findpath = 3;
            transform_NWUworld_from_body(_v[0], _v[1], _v[2],
                    posSlam[0], posSlam[1], posSlam[2], Rworldslam, Tworldslam);
            poseXYZ_target[0] = posSlam[0];
            poseXYZ_target[1] = posSlam[1];
            poseXYZ_target[2] = posSlam[2];
            yaw_target = _v[3] + yawworldslam;
            //task_number = HOVER_TASK;
            // if(!isTestModeOfFindPath){
            //     hoverTASK(_dt);
            // }
        }
    }
    else if(state_findpath == 3)  //to the end
    {
        state_findpath = 0;
        isTestModeOfFindPath = false;
        task_number = HOVER_TASK;
        isStableHover = false;
        ROS_INFO("[AIBRAIN]to destination!");
        // hoverTASK(_dt);
    }
}

void AIBrain::initSubscriber()
{
    /*
    btn1_sub = nh->subscribe("/remotecontroller/btn1", 1, &AIBrain::btn1Callback, this);
    btn3_sub = nh->subscribe("/remotecontroller/btn3", 1, &AIBrain::btn3Callback, this);
    axes0lr_sub = nh->subscribe("/remotecontroller/axes0lr", 1, &AIBrain::axes0lrCallback, this);
    axes0ud_sub = nh->subscribe("/remotecontroller/axes0ud", 1, &AIBrain::axes0udCallback, this);
    axes1lr_sub = nh->subscribe("/remotecontroller/axes1lr", 1, &AIBrain::axes1lrCallback, this);
    axes1ud_sub = nh->subscribe("/remotecontroller/axes1ud", 1, &AIBrain::axes1udCallback, this);
    btn2_sub = nh->subscribe("/remotecontroller/btn2", 1, &AIBrain::btn2Callback, this);
    btn4_sub = nh->subscribe("/remotecontroller/btn4", 1, &AIBrain::btn4Callback, this);
    btn5_sub = nh->subscribe("/remotecontroller/btn5", 1, &AIBrain::btn5Callback, this);
    */
    joy_sub = nh->subscribe("/joy", 1, &AIBrain::joyCallback, this);
    pose_sub = nh->subscribe("/est_pose", 1, &AIBrain::poseCallback, this);
    vel_sub = nh->subscribe("/est_vel", 1, &AIBrain::velCallback, this);
    qtSlamWorld = nh->subscribe("/qtworldslam", 1, &AIBrain::qtslamworldCallback, this);
    move_base_goal_sub = nh->subscribe("/move_base_simple/goal", 1, &AIBrain::movebasegoalCallback, this);
    ptop_sub = nh->subscribe("/point_to_point/path", 1, &AIBrain::ptopCallback, this);
    farptop_sub = nh->subscribe("/far_point_to_point/path", 1, &AIBrain::farptopCallback, this);
}

void AIBrain::initService()
{
    setDestination_ser = nh->advertiseService("/ai_robot_navigation/setdestination", &AIBrain::setdestinationCallback, this);
}

void AIBrain::givePosToFindpath()
{
    float posSlam[3];
    transform_body_from_NWUworld(posSlam[0], posSlam[1], posSlam[2],
            poseXYZ_now[0], poseXYZ_now[1], poseXYZ_now[2], Rworldslam, Tworldslam);
    float R_bodyworld[9];
    qToRotation(qwxyz_now, R_bodyworld);
    Eigen::Matrix<float, 3, 3> Rslamworld, Rbodyworld, Rbodyslam;
    Rbodyworld << R_bodyworld[0], R_bodyworld[1], R_bodyworld[2],
            R_bodyworld[3], R_bodyworld[4], R_bodyworld[5],
            R_bodyworld[6], R_bodyworld[7], R_bodyworld[8];
    Rslamworld << Rworldslam[0], Rworldslam[1], Rworldslam[2],
            Rworldslam[3], Rworldslam[4], Rworldslam[5],
            Rworldslam[6], Rworldslam[7], Rworldslam[8];
    Rbodyslam = Rslamworld.inverse() * Rbodyworld;
    float R_bodyslam[9]={Rbodyslam(0,0),Rbodyslam(0,1),Rbodyslam(0,2),
                        Rbodyslam(1,0),Rbodyslam(1,1),Rbodyslam(1,2),
                        Rbodyslam(2,0),Rbodyslam(2,1),Rbodyslam(2,2)}, qbs[4], _r, _p, _y;
    //cout <<"Rbodyslam:"<< R_bodyslam[0] <<","<< R_bodyslam[1] <<","<< R_bodyslam[2] <<endl
    //       << R_bodyslam[3] <<","<< R_bodyslam[4] <<","<< R_bodyslam[5] <<endl
    //          << R_bodyslam[6] <<","<< R_bodyslam[7] <<","<< R_bodyslam[8] <<endl;
    rotationToQ(qbs, R_bodyslam);
    //problem with qbs
    qToEuler(_r, _p, _y, qbs);
    //cout << "qbs:"<<qbs[0]<<","<<qbs[1]<<","<<qbs[2]<<","<<qbs[3]<<endl;
    //cout << "rpy:"<<_r<<","<<_p<<","<<_y<<endl;
    fpsrm.setPosForVisual(posSlam[0], posSlam[1], posSlam[2], _y);
}

void AIBrain::qtslamworldCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg)
{
    Tworldslam[0] = float(_msg->pose.position.x);
    Tworldslam[1] = float(_msg->pose.position.y);
    Tworldslam[2] = float(_msg->pose.position.z);
    float _q[4] = {float(_msg->pose.orientation.w),
                   float(_msg->pose.orientation.x),
                   float(_msg->pose.orientation.y),
                   float(_msg->pose.orientation.z)};
    qToRotation(_q, Rworldslam);
    float _ro, _pi;
    qToEuler(_ro, _pi, yawworldslam, _q);
    ROS_INFO("[AIBRAIN]Got Rotation and Transformation of slam and world!");
    cout << "{" << Rworldslam[0] <<", "<< Rworldslam[1] <<", "<< Rworldslam[2]<<endl
            << Rworldslam[3] <<", "<< Rworldslam[4] <<", "<< Rworldslam[5]<<endl
            << Rworldslam[6] <<", "<< Rworldslam[7] <<", "<< Rworldslam[8]<<"}"<<endl;
}

void AIBrain::movebasegoalCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg)
{
    ROS_INFO("[AIBRAIN]set destination");
    if(_msg->header.frame_id == "map"){
        transform_NWUworld_from_body(_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z, poseXYZ_destination[0],
                poseXYZ_destination[1], poseXYZ_destination[2], Rworldslam, Tworldslam);
        float posSlam[3];
        transform_body_from_NWUworld(posSlam[0], posSlam[1], posSlam[2],
                poseXYZ_now[0], poseXYZ_now[1], poseXYZ_now[2], Rworldslam, Tworldslam);
        fpsrm.resetAll(posSlam[0], posSlam[1], posSlam[2],
                _msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z);
        ROS_INFO("[AIBRAIN]get (%f,%f,%f) in slam coordinate and set destination to (%f,%f,%f) in world.\n",
               _msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z,
               poseXYZ_destination[0], poseXYZ_destination[1], poseXYZ_destination[2]);
        task_number = FINDPATH_TASK;
        state_findpath = 0;
    }
    else if(_msg->header.frame_id == "base_link"){
        float posSlam[3], Rworldbody[9], tarslam[3];
        //get self pose in slam
        transform_body_from_NWUworld(posSlam[0], posSlam[1], posSlam[2],
                poseXYZ_now[0], poseXYZ_now[1], poseXYZ_now[2], Rworldslam, Tworldslam);
        qToRotation(qwxyz_now, Rworldbody);
        //relative target from body to world
        transform_NWUworld_from_body(_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z,
                                     poseXYZ_destination[0], poseXYZ_destination[1], poseXYZ_destination[2], Rworldbody, poseXYZ_now);
        //target from world to slam
        transform_body_from_NWUworld(tarslam[0],tarslam[1],tarslam[2],
                 poseXYZ_destination[0], poseXYZ_destination[1], poseXYZ_destination[2], Rworldslam, Tworldslam);
        //set target
        fpsrm.resetAll(posSlam[0], posSlam[1], posSlam[2],
                tarslam[0],tarslam[1],tarslam[2]);
        ROS_INFO("[AIBRAIN]get (%f,%f,%f) in body coordinate and set destination to (%f,%f,%f) in world, to (%f,%f,%f) in slam.\n",
                 _msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z,
                 poseXYZ_destination[0], poseXYZ_destination[1], poseXYZ_destination[2],
                 tarslam[0],tarslam[1],tarslam[2]);
        task_number = FINDPATH_TASK;
        state_findpath = 0;
    }
}

void AIBrain::ptopCallback(const nav_msgs::Path::ConstPtr &_msg)
{
    ptop.clear();
    for(int i=0; i<_msg->poses.size(); ++i){
        ptop.addpoint(_msg->poses[i].pose.position.x,
                      _msg->poses[i].pose.position.y,
                      _msg->poses[i].pose.position.z);
    }
    ROS_INFO("[AIBRAIN]set point to point ok.");
    task_number = POINTTOPOINT_TASK;
    state_ptop = 1;
}

void AIBrain::farptopCallback(const nav_msgs::Path::ConstPtr &_msg){
    farptop_path = *_msg;
    cntdown = 0;
    for(int i=0; i<farptop_path.poses.size(); ++i){
        printf("waypoint %d (%f,%f,%f)\n", i, farptop_path.poses[i].pose.position.x, farptop_path.poses[i].pose.position.y, farptop_path.poses[i].pose.position.z);
    }
    ROS_INFO("[AIBRAIN]set far point to point ok.");
    task_number = FAR_POINTTOPOINT_TASK;
    state_farptop = 0;
}

void AIBrain::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg)
{
    poseXYZ_now[0] = _msg->pose.position.x;
    poseXYZ_now[1] = _msg->pose.position.y;
    poseXYZ_now[2] = _msg->pose.position.z;
    qwxyz_now[0] = _msg->pose.orientation.w;
    qwxyz_now[1] = _msg->pose.orientation.x;
    qwxyz_now[2] = _msg->pose.orientation.y;
    qwxyz_now[3] = _msg->pose.orientation.z;
    qToEuler(roll_now, pitch_now, yaw_now, qwxyz_now);
}

void AIBrain::velCallback(const geometry_msgs::TwistStamped::ConstPtr &_msg)
{
    velXYZ_now[0] = _msg->twist.linear.x;
    velXYZ_now[1] = _msg->twist.linear.y;
    velXYZ_now[2] = _msg->twist.linear.z;
}

void AIBrain::btn1Callback(const std_msgs::Bool::ConstPtr &_msg)
{
    if(_msg->data)
    {
        task_number = TAKEOFF_TASK;
        isStableHover = false;
        if(isTestModeOfFindPath){
            isTestModeOfFindPath = false;
            ROS_INFO("[AIBRAIN]exit test mode of find path!");
        }
    }
}

void AIBrain::btn3Callback(const std_msgs::Bool::ConstPtr &_msg)
{
    if(_msg->data)
    {
        task_number = LAND_TASK;
        isStableHover = false;
        if(isTestModeOfFindPath){
            isTestModeOfFindPath = false;
            ROS_INFO("[AIBRAIN]exit test mode of find path!");
        }
    }
}

void AIBrain::btn2Callback(const std_msgs::Bool::ConstPtr &_msg)
{
    if(_msg->data)
    {
        task_number = FINDPATH_TASK;
        isStableHover = false;
    }
}

void AIBrain::btn4Callback(const std_msgs::Bool::ConstPtr &_msg)
{
    if(_msg->data)
    {
        task_number = POINTTOPOINT_TASK;
        isStableHover = false;
    }
}

void AIBrain::btn5Callback(const std_msgs::Bool::ConstPtr &_msg)
{
    if(_msg->data && !isTestModeOfFindPath)
    {
        isTestModeOfFindPath = true;
        state_findpath = 0;
        ROS_INFO("[AIBRAIN]enter test mode of find path!");
    }
}

void AIBrain::axes0lrCallback(const std_msgs::Float64::ConstPtr &_msg)
{
    leftright_ctrl = _msg->data;
    if(fabs(leftright_ctrl)>0.01)
    {
        task_number = MOVE_TASK;  //for safety
        isControllerInput = true;
        isStableHover = false;
    }    
}

void AIBrain::axes0udCallback(const std_msgs::Float64::ConstPtr &_msg)
{
    forbackward_ctrl = _msg->data;
    if(fabs(forbackward_ctrl)>0.01)
    {
        task_number = MOVE_TASK;  //for safety
        isControllerInput = true;
        isStableHover = false;
    }
}

void AIBrain::axes1udCallback(const std_msgs::Float64::ConstPtr &_msg)
{
    updown_ctrl = _msg->data;
    if(fabs(updown_ctrl)>0.01)
    {
        task_number = MOVE_TASK;  //for safety
        isControllerInput = true;
        isStableHover = false;
    }
}

void AIBrain::axes1lrCallback(const std_msgs::Float64::ConstPtr &_msg)
{
    turnlr_ctrl = _msg->data;
    if(fabs(turnlr_ctrl)>0.01)
    {
        task_number = MOVE_TASK;  //for safety
        isControllerInput = true;
        isStableHover = false;
    }    
}

void AIBrain::joyCallback(const sensor_msgs::Joy::ConstPtr &msg){
    if(msg->buttons[0] > 0.5){
        //A
        task_number = LAND_TASK;
        isStableHover = false;
        if(isTestModeOfFindPath){
            isTestModeOfFindPath = false;
            ROS_INFO("[AIBRAIN]exit test mode of find path!");
        }
    }
    else if(msg->buttons[1] > 0.5){
        //B
        task_number = FINDPATH_TASK;
        isStableHover = false;
    }
    else if(msg->buttons[2] > 0.5){
        //X
        task_number = POINTTOPOINT_TASK;
        isStableHover = false;
    }
    else if(msg->buttons[3] > 0.5){
        //Y
        task_number = TAKEOFF_TASK;
        isStableHover = false;
        if(isTestModeOfFindPath){
            isTestModeOfFindPath = false;
            ROS_INFO("[AIBRAIN]exit test mode of find path!");
        }
    }
    else if(msg->buttons[4] > 0.5){
        //LB
        isTestModeOfFindPath = !isTestModeOfFindPath;
        state_findpath = 0;
        ROS_INFO("[AIBRAIN]enter test mode of find path!");
    }

    leftright_ctrl = msg->axes[0];
    forbackward_ctrl = msg->axes[1];
    turnlr_ctrl = msg->axes[3];
    updown_ctrl = msg->axes[4];
    if(fabs(leftright_ctrl)>0.01 || fabs(turnlr_ctrl)>0.01 ||
            fabs(updown_ctrl)>0.01 || fabs(forbackward_ctrl)>0.01){
        task_number = MOVE_TASK;  //for safety
        isControllerInput = true;
        isStableHover = false;
    }
}

bool AIBrain::setdestinationCallback(ai_robot_msgs::set_destination::Request &req, ai_robot_msgs::set_destination::Response &res)
{
    transform_NWUworld_from_body(req.x, req.y, req.z, poseXYZ_destination[0],
            poseXYZ_destination[1], poseXYZ_destination[2], Rworldslam, Tworldslam);
    float posSlam[3];
    transform_body_from_NWUworld(posSlam[0], posSlam[1], posSlam[2],
            poseXYZ_now[0], poseXYZ_now[1], poseXYZ_now[2], Rworldslam, Tworldslam);
    fpsrm.resetAll(posSlam[0], posSlam[1], posSlam[2],
            req.x, req.y, req.z);
    printf("[AIBRAIN]get (%f,%f,%f) in slam coordinate and set destination to (%f,%f,%f) in world.\n",
           req.x, req.y, req.z,
           poseXYZ_destination[0], poseXYZ_destination[1], poseXYZ_destination[2]);
//    poseXYZ_destination[0] = req.x;
//    poseXYZ_destination[1] = req.y;
//    poseXYZ_destination[2] = req.z;

    res.set_ok = true;
    return 1;
}
