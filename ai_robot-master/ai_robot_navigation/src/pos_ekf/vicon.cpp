#include "vicon.h"
extern deque< pair<double, Vector3d> >vicon_pose_q;
extern deque< pair<double, Vector3d> >vicon_vel_q;
void Vicon::setup()
{
     ROS_INFO("[VICON]Vicon Setup ....");

      state.x=0;
      state.y=0;
      state.z=0;
      state.vx=0;
      state.vy=0;
      state.vz=0;
      state.roll=0;
      state.pitch=0;
      state.yaw=0;
      for(int i=0;i<4;i++){
          qWFromV[i]=0;
          qbw_start[i]=0;
      }

      viconToWorld_pub=nh.advertise<geometry_msgs::Transform>("/vicon/transformVToW",1);

      //*********************kf for vicon velocity*****************
      viconKF_vx=0;viconKF_vy=0;viconKF_vz=0;
      kfP=1;kfQ=0.00001;kfR=0.01;kfKk=0;
      //***********************************************************
      //*********************kf for vicon acceleration*****************
      viconKF_ax=0;viconKF_ay=0;viconKF_az=0;
      acckfP=1;acckfQ=0.00001;acckfR=0.01;acckfKk=0;
      //***********************************************************

     ROS_INFO("[VICON]Vicon Setup  over!");
}

void Vicon::viconDataCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
    static double transformSendTime=ros::Time::now().toSec();
    double qbv[4],pose_vicon[3],pose[3];
    qbv[0]=msg->transform.rotation.w;
    qbv[1]=msg->transform.rotation.x;
    qbv[2]=msg->transform.rotation.y;
    qbv[3]=msg->transform.rotation.z;
    pose_vicon[0]=msg->transform.translation.x;
    pose_vicon[1]=msg->transform.translation.y;
    pose_vicon[2]=msg->transform.translation.z;
//    double dt=msg->header.stamp.toSec()-dataTime;
//    dataTime=msg->header.stamp.toSec();
    if(!isViconUpdate){
            start_time=msg->header.stamp.toSec();
            frameCountStart=msg->header.seq;
            frameCountLast=frameCount=msg->header.seq;
            isViconUpdate=true;
            return;
        }

        frameCount=msg->header.seq;
        double dt=(frameCount-frameCountLast)*timeBetweenFrame;
        frameCountLast=frameCount;

    if(!isViconInit){
        if(isSetQbw){
            //conjugate
        qbv[1]=-qbv[1];
        qbv[2]=-qbv[2];
        qbv[3]=-qbv[3];
        qmultiplyq(qbw_start,qbv,qWFromV);
        getRotation(qWFromV,Rwv);
        translation[0]=pose_vicon[0];
        translation[1]=pose_vicon[1];
        translation[2]=pose_vicon[2];

        viconToWorldTransform.translation.x=translation[0];
        viconToWorldTransform.translation.y=translation[1];
        viconToWorldTransform.translation.z=translation[2];
        viconToWorldTransform.rotation.w=qWFromV[0];
        viconToWorldTransform.rotation.x=qWFromV[1];
        viconToWorldTransform.rotation.y=qWFromV[2];
        viconToWorldTransform.rotation.z=qWFromV[3];

        viconToWorld_pub.publish(viconToWorldTransform);

        state.x=0;
        state.y=0;
        state.z=0;
        pose_last[0]=state.x;
        pose_last[1]=state.y;
        pose_last[2]=state.z;
        isposedata=true;
        isViconInit=true;
        }
    }
    else{
        if(ros::Time::now().toSec()-transformSendTime>3.3){//reduce the sending frequence to 3Hz while ensure the transform information is aqcuired by sdk
            transformSendTime=ros::Time::now().toSec();
            viconToWorld_pub.publish(viconToWorldTransform);
        }
        qmultiplyq(qWFromV,qbv,qbw);
        qtoEuler(state.roll,state.pitch,state.yaw,qbw);
        for(int i=0;i<3;++i)pose_vicon[i]-=translation[i];
        transform_world_from_vicon(pose_vicon,pose);
        state.x=pose[0];
        state.y=pose[1];
        state.z=pose[2];
        if(isposedata){
            state.vx=(pose[0]-pose_last[0])/dt;
            state.vy=(pose[1]-pose_last[1])/dt;
            state.vz=(pose[2]-pose_last[2])/dt;
            pose_last[0]=state.x;
            pose_last[1]=state.y;
            pose_last[2]=state.z;

            //***********KF for vicon velocity**********
//                            double zk_vx,zk_vy,zk_vz;
//                            zk_vx=state.vx;
//                            zk_vy=state.vy;
//                            zk_vz=state.vz;
//                            viconKF_vx=viconKF_vx;
//                            viconKF_vy=viconKF_vy;
//                            viconKF_vz=viconKF_vz;
//                            kfP=kfP+kfQ;
//                            kfKk=kfP/(kfP+kfR);
//                            viconKF_vx=viconKF_vx+kfKk*(zk_vx-viconKF_vx);
//                            viconKF_vy=viconKF_vy+kfKk*(zk_vy-viconKF_vy);
//                            viconKF_vz=viconKF_vz+kfKk*(zk_vz-viconKF_vz);
//                            kfP=(1-kfKk)*kfP;
//                            state.vx=viconKF_vx;
//                            state.vy=viconKF_vy;
//                            state.vz=viconKF_vz;
                            //****************************************//

            if(!isveldata){
                vel_last[0]=state.vx;
                vel_last[1]=state.vy;
                vel_last[2]=state.vz;
            }
            isveldata=true;
        }
    }
}

void Vicon::qtoEuler(double& _roll,double& _pitch,double& _yaw,double q[])
{
    float_t w,x,y,z;
    //diffrent coordinates
    w=q[0];
    x=q[1];
    y=q[2];
    z=q[3];
    //cout<<"q:"<<w<<"    "<<x<<"    "<<y<<"    "<<z<<endl;
    _roll  = atan2(2 * (w * x + y* z) , 1 - 2 * (x * x + y * y));
    if(2 * (w * y - z * x)>1)_pitch =asin(1.0) ;
    else if(2 * (w * y - z * x)<-1.0)_pitch = asin(-1.0);
    else _pitch = asin(2 * (w * y - z * x));
    _yaw   = atan2(2 * (w * z + x * y) , 1 - 2 * (y * y + z * z));
}

void Vicon::qmultiplyq(double q1[],double q2[],double q_result[])
{
    q_result[0]=q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
    q_result[1]=q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2];
    q_result[2]=q1[0]*q2[2]+q1[2]*q2[0]+q1[3]*q2[1]-q1[1]*q2[3];
    q_result[3]=q1[0]*q2[3]+q1[3]*q2[0]+q1[1]*q2[2]-q1[2]*q2[1];
}

void Vicon::getqInworld(float q[])
{
    for(int i=0;i<4;++i)qbw_start[i]=q[i];
    isSetQbw=true;
    //can't put this subscribe in setup, if put it in setup will cause conflict of read and write of 'isSetQbw'
    vicondata_sub=nh.subscribe("/vicon/uav_stereo02/uav_stereo02", 20,&Vicon::viconDataCallback,this);
}

void  Vicon::getRotation(double q[],double R[])
{
    float_t w,x,y,z;
    // coordinates in NWU
    w=q[0];
    x=q[1];
    y=q[2];
    z=q[3];

    R[0]=w*w+x*x-y*y-z*z;
    R[1]=2*(x*y-w*z);
    R[2]=2*(x*z+w*y);
    R[3]=2*(x*y+w*z);
    R[4]=w*w-x*x+y*y-z*z;
    R[5]=2*(y*z-w*x);
    R[6]=2*(x*z-w*y);
    R[7]=2*(y*z+w*x);
    R[8]=w*w-x*x-y*y+z*z;
}

void Vicon::transform_world_from_vicon(const double a_vicon[3],double a_world[3])
{
    for(int i=0;i<3;++i) a_world[i]=0;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            a_world[i]+=Rwv[j+3*i]*a_vicon[j];
        }
    }
}
