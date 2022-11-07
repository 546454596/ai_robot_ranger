// paramfile should be input to run the exe

#include "findpathsrm.h"
#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv,"testfp");
    ros::NodeHandle nh;
    ros::Rate r(50);
    int64_t start1=0,end1=0;
    FindPathSRM fpsrm(nh, true);

    // open log file
    // string logpath;
    // nh.getParam("logfilepath", logpath);
    // ofstream out;  
    // out.open(logpath, ios::out);

    // read start end nodes
    string paramfile; 
    nh.getParam("paramfile", paramfile);
    cout<<"st pos file path "<<paramfile<<endl;
    cv::FileStorage fsSettings(paramfile.c_str(), cv::FileStorage::READ);
    cv::Mat tmp;
    fsSettings["startendpos"] >> tmp;
    vector<vector<double> > stenpos;    
    vector<double> stentmp;
    stentmp.resize(6);
    for(int i=0; i<tmp.rows; ++i)
    {
        stentmp[0] = double(tmp.at<double>(i,0));
        stentmp[1] = double(tmp.at<double>(i,1));
        stentmp[2] = double(tmp.at<double>(i,2));
        stentmp[3] = double(tmp.at<double>(i,3));
        stentmp[4] = double(tmp.at<double>(i,4));
        stentmp[5] = double(tmp.at<double>(i,5));
        stenpos.push_back(stentmp);
        // cout<<"read start end node ["<<stentmp[0]<<", "<<stentmp[1]<<"], ["<<stentmp[2]<<", "<<stentmp[3]<<"]"<<endl;
    }
    fpsrm.display();
    int a;
    cin >> a;

    // fpsrm.resetAll(stenpos[0][0], stenpos[0][1], stenpos[0][2], stenpos[0][3], stenpos[0][4], stenpos[0][5]);
    // fpsrm.findPath();
    int i=0, testtime = 10;
    while (true){//ros::ok()
        if(i >= stenpos.size()) break;
        cout<<"start end node ["<<stenpos[i][0]<<", "<<stenpos[i][1]<<", "<<stenpos[i][2]<<"], ["
            <<stenpos[i][3]<<", "<<stenpos[i][4]<<", "<<stenpos[i][5]<<"]"<<endl;
        // start1 = cv::getTickCount();
        // findpath
        for(int j=0; j<testtime; ++j)
        {
            fpsrm.resetAll(stenpos[i][0], stenpos[i][1], stenpos[i][2], stenpos[i][3], stenpos[0][4], stenpos[0][5]);
            fpsrm.findPath();
        }
        
        // end1 = cv::getTickCount();
        // time for find path
        // double _t = 1000*double(end1 - start1)/cv::getTickFrequency();
        // length of path
        // double length = fpsrm.getShortestPathLength();
        // write to file
        // out << _t << " " << length << endl;
        // cout << "computation time: "<<_t<< ", length:"<<length<<endl;
        ++i;
        // fpsrm.display();
        // int a;
        // cin >> a;
        // ros::spinOnce();
        // r.sleep();
    }
    // out.close();
    cout<<"test end."<<endl;

    ros::shutdown();
    return 0;
}