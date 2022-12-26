
对压缩包src解压，如果只测试基于高程地图的导航，可以删除ai_robot_clean、cmake_build-debug、darknet_ros、navigation、slam_toolbox。这里面部分包编译中容易出现问题。

创建工作空间：mkdir -p ai_robot_ranger/src
将解压的文件放入创建的工作目录src下
退出src，直接进行编译catkin_make，如果出现缺少依赖，根据提示安装即可

使用过程：
1.打开仿真环境
roslaunch ranger_mini_gazebo ai_robot_ranger_gazebo.launch
2.载入地图，打开AMCL定位
roslaunch ai_robot_nav_demo brainnav.launch path:=false
3.打开基于拓扑地图的全局导航
roslaunch ai_robot_navigation globalnavsim.launch
4.打开高程建图、可视化node
roslaunch elevation_mapping_demos mytest_gazebo2.launch 
5.打开基于高程地图的局部避障
roslaunch sele_path_planner testgazebo.launch 


对速度进行平滑：
1.启动仿真场景
roslaunch ranger_mini_gazebo ai_robot_ranger_gazebo.launch
2.启动定位与路径规划
roslaunch ai_robot_nav_demo brainnav.launch path:=true
3.对TEB输出的速度cmd_vel进一步进行平滑处理，得到sim_p3at/cmd_vel
roslaunch yocs_velocity_smoother standalone.launch 

基于拓扑地图的全局规划雨局部避障：
在该部分代码中，初步实现了输出拓扑轨迹（在使用中，需要注释掉move_base_simple/goal话题，使用globalsim作为目的地输入，并输出拓扑轨迹，用于TEB），并将轨迹传递至TEB模块，但目前实现的无法实现导航至目的地，出现乱跑的行为。
  //建立了接收拓扑轨迹的topic，作为全局规划的路点，并计算控制速度
  void MoveBase::pathCB(const nav_msgs::Path &path) {
      geometry_msgs::PoseStamped single_plan_msg;
      planner_plan_->clear();
      planner_plan_->resize(path.poses.size());
      std::cout << planner_plan_->size() << std::endl;
      //第一个为目的地，最后一个为起点,为了使轨迹是起点指向终点，所以倒序传入planner_plan_
      for (int i = path.poses.size() - 1; i>=0; --i) {
          single_plan_msg.pose = path.poses[i].pose;
          single_plan_msg.header = path.poses[i].header;
          single_plan_msg.header.frame_id = "map";
          planner_plan_->push_back(single_plan_msg);
      }
      planner_plan_->erase(planner_plan_->cbegin(), planner_plan_->cbegin()+path.poses.size());
      for(auto &i : *planner_plan_)
          std::cout << i << std::endl;
/*      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      tc_->setPlan(*planner_plan_);
      geometry_msgs::Twist cmd_vel;
      tc_->computeVelocityCommands(cmd_vel);
      vel_pub_.publish(cmd_vel);
      lock.unlock();*/
  }
