
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



初步版本，实现手柄控制、远程遥控
