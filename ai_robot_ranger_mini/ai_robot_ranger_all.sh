#!/bin/bash
## change these to whatever you actually need
command1="cd /home/cx/brainnavi/ai_robot_ranger;source devel/setup.bash;roslaunch ranger_mini_gazebo ai_robot_ranger_gazebo.launch; bash"
command2="sleep 2s;cd /home/cx/brainnavi/ai_robot_ranger; source devel/setup.bash;roslaunch ai_robot_nav_demo brainnav.launch;exec bash; bash"
command3="sleep 3s;cd /home/cx/brainnavi/ai_robot_ranger; source devel/setup.bash; roslaunch ai_robot_navigation globalnavsim.launch;exec bash; bash"
command4="sleep 4s;cd /home/cx/brainnavi/ai_robot_ranger; source devel/setup.bash; roslaunch ai_robot_navigation obsavoidsim.launch; bash"
## Modify terminator's config
sed -i.bak "s#COMMAND1#$command1#; s#COMMAND2#$command2#; s#COMMAND3#$command3#; s#COMMAND4#$command4#;" ~/.config/terminator/config
## Launch a terminator instance using the new layout
terminator -l default
## Return the original config file
mv ~/.config/terminator/config.bak ~/.config/terminator/config
