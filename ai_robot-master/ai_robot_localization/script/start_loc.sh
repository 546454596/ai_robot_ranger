#! /bin/bash
cd ${HOME}/Record/map
# choose folder
MapFolder=$(yad --width=800 --height=400 \
  --title="Select map folder" --file-selection --directory)
echo ${MapFolder}

# start localization
roslaunch ai_robot_localization carto_hw_loc.launch load_state_filename:=${MapFolder}/map.pbstream init_pose_file:=${MapFolder}/traj0_endpose.txt
