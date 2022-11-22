# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/local/include/eigen3;/usr/local/include".split(';') if "${prefix}/include;/usr/local/include/eigen3;/usr/local/include" != "" else []
PROJECT_CATKIN_DEPENDS = "dynamic_reconfigure;geometry_msgs;laser_geometry;map_msgs;message_filters;message_runtime;nav_msgs;pluginlib;roscpp;sensor_msgs;std_msgs;tf2_ros;visualization_msgs;voxel_grid".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lcostmap_2d;-llayers;/usr/local/lib/libboost_system.so;/usr/local/lib/libboost_thread.so;-lpthread;/usr/local/lib/libboost_chrono.so;/usr/local/lib/libboost_date_time.so;/usr/local/lib/libboost_atomic.so".split(';') if "-lcostmap_2d;-llayers;/usr/local/lib/libboost_system.so;/usr/local/lib/libboost_thread.so;-lpthread;/usr/local/lib/libboost_chrono.so;/usr/local/lib/libboost_date_time.so;/usr/local/lib/libboost_atomic.so" != "" else []
PROJECT_NAME = "costmap_2d"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "1.16.7"
