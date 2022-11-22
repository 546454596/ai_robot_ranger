# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/local/include".split(';') if "${prefix}/include;/usr/local/include" != "" else []
PROJECT_CATKIN_DEPENDS = "cv_bridge;roscpp;actionlib;rospy;std_msgs;darknet_ros_msgs;image_transport;nodelet".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ldarknet_ros_lib;/usr/local/lib/libboost_thread.so;-lpthread;/usr/local/lib/libboost_chrono.so;/usr/local/lib/libboost_system.so;/usr/local/lib/libboost_date_time.so;/usr/local/lib/libboost_atomic.so".split(';') if "-ldarknet_ros_lib;/usr/local/lib/libboost_thread.so;-lpthread;/usr/local/lib/libboost_chrono.so;/usr/local/lib/libboost_system.so;/usr/local/lib/libboost_date_time.so;/usr/local/lib/libboost_atomic.so" != "" else []
PROJECT_NAME = "darknet_ros"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "1.1.5"
