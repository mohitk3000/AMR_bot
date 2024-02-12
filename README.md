**Control the AMR_Bot**

Start  ROSserial:

    rosrun rosserial_python serial_node.py /dev/ttyUSB0
    rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200

Teleop twist keyboard:

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

create package:

    catkin_create_pkg beginner_tutorials std_msgs rospy roscpp

Finding ros package:

    rospack find xxxx
    rosrun rviz rviz

build single package:

    catkin_make --pkg <my_package_name>
Dependencies install:

    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro noetic


starting lidar:

    roslaunch rplidar_ros rplidar_a2m12.launch 
 EKF launch :

     roslaunch odometry ekf_node.launch
starting Gmapping:

    rosrun gmapping slam_gmapping scan:=scan _odom_frame:=odom
    rosrun map_server map_saver -f <map_name>

starting the navigation:

    roslaunch navigation navigation.launch

Abort the navigation:

    rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}


find the serial port:

    ls /dev/tty*
permission for serial port

    sudo chmod 666 tty*
  
