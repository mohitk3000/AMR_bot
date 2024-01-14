# AMRbot
this repo contains all codes and docs for AMR project done under ARTPARK

**Start  ROSserial:**

    rosrun rosserial_python serial_node.py /dev/ttyUSB0
    rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200

Teleop twist keyboard:

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

create package:

    catkin_create_pkg beginner_tutorials std_msgs rospy roscpp

build single package:
```
catkin_make --pkg <my_package_name>
```

find the serial port:

    ls /dev/tty*
