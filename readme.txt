INSTALLING/SETUP:

Required ROS packages (assuming you already have OpenCV):

    sudo apt-get install ros-$ROS_DISTRO-pointgrey-camera-driver
    sudo apt-get install ros-$ROS_DISTRO-cv-bridge

Required Python Modules:

    sudo apt-get install python-imaging-tk

For port address:

    dmesg | grep pl2303

To start:

    roslaunch siskiyou siskiyou.launch


USAGE:

