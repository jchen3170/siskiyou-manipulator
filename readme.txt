INSTALLING/SETUP:

Required additional ROS packages (assuming you already have OpenCV):

    sudo apt-get install ros-$ROS_DISTRO-pointgrey-camera-driver
    sudo apt-get install ros-$ROS_DISTRO-cv-bridge

Required Python Modules:

    sudo apt-get install python-imaging-tk

Download PointGrey Camera Drivers (Model FireFlyMV FMVU-03MTC):
    
    Included drivers (from https://www.ptgrey.com/support/downloads):
        Ubuntu-16.04: flycapture2-2.11.3.121-amd64-pkg
        Ubuntu-14.04: flycapture2-2.9.3.43-amd64-pkg

    Download, extract, and install with instructions from the README.txt
    Restart computer

For port address (/ttyUSB#) after plugging in controller:

    dmesg | grep pl2303


USAGE:

To start:
    roscore (if roscore not already running)
    source the workspace (source <address to workspace>/devel/setup.bash)
    roslaunch siskiyou siskiyou.launch