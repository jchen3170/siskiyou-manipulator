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

For port address (/dev/ttyUSB#) after plugging in controller:

    dmesg | grep pl2303


USAGE:

To start:
    roscore (if roscore not already running)
    source the workspace (source <address to workspace>/devel/setup.bash)
    roslaunch siskiyou siskiyou.launch
Adjustments:
    siskiyouMain.py: change PORT to match your port address
    siskiyouVision.py: change the HSV min/max to adjust filter results
    siskiyouGUI.py: change SP/AC to adjust default max speed/acceleration values

GUI:

Values:
    Position: X, Y, Z positions in encoder counts
    Moving: indicates if axes are moving
    Limits: indicates if axes have hit their mechanical hardstop
    Status: raw 16-bit status. Look in MVP2001 manual for more information
    Velocity: current set velocity

Basic Controls:
    Zero: zero selected axis
    Move +: move in positive direction at default speed
    Move -: move in negative direction at default speed
    Stop: stop axis

Fixed Move:
    Enter distance in encoder counts and hit "move" (max range is ~4000000)

Advanced:
    Calibrate: preset cycle that centers every axis at their midpoint
    Flush: flushes output from controller for 1 sec.
    Power Cycle: software reset. (Use when motors get stuck on intialization)

Image:
    Display: click image to add points
    Move: move from current location to first selected point
    Undo: remove last added point
    Reset: remove all points
    Show Edges: show results of the edgemap of the filter
    Show Contour: show the contour approximation used to find the pipette tip