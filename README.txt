NOTE: Tested to be working on: 
    Ubuntu-16.04 with ros-kinetic 
    Ubuntu-14.04 with ros-indigo

INSTALLING/SETUP:

Required additional ROS packages (assuming you already have OpenCV):

    sudo apt-get install ros-$ROS_DISTRO-pointgrey-camera-driver
    sudo apt-get install ros-$ROS_DISTRO-cv-bridge

Required Python Modules:

    sudo apt-get install python-imaging-tk

Download/install PointGrey Camera Drivers (Model FireFlyMV FMVU-03MTC):
    
    Included drivers (from https://www.ptgrey.com/support/downloads):
        Ubuntu-16.04: flycapture2-2.11.3.121-amd64-pkg
        Ubuntu-14.04: flycapture2-2.9.3.43-amd64-pkg

    Download, extract, and install with instructions from the README.txt
    Restart computer

Setup ROS workspace (wiki.ros.org/catkin/Tutorials/create_a_workspace)


USAGE:

To start:
    roscore (if roscore not already running)
    source the workspace ("source <address to workspace>/devel/setup.bash")
        (test by calling "roscd siskiyou")
    roslaunch siskiyou siskiyou.launch
        OR (if you don't need the camera)
    rosrun siskiyou siskiyouMain.py

Adjustments:
    siskiyouMain.py: change PORT to match your port address if autofinding port
        is not working

    siskiyouVision.py: change the HSV min/max to adjust filter results

    siskiyouCommands.py: counts_per_mm is the actual conversion ratio of 
        encoder counts per mm specified by the manufacturer

    siskiyouGUI.py: 
        Change SP/AC to adjust default max speed/acceleration values.
        Add/remove commands from the program array for preset programs. The 
            commands should follow the format:

            "lambda:com.<function>(<inputs>)"
            
            All functions used should come from siskiyouCommands (descriptions
            are listed for each function). The current set program moves the
            end effector in a 5mm x 5mm square trajectory


GUI:

Values:
    Position: X, Y, Z positions in encoder counts
    Moving: indicates if axes are moving
    Limits: indicates if axes have hit their mechanical hardstop
    Status: raw 16-bit status. Look in MVP2001 manual for more information
    Change Units: swap between encoder counts and mm for input/output

Basic Controls:
    Zero: zero selected axis
    Move +: move in positive direction at default speed
    Move -: move in negative direction at default speed
    Stop: stop axis

Fixed Move:
    Enter distance in encoder counts or mm and hit "move" 
    Max range is ~4000000 cts or 20mm

Advanced:
    Program: run preset program specified in siskiyouGUI
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