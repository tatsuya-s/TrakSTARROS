# TrakSTARROS

ROS driver for the TrakSTAR.
This project includes following Linux USB driver package.

* `ATC3DGTracker` - from: https://github.com/ChristophJud/ATC3DGTracker

## Dependencies

* `libusb`

## Installation

    $ sudo apt-get install libusb-dev
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/tatsuya-s/TrakSTARROS
    $ cd ~/catkin_ws/
    $ catkin build
    $ sudo cp ~/catkin_ws/src/TrakSTARROS/99-libusb.rules /etc/udev/rules.d

## Usage

    $ rosrun trakstar_ros trakstar_node