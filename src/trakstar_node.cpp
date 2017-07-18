#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "tf/tf.h"
#include "trakstar_ros/Sensor.h"
#include "Utils.h"
#include "PointATC3DG.h"

class TrakSTARROS {
    public:
        TrakSTARROS();
        ~TrakSTARROS();
        void MainLoop();

    private:
        ros::NodeHandle nh;
        ros::Publisher sensor_pub;
        ros::Rate loop_rate;
        PointATC3DG bird;
        int sensor_num;
};

TrakSTARROS::TrakSTARROS() : loop_rate(10) {
    this->sensor_pub = this->nh.advertise<trakstar_ros::Sensor>("TrakSTAR/Sensor", 100);
    if (!this->bird) {
        ROS_ERROR("Failed to initialize trakSTAR");
    }
    this->bird.setSuddenOutputChangeLock(0);
    this->sensor_num = bird.getNumberOfSensors();

}

TrakSTARROS::~TrakSTARROS() {
}

void TrakSTARROS::MainLoop() {
    double sensor_x, sensor_y, sensor_z;
    double sensor_azim, sensor_elev, sensor_roll;
    trakstar_ros::Sensor sensor_msg;
    std_msgs::Header header;
    geometry_msgs::Pose pose_msg;

    while (ros::ok() & !kbhit()) {
        for (int i = 0; i < this->sensor_num; ++i) {
            this->bird.getCoordinatesAngles(i, sensor_x, sensor_y, sensor_z
                                             , sensor_azim, sensor_elev, sensor_roll);
            pose_msg.position.x = sensor_x * 0.0254;
            pose_msg.position.y = sensor_y * 0.0254;
            pose_msg.position.z = sensor_z * 0.0254;
            sensor_azim = sensor_azim * M_PI / 180.0;
            sensor_elev = sensor_elev * M_PI / 180.0;
            sensor_roll = sensor_roll * M_PI / 180.0;
            pose_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(sensor_roll, sensor_elev, sensor_azim);
            sensor_msg.pose_array.push_back(pose_msg);
        }
        
        sensor_msg.header.stamp = ros::Time::now();
        this->sensor_pub.publish(sensor_msg);
        sensor_msg.pose_array.clear();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "TrakSTAR_node");
    
    TrakSTARROS trakstar;

    trakstar.MainLoop();

    return 0;
}