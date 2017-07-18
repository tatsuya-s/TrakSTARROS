#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "trakstar_ros/Sensor.h"

class VizTest {
    public:
        VizTest();
        ~VizTest();
        void PseudoColor(visualization_msgs::Marker& marker, double phase);
        void SensorCallback(const trakstar_ros::Sensor::ConstPtr& msg);
    private:
        ros::NodeHandle nh;
        ros::Publisher marker_pub;
        ros::Subscriber sensor_sub;
        ros::Rate loop_rate;
        visualization_msgs::MarkerArray markers;
};

VizTest::VizTest() : loop_rate(10) {
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
    sensor_sub = nh.subscribe("TrakSTAR/Sensor", 100, &VizTest::SensorCallback, this);
}

VizTest::~VizTest() {
}

void VizTest::PseudoColor(visualization_msgs::Marker& marker, double phase) {
    marker.color.r = (std::sin(1.5 * M_PI * phase + M_PI + M_PI / 4) + 1) / 2.0;
    marker.color.g = (std::sin(1.5 * M_PI * phase + M_PI + M_PI / 4 + M_PI / 2) + 1) / 2.0;
    marker.color.b = (std::sin(1.5 * M_PI * phase + M_PI + M_PI / 4 + M_PI) + 1) / 2.0;
    marker.color.a = 1.0;
}

void VizTest::SensorCallback(const trakstar_ros::Sensor::ConstPtr& msg) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/my_frame";
    marker.header.stamp = msg->header.stamp;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.lifetime = ros::Duration(5.0);

    for (int i = 0; i < msg->pose_array.size(); ++i) {
        marker.ns = "sensor" + std::to_string(i + 1);
        marker.id = i;
        marker.pose = msg->pose_array[i];
        PseudoColor(marker, 1.0 * i / msg->pose_array.size());
        this->markers.markers.push_back(marker);
    }

    marker_pub.publish(markers);
    markers.markers.clear();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "viz_test");
    
    VizTest viz;

    ros::spin();

    return 0;
}
