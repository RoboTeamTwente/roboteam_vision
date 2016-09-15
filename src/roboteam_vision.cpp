#include <iostream>

#include "ros/ros.h"
#include "roboteam_vision/DetectionFrame.h"

#include "net/robocup_ssl_client.h"

#include "proto_to_ros.h"


int main(int argc, char **argv)
{
    // Init ros.
    ros::init(argc, argv, "roboteam_vision");
    ros::NodeHandle n;

    // Run at 200 hz.
    ros::Rate loop_rate(200);

    ros::Publisher detection_pub = n.advertise<roboteam_vision::DetectionFrame>("vision_detection", 1000);
    ros::Publisher geometry_pub = n.advertise<roboteam_vision::GeometryData>("vision_geometry", 1000);


    RoboCupSSLClient client;

    // Open the client, blocking = false.
    client.open(false);

    SSL_WrapperPacket packet;

    while (ros::ok()) {
        while (client.receive(packet)) {

            // Detection package.
            if (packet.has_detection()) {
                // Convert the detection frame.
                roboteam_vision::DetectionFrame frame = convert_detection_frame(packet.detection());
                // Publish the frame.
                detection_pub.publish(frame);
            }

            if (packet.has_geometry()) {
                // Convert the geometry frame.
                roboteam_vision::GeometryData data = convert_geometry_data(packet.geometry());
                // Publish the data.
                geometry_pub.publish(data);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
