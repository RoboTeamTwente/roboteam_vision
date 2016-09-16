#include <iostream>

#include "ros/ros.h"
#include "roboteam_vision/DetectionFrame.h"
#include "roboteam_vision/RefboxCmd.h"

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
    ros::Publisher refbox_pub = n.advertise<roboteam_vision::RefboxCmd>("vision_refbox", 1000);


    RoboCupSSLClient vision_client = RoboCupSSLClient(10006, "224.5.23.2");
    RoboCupSSLClient refbox_client = RoboCupSSLClient(10003, "224.5.23.1");;

    // Open the clients, blocking = false.
    vision_client.open(false);
    refbox_client.open(false);

    SSL_WrapperPacket vision_packet;
    Refbox_Log refbox_packet;

    ROS_INFO("Vision ready");

    while (ros::ok()) {
        while (vision_client.receive(vision_packet)) {

            // Detection package.
            if (vision_packet.has_detection()) {
                // Convert the detection frame.
                roboteam_vision::DetectionFrame frame = convert_detection_frame(vision_packet.detection());
                // Publish the frame.
                detection_pub.publish(frame);
            }

            if (vision_packet.has_geometry()) {
                // Convert the geometry frame.
                roboteam_vision::GeometryData data = convert_geometry_data(vision_packet.geometry());
                // Publish the data.
                geometry_pub.publish(data);
            }
        }

        while (refbox_client.receive(refbox_packet)) {

            // Don't forward empty referee commands.
            if (refbox_packet.log_size() > 0) {

                roboteam_vision::RefboxCmd cmd;

                for (int i = 0; i < refbox_packet.log().size(); ++i) {
                    Log_Frame frame = refbox_packet.log().Get(i);

                    cmd.refbox_cmd.push_back(frame.refbox_cmd());
                }

                // Publish the commands.
                refbox_pub.publish(cmd);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
