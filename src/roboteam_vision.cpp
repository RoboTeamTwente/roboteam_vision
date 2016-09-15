#include <iostream>

#include "ros/ros.h"
#include "roboteam_vision/DetectionFrame.h"

#include "net/robocup_ssl_client.h"


int main(int argc, char **argv)
{
    // Init ros.
    ros::init(argc, argv, "roboteam_vision");
    ros::NodeHandle n;

    ros::Publisher detection_pub = n.advertise<roboteam_vision::DetectionFrame>("vision_detection", 1000);


    RoboCupSSLClient client;

    // Open the client, blocking = false.
    client.open(false);

    SSL_WrapperPacket packet;

    while (ros::ok()) {
        while (client.receive(packet)) {

            // Detection package.
            if (packet.has_detection()) {
                roboteam_vision::DetectionFrame msg;

                SSL_DetectionFrame frame = packet.detection();

                msg.frame_number = frame.frame_number();
                msg.t_capture = frame.t_capture();
                msg.t_sent = frame.t_sent();
                msg.camera_id = frame.camera_id();

                // TODO This can probably be done more efficient.
                // Maybe with std::copy or something like that.
                for (int i = 0; i < frame.balls().size(); ++i) {
                    SSL_DetectionBall protoBall = frame.balls().Get(i);
                    roboteam_vision::DetectionBall rosBall;

                    rosBall.confidence = protoBall.confidence();
                    rosBall.area = protoBall.area();
                    rosBall.x = protoBall.x();
                    rosBall.y = protoBall.y();
                    rosBall.z = protoBall.z();
                    rosBall.pixel_x = protoBall.pixel_x();
                    rosBall.pixel_y = protoBall.pixel_y();

                    msg.balls.push_back(rosBall);
                }


                for (int i = 0; i < frame.robots_yellow().size(); ++i) {
                    SSL_DetectionRobot protoBot = frame.robots_yellow().Get(i);
                    roboteam_vision::DetectionRobot rosBot;

                    rosBot.confidence = protoBot.confidence();
                    rosBot.robot_id = protoBot.robot_id();
                    rosBot.x = protoBot.x();
                    rosBot.y = protoBot.y();
                    rosBot.orientation = protoBot.orientation();
                    rosBot.pixel_x = protoBot.pixel_x();
                    rosBot.pixel_y = protoBot.pixel_y();
                    rosBot.height = protoBot.height();

                    msg.robots_yellow.push_back(rosBot);
                }

                // TODO The bot conversion can probably be put in a function
                // as it is called twice.
                for (int i = 0; i < frame.robots_blue().size(); ++i) {
                    SSL_DetectionRobot protoBot = frame.robots_blue().Get(i);
                    roboteam_vision::DetectionRobot rosBot;

                    rosBot.confidence = protoBot.confidence();
                    rosBot.robot_id = protoBot.robot_id();
                    rosBot.x = protoBot.x();
                    rosBot.y = protoBot.y();
                    rosBot.orientation = protoBot.orientation();
                    rosBot.pixel_x = protoBot.pixel_x();
                    rosBot.pixel_y = protoBot.pixel_y();
                    rosBot.height = protoBot.height();

                    msg.robots_blue.push_back(rosBot);
                }

                // Publish the message.
                detection_pub.publish(msg);
            }
        }

        ros::spinOnce();
    }

    return 0;
}
