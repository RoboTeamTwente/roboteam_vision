#include <iostream>

#include "ros/ros.h"
#include "roboteam_vision/DetectionFrame.h"

#include "net/robocup_ssl_client.h"


int main(int argc, char **argv)
{
    // Init ros.
    //ros::init(argc, argv, "roboteam_vision");
    //ros::NodeHandle n;


    RoboCupSSLClient client;

    // Open the client, blocking = false.
    client.open(false);

    SSL_WrapperPacket packet;

    while (true) {
        while (client.receive(packet)) {
            if (packet.has_detection()) {
                std::cout << "It works!";
            }
        }
    }

    return 0;
}
