#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "roboteam_vision/DetectionFrame.h"
#include "roboteam_vision/RefboxCmd.h"

#include "net/robocup_ssl_client.h"

#include "proto_to_ros.h"


// The max amount of cameras.
const uint NUM_CAMS = 5;

// Keeps track of the last frames sent by the cameras.
// This allows us to drop duplicate or delayed frames.
uint last_frames [NUM_CAMS];


/**
 * Resets the 'last_frames' array back to 0.
 */
bool reset_frames(std_srvs::Empty::Request& req,
                  std_srvs::Empty::Response& res) {
    for (int i = 0; i < NUM_CAMS; ++i) {
        last_frames[i] = 0;
    }

    return true;
}


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

    // Add the service to reset the last frame trackers.
    ros::ServiceServer reset_service = n.advertiseService("vision_reset", reset_frames);


    RoboCupSSLClient vision_client = RoboCupSSLClient(10006, "224.5.23.2");
    RoboCupSSLClient refbox_client = RoboCupSSLClient(10003, "224.5.23.1");;

    // Open the clients, blocking = false.
    vision_client.open(false);
    refbox_client.open(false);

    ROS_INFO("Vision ready.");

    SSL_WrapperPacket vision_packet;
    Refbox_Log refbox_packet;

    while (ros::ok()) {
        while (vision_client.receive(vision_packet)) {

            // Detection package.
            if (vision_packet.has_detection()) {
                SSL_DetectionFrame protoFrame = vision_packet.detection();

                uint cam_id = protoFrame.camera_id();

                if (cam_id < NUM_CAMS) {
                    // Check if we haven't already received this frame from this camera.
                    if (protoFrame.frame_number() > last_frames[cam_id]) {

                        last_frames[cam_id] = protoFrame.frame_number();

                        // Convert the detection frame.
                        roboteam_vision::DetectionFrame frame = convert_detection_frame(protoFrame);
                        // Publish the frame.
                        detection_pub.publish(frame);
                    }
                }
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
