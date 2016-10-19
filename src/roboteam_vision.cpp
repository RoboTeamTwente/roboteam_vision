#include <string>

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/RefboxCmd.h"

#include "net/robocup_ssl_client.h"

#include "proto_to_ros.h"


// The max amount of cameras.
const uint NUM_CAMS = 5;

// Keeps track of the last frames sent by the cameras.
// This allows us to drop duplicate or delayed frames.
uint last_frames [NUM_CAMS];

// Keeps track of which team is us.
// True is yellow, false is blue.
bool us_is_yellow;


/**
 * Sets the `our_color` parameter to the default "yellow".
 * Call if the parameter hasn't been set yet, or was set to garbage.
 */
void default_our_color_parameter() {
    // Default to yellow.
    us_is_yellow = true;
    ros::param::set("/our_color", "yellow");
}


/**
 * Reads in the `our_color` parameter.
 * If the parameter is not set, defaults to yellow.
 */
void read_our_color_parameter() {
    std::string our_color;
    if (ros::param::get("/our_color", our_color)) {
        if (our_color == "yellow") {
            us_is_yellow = true;
        } else if (our_color == "blue") {
            us_is_yellow = false;
        } else {
            default_our_color_parameter();
        }
    } else {
        default_our_color_parameter();
    }
}


/**
 * Resets the 'last_frames' array back to 0.
 */
void reset_frames() {
    for (int i = 0; i < NUM_CAMS; ++i) {
        last_frames[i] = 0;
    }
}


/**
 * Resets the vision system.
 * To be called on `vision_reset`.
 *
 * Calls `read_our_color_parameter`
 * and calls `reset_frames`.
 */
bool vision_reset(std_srvs::Empty::Request& req,
                  std_srvs::Empty::Response& res) {

    read_our_color_parameter();
    reset_frames();

    return true;
}


// Sends a SSL_DetectionFrame out through the supplied publisher.
void send_detection_frame(SSL_DetectionFrame detectionFrame, ros::Publisher publisher) {
    uint cam_id = detectionFrame.camera_id();

    if (cam_id < NUM_CAMS) {
        // Check if we haven't already received this frame from this camera.
        // TODO: See if it is necessary to remove duplicate frames.
        // Are they even duplicate?
        // if (protoFrame.frame_number() > last_frames[cam_id]) {

            last_frames[cam_id] = detectionFrame.frame_number();

            // Convert the detection frame.
            roboteam_msgs::DetectionFrame frame = convert_detection_frame(detectionFrame, us_is_yellow);
            // Publish the frame.
            publisher.publish(frame);
        //}
    }
}



int main(int argc, char **argv)
{
    // Init ros.
    ros::init(argc, argv, "roboteam_msgs");
    ros::NodeHandle n;

    // Run at 200 hz.
    ros::Rate loop_rate(200);

    ros::Publisher detection_pub = n.advertise<roboteam_msgs::DetectionFrame>("vision_detection", 1000);
    ros::Publisher geometry_pub = n.advertise<roboteam_msgs::GeometryData>("vision_geometry", 1000);
    ros::Publisher refbox_pub = n.advertise<roboteam_msgs::RefboxCmd>("vision_refbox", 1000);

    // Add the service to reset the last frame trackers.
    ros::ServiceServer reset_service = n.advertiseService("vision_reset", vision_reset);


    RoboCupSSLClient vision_client = RoboCupSSLClient(10006, "224.5.23.2");
    RoboCupSSLClient refbox_client = RoboCupSSLClient(10003, "224.5.23.1");;

    // Open the clients, blocking = false.
    vision_client.open(false);
    refbox_client.open(false);

    // Initialize which team we are.
    read_our_color_parameter();

    ROS_INFO("Vision ready.");

    SSL_WrapperPacket vision_packet;
    Refbox_Log refbox_packet;

    while (ros::ok()) {
        while (vision_client.receive(vision_packet)) {

            // Detection frame.
            if (vision_packet.has_detection()) {
                send_detection_frame(vision_packet.detection(), detection_pub);
            }

            if (vision_packet.has_geometry()) {
                // Convert the geometry frame.
                roboteam_msgs::GeometryData data = convert_geometry_data(vision_packet.geometry());
                // Publish the data.
                geometry_pub.publish(data);
            }
        }

        while (refbox_client.receive(refbox_packet)) {

            // Don't forward empty referee commands.
            if (refbox_packet.log_size() > 0) {

                roboteam_msgs::RefboxCmd cmd;

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
