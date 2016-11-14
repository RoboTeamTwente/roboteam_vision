#include <string>

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/GeometryData.h"
#include "roboteam_msgs/RefereeData.h"

#include "net/robocup_ssl_client.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_wrapper_legacy.pb.h"

#include "convert/convert_detection.h"
#include "convert/convert_geometry_current.h"
#include "convert/convert_geometry_legacy.h"
#include "convert/convert_referee.h"


// The max amount of cameras.
const uint NUM_CAMS = 5;

// Keeps track of the last frames sent by the cameras.
// This allows us to drop duplicate or delayed frames.
uint last_frames [NUM_CAMS];

// Keeps track of which team is us.
// True is yellow, false is blue.
bool us_is_yellow;

// Wether to use the legacy SSL protobuf packets.
bool use_legacy_packets;


/**
 * Sets the `our_color` parameter to the default "yellow".
 * Call if the parameter hasn't been set yet, or was set to garbage.
 */
void default_our_color_parameter() {
    // Default to yellow.
    us_is_yellow = true;
    ros::param::set("our_color", "yellow");
}


/**
 * Reads in the `our_color` parameter.
 * If the parameter is not set, defaults to yellow.
 */
void read_our_color_parameter() {
    std::string our_color;
    if (ros::param::get("our_color", our_color)) {
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
 * Sets the `our_field_side` parameter to the default `right`.
 */
void default_our_field_side_parameter() {
    ros::param::set("our_field_side", "right");
}


/**
 * Reads in the `our_field_side` parameter.
 * If the parameter is not set, defaults to right.
 */
void read_our_field_side_parameter() {
    std::string our_field_side;
    if (ros::param::get("our_field_side", our_field_side)) {
        if (our_field_side == "left") {

        } else if (our_field_side == "right") {

        } else {
            default_our_field_side_parameter();
        }
    } else {
        default_our_field_side_parameter();
    }
}


/**
 * Sets the `use_legacy_packets` parameter to the default `false`.
 */
void default_use_legacy_packets_parameter() {
    // Default to false.
    ROS_INFO("===========================================");
    ROS_INFO("= BEWARE! USING LEGACY PACKETS BY DEFAULT =");
    ROS_INFO("===========================================");
    use_legacy_packets = true;
    ros::param::set("use_legacy_packets", use_legacy_packets);
}


/**
 * Reads in the `use_legacy_packets` parameter.
 * If the parameter is not set, calls `default_use_legacy_packets_parameter`.
 */
void read_use_legacy_packets_parameter() {
    bool legacy;
    if (ros::param::get("use_legacy_packets", legacy)) {
        // This can probably be done more concise.
        // I just copied this from `read_our_color_parameter`.
        if (legacy == true) {
            use_legacy_packets = true;
        } else if (legacy == false) {
            use_legacy_packets = false;
        } else {
            default_use_legacy_packets_parameter();
        }
    } else {
        default_use_legacy_packets_parameter();
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
    read_use_legacy_packets_parameter();
    read_our_field_side_parameter();

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
            roboteam_msgs::DetectionFrame frame = rtt::convert_detection_frame(detectionFrame, us_is_yellow);

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

    // Create the publishers.
    // The `true` means that the messages will be latched.
    // When a new node subscribes, it will automatically get the latest message of the topic,
    // even if that was published a minute ago.
    ros::Publisher detection_pub = n.advertise<roboteam_msgs::DetectionFrame>("vision_detection", 1000, true);
    ros::Publisher geometry_pub = n.advertise<roboteam_msgs::GeometryData>("vision_geometry", 1000, true);
    ros::Publisher refbox_pub = n.advertise<roboteam_msgs::RefereeData>("vision_refbox", 1000, true);

    // Add the service to reset the last frame trackers.
    ros::ServiceServer reset_service = n.advertiseService("vision_reset", vision_reset);


    RoboCupSSLClient vision_client = RoboCupSSLClient(10006, "224.5.23.2");
    RoboCupSSLClient refbox_client = RoboCupSSLClient(10003, "224.5.23.1");;

    // Open the clients, blocking = false.
    vision_client.open(false);
    refbox_client.open(false);

    // Read the parameters.
    read_our_color_parameter();
    read_use_legacy_packets_parameter();
    read_our_field_side_parameter();

    ROS_INFO("Vision ready.");

    SSL_WrapperPacket vision_packet;
    RoboCup2014Legacy::Wrapper::SSL_WrapperPacket vision_packet_legacy;
    SSL_Referee refbox_packet;

    while (ros::ok()) {
        if (use_legacy_packets) {

            // Receive legacy packets.
            while (vision_client.receive(vision_packet_legacy)) {

                // Detection frame.
                if (vision_packet_legacy.has_detection()) {
                    send_detection_frame(vision_packet_legacy.detection(), detection_pub);
                }

                if (vision_packet_legacy.has_geometry()) {
                    // Convert the geometry frame.
                    roboteam_msgs::GeometryData data = rtt::legacy::convert_geometry_data(vision_packet_legacy.geometry());

                    // Publish the data.
                    geometry_pub.publish(data);
                }
            }

        } else {

            // Receive current version packets.
            while (vision_client.receive(vision_packet)) {

                // Detection frame.
                if (vision_packet.has_detection()) {
                    send_detection_frame(vision_packet.detection(), detection_pub);
                }

                if (vision_packet.has_geometry()) {
                    // Convert the geometry frame.
                    roboteam_msgs::GeometryData data = rtt::convert_geometry_data(vision_packet.geometry());

                    // Publish the data.
                    geometry_pub.publish(data);
                }
            }
        }

        while (refbox_client.receive(refbox_packet)) {
            // Convert the referee data.
            roboteam_msgs::RefereeData data = rtt::convert_referee_data(refbox_packet, us_is_yellow);

            // Publish the data.
            refbox_pub.publish(data);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
