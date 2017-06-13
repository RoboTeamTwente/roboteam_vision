#include <string>
#include <signal.h>
#include <chrono>

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/GeometryData.h"
#include "roboteam_msgs/RefereeData.h"

#include "net/robocup_ssl_client.h"
#include "roboteam_utils/messages_robocup_ssl_wrapper.pb.h"
#include "roboteam_utils/messages_robocup_ssl_wrapper_legacy.pb.h"

#include "roboteam_utils/normalize.h"
#include "roboteam_utils/constants.h"
#include "roboteam_utils/Vector2.h"

#include "roboteam_vision/convert/convert_detection.h"
#include "roboteam_vision/convert/convert_geometry_current.h"
#include "roboteam_vision/convert/convert_geometry_legacy.h"
#include "roboteam_vision/convert/convert_referee.h"
#include "roboteam_vision/transform.h"


// Keeps track of which team is us.
// True is yellow, false is blue.
bool us_is_yellow = true;
// Wether to use the legacy SSL protobuf packets.
bool use_legacy_packets = false;
bool ours_is_left = true;
bool normalize_field = true;

bool transform_field = false;
rtt::Vector2 transform_move = rtt::Vector2(0, 0);
rtt::Vector2 transform_scale = rtt::Vector2(1, 1);
bool transform_rotate_right_angle = false;



void update_parameters_from_ros() {
    if (rtt::has_PARAM_NORMALIZE_FIELD()) {
        rtt::get_PARAM_NORMALIZE_FIELD(normalize_field);
    } else {
        rtt::set_PARAM_NORMALIZE_FIELD(true);
    }

    if (rtt::has_PARAM_OUR_SIDE()) {
        std::string our_side;
        rtt::get_PARAM_OUR_SIDE(our_side);

        if (our_side == "left") {
            ours_is_left = true;
        } else if (our_side == "right") {
            ours_is_left = false;
        } else {
            rtt::set_PARAM_OUR_SIDE("left");
        }
    } else {
        rtt::set_PARAM_OUR_SIDE("left");
    }

    if (rtt::has_PARAM_OUR_COLOR()) {
        std::string our_color;
        rtt::get_PARAM_OUR_COLOR(our_color);

        if (our_color == "yellow") {
            us_is_yellow = true;
        } else if (our_color == "blue") {
            us_is_yellow = false;
        } else {
            rtt::set_PARAM_OUR_COLOR("yellow");
        }
    } else {
        rtt::set_PARAM_OUR_COLOR("yellow");
    }

    if (ros::param::has("use_legacy_packets")) {
        ros::param::get("use_legacy_packets", use_legacy_packets);
    } else {
        ros::param::set("use_legacy_packets", false);
    }


    // ---- Transformation parameters ----

    if (ros::param::has("transform_field/enabled")) {
        ros::param::get("transform_field/enabled", transform_field);

        float move_x = 0;
        ros::param::get("transform_field/move/x", move_x);
        float move_y = 0;
        ros::param::get("transform_field/move/y", move_y);
        float scale_x = 1;
        ros::param::get("transform_field/scale/x", scale_x);
        float scale_y = 1;
        ros::param::get("transform_field/scale/y", scale_y);

        ros::param::get("transform_field/rotate", transform_rotate_right_angle);

        transform_move.x = move_x;
        transform_move.y = move_y;
        transform_scale.x = scale_x;
        transform_scale.y = scale_y;
    }

    // ---- /Transformation parameters ----
}



namespace {

bool shuttingDown = false;

void sigIntHandler(int) {
    shuttingDown = true;
}

} // anonymous namespace

// Sends a SSL_DetectionFrame out through the supplied publisher.
void send_detection_frame(SSL_DetectionFrame detectionFrame, ros::Publisher publisher, bool normalize_field) {
    uint cam_id = detectionFrame.camera_id();

    // TODO: See if it is necessary to remove duplicate frames.
    // Are they even duplicate?

    // Convert the detection frame.
    roboteam_msgs::DetectionFrame frame = rtt::convert_detection_frame(detectionFrame, us_is_yellow);

    if (normalize_field) {
        frame = rtt::normalizeDetectionFrame(frame);
    }

    if (transform_field) {
        rtt::transformDetectionFrame(frame, transform_move, transform_rotate_right_angle);
    }

    // Publish the frame.
    publisher.publish(frame);
}

int main(int argc, char **argv) {

    std::vector<std::string> args(argv, argv + argc);

    // Init ros.
    ros::init(argc, argv, "roboteam_msgs", ros::init_options::NoSigintHandler); // What?
    // Flower power!
    signal(SIGINT, sigIntHandler);
    ros::NodeHandle n;

    // Run at 200 hz.
    ros::Rate loop_rate(60);

    // Create the publishers.
    // The `true` means that the messages will be latched.
    // When a new node subscribes, it will automatically get the latest message of the topic,
    // even if that was published a minute ago.
    ros::Publisher detection_pub = n.advertise<roboteam_msgs::DetectionFrame>("vision_detection", 1, true);
    ros::Publisher geometry_pub = n.advertise<roboteam_msgs::GeometryData>("vision_geometry", 1, true);
    ros::Publisher refbox_pub = n.advertise<roboteam_msgs::RefereeData>("vision_refbox", 1, true);


    RoboCupSSLClient vision_client = RoboCupSSLClient(10006, "224.5.23.2");
    RoboCupSSLClient refbox_client = RoboCupSSLClient(10003, "224.5.23.1");;

    // Open the clients, blocking = false.
    vision_client.open(false);
    refbox_client.open(false);

    std::chrono::high_resolution_clock::time_point last_parameter_update_time = std::chrono::high_resolution_clock::now();

    update_parameters_from_ros();

    ROS_INFO("Vision ready.");

    SSL_WrapperPacket vision_packet;
    RoboCup2014Legacy::Wrapper::SSL_WrapperPacket vision_packet_legacy;
    SSL_Referee refbox_packet;

    while (ros::ok() && !shuttingDown) {
        if (use_legacy_packets) {

            // Receive legacy packets.
            while (vision_client.receive(vision_packet_legacy)) {

                // Detection frame.
                if (vision_packet_legacy.has_detection()) {
                    send_detection_frame(vision_packet_legacy.detection(), detection_pub, normalize_field);
                }

                if (vision_packet_legacy.has_geometry()) {
                    // Convert the geometry frame.
                    roboteam_msgs::GeometryData data = rtt::legacy::convert_geometry_data(vision_packet_legacy.geometry());

                    if (normalize_field) {
                        data = rtt::normalizeGeometryData(data);
                    }

                    if (transform_field) {
                        rtt::scaleGeometryData(data, transform_scale);
                    }

                    // Publish the data.
                    geometry_pub.publish(data);
                }
            }

        } else {

            // Receive current version packets.
            while (vision_client.receive(vision_packet)) {

                // Detection frame.
                if (vision_packet.has_detection()) {
                    send_detection_frame(vision_packet.detection(), detection_pub, normalize_field);
                }

                if (vision_packet.has_geometry()) {
                    // Convert the geometry frame.
                    roboteam_msgs::GeometryData data = rtt::convert_geometry_data(vision_packet.geometry());

                    if (normalize_field) {
                        data = rtt::normalizeGeometryData(data);
                    }

                    if (transform_field) {
                        rtt::scaleGeometryData(data, transform_scale);
                    }

                    // Publish the data.
                    geometry_pub.publish(data);
                }
            }
        }

        while (refbox_client.receive(refbox_packet)) {
            // Convert the referee data.
            roboteam_msgs::RefereeData data = rtt::convert_referee_data(refbox_packet, us_is_yellow);

            if (normalize_field) {
                data = rtt::normalizeRefereeData(data);
            }

            if (transform_field) {
                rtt::transformRefereeData(data, transform_move, transform_rotate_right_angle);
            }

            // Publish the data.
            refbox_pub.publish(data);
        }

        // ---- Param updates ----

        auto time_now = std::chrono::high_resolution_clock::now();
        auto time_diff = time_now - last_parameter_update_time;

        // Every second...
        if (std::chrono::duration_cast<std::chrono::milliseconds>(time_diff).count() > 1000) {
            last_parameter_update_time = time_now;

            update_parameters_from_ros();
        }

        // ---- /Param updates ----

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Destructors do not call close properly. Just to be sure we do.
    vision_client.close();
    refbox_client.close();

    // This is needed because we use our own SIGINT handler!
    // See crashtest.cpp for details
    ros::shutdown();

    return 0;
}
