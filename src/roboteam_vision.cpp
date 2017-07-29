#include <string>
#include <signal.h>
#include <chrono>
#include <memory>
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

constexpr int DEFAULT_VISION_PORT = 10036;
constexpr int DEFAULT_REFEREE_PORT = 10033;

// Keeps track of which team is us.
// True is yellow, false is blue.
bool us_is_yellow = true;
// Wether to use the legacy SSL protobuf packets.
bool use_legacy_packets = false;
bool our_side_is_left = true;
bool normalize_field = true;

// Field size used for calculating the transormation scale.
// Is updated when a GeometryData packet arrives.
rtt::Vector2 field_size = rtt::Vector2(6, 9);

bool transform_field = false;
rtt::Vector2 transform_move = rtt::Vector2(0, 0);
rtt::Vector2 transform_scale = rtt::Vector2(1, 1);
float transform_top = 0;
float transform_bottom = 0;
float transform_left = 0;
float transform_right = 0;

bool transform_rotate_right_angle = false;

std::unique_ptr<RoboCupSSLClient> vision_client;
std::unique_ptr<RoboCupSSLClient> refbox_client;
boost::optional<roboteam_msgs::RefereeData> latestReferee;
boost::optional<roboteam_msgs::GeometryData> latestGeom;
boost::optional<roboteam_msgs::DetectionFrame> latestFrame;

boost::optional<int> lastKnownVisionPort, lastKnownRefereePort;

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
            our_side_is_left = true;
        } else if (our_side == "right") {
            our_side_is_left = false;
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
        ros::param::get("transform_field/rotate", transform_rotate_right_angle);
	
        ros::param::get("transform_field/offset/top", transform_top);
        ros::param::get("transform_field/offset/bottom", transform_bottom);
        ros::param::get("transform_field/offset/left", transform_left);
        ros::param::get("transform_field/offset/right", transform_right);

        float new_width = field_size.x - (transform_left + transform_right);
        float new_height = field_size.y - (transform_top + transform_bottom);

        float relative_width = new_width / field_size.x;
        float relative_height = new_height / field_size.y;

        transform_move.x = (transform_right - transform_left) / 2;
        transform_move.y = (transform_top - transform_bottom) / 2;

        if (transform_rotate_right_angle) {
            transform_scale.x = new_height / field_size.x;
            transform_scale.y = new_width / field_size.y;
        } else {
            transform_scale.x = relative_width;
            transform_scale.y = relative_height;
        }
    }

    // ---- /Transformation parameters ----

    if (ros::param::has("vision_source_port")) {
        int currentPort;
        ros::param::get("vision_source_port", currentPort);
        if (lastKnownVisionPort && currentPort != *lastKnownVisionPort) {
            // Vision port changed; reset the client
            lastKnownVisionPort = currentPort;
            if (vision_client) {
                vision_client->close();
                delete vision_client.release();
            }
            vision_client = std::make_unique<RoboCupSSLClient>(currentPort, "224.5.23.2");
            vision_client->open(false);
        } else if (!lastKnownVisionPort) {
            lastKnownVisionPort = currentPort;
        }
    } else {
        ros::param::set("vision_source_port", DEFAULT_VISION_PORT);
    }

    if (ros::param::has("referee_source_port")) {
        int currentPort;
        ros::param::get("referee_source_port", currentPort);
        if (lastKnownRefereePort && currentPort != *lastKnownRefereePort) {
            // Referee port changed; reset the client
            lastKnownRefereePort = currentPort;
            if (refbox_client) {
                refbox_client->close();
                delete refbox_client.release();
            }
            refbox_client = std::make_unique<RoboCupSSLClient>(currentPort, "224.5.23.1");
            refbox_client->open(false);
        } else if (!lastKnownRefereePort) {
            lastKnownRefereePort = currentPort;
        }
    } else {
        ros::param::set("referee_source_port", DEFAULT_REFEREE_PORT);
    }
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

    if (transform_field) {
        rtt::dropObjectsOutsideTransform(
                frame, 
                field_size,
                transform_top, 
                transform_right,
                transform_bottom,
                transform_left
                );

        rtt::transformDetectionFrame(frame, transform_move, transform_rotate_right_angle);
    }

    if (normalize_field && !our_side_is_left) {
        frame = rtt::normalizeDetectionFrame(frame);
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
    ros::Rate loop_rate(80);

    // Create the publishers.
    // The `true` means that the messages will be latched.
    // When a new node subscribes, it will automatically get the latest message of the topic,
    // even if that was published a minute ago.
    ros::Publisher detection_pub = n.advertise<roboteam_msgs::DetectionFrame>("vision_detection", 1000, true);
    ros::Publisher geometry_pub = n.advertise<roboteam_msgs::GeometryData>("vision_geometry", 1000, true);
    ros::Publisher refbox_pub = n.advertise<roboteam_msgs::RefereeData>("vision_refbox", 1000, true);

    int initialVisionPort = DEFAULT_VISION_PORT;
    if (ros::param::has("vision_source_port")) {
        ros::param::get("vision_source_port", initialVisionPort);
    }

    int initialRefPort = DEFAULT_REFEREE_PORT;
    if (ros::param::has("referee_source_port")) {
        ros::param::get("referee_source_port", initialRefPort);
    }
    
    vision_client = std::make_unique<RoboCupSSLClient>(initialVisionPort, "224.5.23.2");
    refbox_client = std::make_unique<RoboCupSSLClient>(initialRefPort, "224.5.23.1");

    // vision_client = std::make_unique<RoboCupSSLClient>(initialVisionPort, "224.0.0.1");
    // refbox_client = std::make_unique<RoboCupSSLClient>(initialRefPort, "224.0.1.100");

    ROS_INFO("Starting vision client on port %d and refbox client on port %d.", initialVisionPort, initialRefPort);

    // Open the clients, blocking = false.
    vision_client->open(false);
    refbox_client->open(false);

    std::chrono::high_resolution_clock::time_point last_parameter_update_time = std::chrono::high_resolution_clock::now();

    update_parameters_from_ros();

    ROS_INFO("Vision ready.");

    SSL_WrapperPacket vision_packet;
    RoboCup2014Legacy::Wrapper::SSL_WrapperPacket vision_packet_legacy;
    SSL_Referee refbox_packet;

    // int detectionPackets = 0;
    // using namespace std::chrono;
    // auto lastStatistics = std::chrono::high_resolution_clock::now();

    while (ros::ok() && !shuttingDown) {
	if (vision_client) {
        if (use_legacy_packets) {

            // Receive legacy packets.
            while (vision_client->receive(vision_packet_legacy)) {

                // Detection frame.
                if (vision_packet_legacy.has_detection()) {
                    send_detection_frame(vision_packet_legacy.detection(), detection_pub, normalize_field);
                }

                if (vision_packet_legacy.has_geometry()) {
                    // Convert the geometry frame.
                    roboteam_msgs::GeometryData data = rtt::legacy::convert_geometry_data(vision_packet_legacy.geometry());

                    field_size.x = data.field.field_length;
                    field_size.y = data.field.field_width;

                    if (transform_field) {
                        rtt::scaleGeometryData(data, transform_scale);
                    }

                    // Not sure if this is needed - Bob
                    // if (normalize_field) {
                        // data = rtt::normalizeGeometryData(data);
                    // }

                    // Publish the data.
                    geometry_pub.publish(data);
                }
            }

        } else {

            // Receive current version packets.
            while (vision_client->receive(vision_packet)) {

                // Detection frame.
                if (vision_packet.has_detection()) {
                    send_detection_frame(vision_packet.detection(), detection_pub, normalize_field);
                }

                if (vision_packet.has_geometry()) {
                    // Convert the geometry frame.
                    roboteam_msgs::GeometryData data = rtt::convert_geometry_data(vision_packet.geometry());

                    field_size.x = data.field.field_length;
                    field_size.y = data.field.field_width;
                    
                    if (transform_field) {
                        rtt::scaleGeometryData(data, transform_scale);
                    }
                    // std::cout << field_size.x << std::endl;

                    // Not sure if this is needed either - Bob
                    // if (normalize_field) {
                        // data = rtt::normalizeGeometryData(data);
                    // }

                    // Publish the data.
                    geometry_pub.publish(data);
                }
            }
        }
	} else {
	    ROS_ERROR("roboteam_vision: Vision disconnected!");
	    if (latestFrame) {
                detection_pub.publish(*latestFrame);
            }
            if (latestGeom) {
                geometry_pub.publish(*latestGeom);
            }
	}

	if (refbox_client) {
        while (refbox_client->receive(refbox_packet)) {
            // Convert the referee data.
            roboteam_msgs::RefereeData data = rtt::convert_referee_data(refbox_packet, us_is_yellow);

            if (transform_field) {
                rtt::transformRefereeData(data, transform_move, transform_rotate_right_angle);
            }

            if (normalize_field && !our_side_is_left) {
                data = rtt::normalizeRefereeData(data);
            }

            // Publish the data.
            refbox_pub.publish(data);
        }
	} else {
	    ROS_WARN("roboteam_vision: Referee Disconnected!");
	    if (latestReferee) {
		refbox_pub.publish(*latestReferee);
            }
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

        // auto timeNow = high_resolution_clock::now();
        // auto timeDiff = timeNow - lastStatistics;

        // // Every second...
        // if (duration_cast<milliseconds>(timeDiff).count() > 1000) {
            // std::cout << "Detectionpackets: " << detectionPackets << "\n";
            // detectionPackets = 0;
            // lastStatistics = timeNow;
        // }
    }

    // Destructors do not call close properly. Just to be sure we do.
    vision_client->close();
    refbox_client->close();

    // This is needed because we use our own SIGINT handler!
    // See crashtest.cpp for details
    ros::shutdown();

    return 0;
}
