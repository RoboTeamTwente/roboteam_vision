#include <string>
#include <signal.h>

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

#include "roboteam_vision/convert/convert_detection.h"
#include "roboteam_vision/convert/convert_geometry_current.h"
#include "roboteam_vision/convert/convert_geometry_legacy.h"
#include "roboteam_vision/convert/convert_referee.h"


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
 * Sets the `our_side` parameter to the default `right`.
 */
void default_our_side_parameter() {
    ros::param::set("our_side", "right");
}


/**
 * Reads in the `our_side` parameter.
 * If the parameter is not set, defaults to right.
 */
void read_our_side_parameter() {
    std::string our_side;
    if (ros::param::get("our_side", our_side)) {
        if (our_side == "left") {

        } else if (our_side == "right") {

        } else {
            default_our_side_parameter();
        }
    } else {
        default_our_side_parameter();
    }
}


/**
 * Sets the `use_legacy_packets` parameter to the default `false`.
 */
void default_use_legacy_packets_parameter() {
    // Default to false.
    use_legacy_packets = false;
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
 * Resets the vision system.
 * To be called on `vision_reset`.
 *
 * Calls `read_our_color_parameter`.
 */
bool vision_reset(std_srvs::Empty::Request& req,
                  std_srvs::Empty::Response& res) {

    read_our_color_parameter();
    read_use_legacy_packets_parameter();
    read_our_side_parameter();

    return true;
}

namespace {

bool shuttingDown = false;

void sigIntHandler(int) {
    shuttingDown = true;
}

bool hasArg(std::vector<std::string> const & args, std::string const & arg) {
    return std::find(args.begin(), args.end(), arg) != args.end();
}

bool MERGING = false;

void swapBots3to6(roboteam_msgs::DetectionFrame & frame) {
    // @Hack

    auto isRobotPresent = [](std::vector<roboteam_msgs::DetectionRobot> const bots, unsigned int id) {
        for (auto const & bot : bots) {
            if (bot.robot_id == id) {
                return true;
            }
        }

        return false;
    };

    auto lookupBotIfPresent = [](std::vector<roboteam_msgs::DetectionRobot> const bots, unsigned int id) {
        for (auto const & bot : bots) {
            if (bot.robot_id == id) {
                return bot;
            }
        }

        return bots[0];
    };

    auto putBotIfPresent = [](std::vector<roboteam_msgs::DetectionRobot> & bots, roboteam_msgs::DetectionRobot const & bot, unsigned int id) {
        for (auto & otherBot : bots) {
            if (otherBot.robot_id == id) {
                otherBot = bot;
                otherBot.robot_id = id;
                return;
            }
        }
    };

    std::string our_color = "yellow";
    ros::param::get("our_color", our_color);

    std::vector<roboteam_msgs::DetectionRobot> blue;
    std::vector<roboteam_msgs::DetectionRobot> yellow;

    if (our_color == "yellow") {
        yellow = frame.us;
        blue = frame.them;
    } else {
        blue = frame.us;
        yellow = frame.them;
    }

    for (unsigned int i = 0; i < 3; ++i) {
        bool yellowPresent = isRobotPresent(yellow, i + 3);
        bool bluePresent = isRobotPresent(blue, i);

        if (yellowPresent && bluePresent) {
            auto robotYellow = lookupBotIfPresent(yellow, i + 3);
            auto robotBlue = lookupBotIfPresent(blue, i);

            putBotIfPresent(yellow, robotBlue, i + 3);
            putBotIfPresent(blue, robotYellow, i);
        }
    }

    if (our_color == "yellow") {
        frame.us = yellow;
        frame.them = blue;
    } else {
        frame.us = blue;
        frame.them = yellow;
    }
}

} // anonymous namespace

// Sends a SSL_DetectionFrame out through the supplied publisher.
void send_detection_frame(SSL_DetectionFrame detectionFrame, ros::Publisher publisher, bool should_normalize) {
    uint cam_id = detectionFrame.camera_id();

    // Check if we haven't already received this frame from this camera.
    // TODO: See if it is necessary to remove duplicate frames.
    // Are they even duplicate?

    // Convert the detection frame.
    roboteam_msgs::DetectionFrame frame = rtt::convert_detection_frame(detectionFrame, us_is_yellow);

    if (should_normalize) {
        frame = rtt::normalizeDetectionFrame(frame);
    }

    if (MERGING) {
        // Swap around robots 3-5 between yellow and blue
        swapBots3to6(frame);
    }

    // Publish the frame.
    publisher.publish(frame);
}

int main(int argc, char **argv) {

    std::vector<std::string> args(argv, argv + argc);

    MERGING = hasArg(args, "--merge-mode");

    if (MERGING) {
        std::cout << "[Vision] Merge mode active\n";
    }

    // Init ros.
    ros::init(argc, argv, "roboteam_msgs", ros::init_options::NoSigintHandler); // What?
    // Flower power!
    signal(SIGINT, sigIntHandler);
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

    ROS_INFO("Vision ready.");

    SSL_WrapperPacket vision_packet;
    RoboCup2014Legacy::Wrapper::SSL_WrapperPacket vision_packet_legacy;
    SSL_Referee refbox_packet;

    while (ros::ok() && !shuttingDown) {
        // Read the parameters.
        read_our_color_parameter();
        read_use_legacy_packets_parameter();
        read_our_side_parameter();

        // Do we need to normalize the frame?
        bool normalize_field = false;
        rtt::get_PARAM_NORMALIZE_FIELD(normalize_field);

        std::string our_side = "left";
        rtt::get_PARAM_OUR_SIDE(our_side);

        bool should_normalize = our_side == "right" && normalize_field;

        if (use_legacy_packets) {

            // Receive legacy packets.
            while (vision_client.receive(vision_packet_legacy)) {

                // Detection frame.
                if (vision_packet_legacy.has_detection()) {
                    send_detection_frame(vision_packet_legacy.detection(), detection_pub, should_normalize);
                }

                if (vision_packet_legacy.has_geometry()) {
                    // Convert the geometry frame.
                    roboteam_msgs::GeometryData data = rtt::legacy::convert_geometry_data(vision_packet_legacy.geometry());

                    if (should_normalize) {
                        data = rtt::normalizeGeometryData(data);
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
                    send_detection_frame(vision_packet.detection(), detection_pub, should_normalize);
                }

                if (vision_packet.has_geometry()) {
                    // Convert the geometry frame.
                    roboteam_msgs::GeometryData data = rtt::convert_geometry_data(vision_packet.geometry());

                    if (should_normalize) {
                        data = rtt::normalizeGeometryData(data);
                    }

                    // Publish the data.
                    geometry_pub.publish(data);
                }
            }
        }

        while (refbox_client.receive(refbox_packet)) {
            // Convert the referee data.
            roboteam_msgs::RefereeData data = rtt::convert_referee_data(refbox_packet, us_is_yellow);

            if (should_normalize) {
                data = rtt::normalizeRefereeData(data);
            }


            // Publish the data.
            refbox_pub.publish(data);
        }

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
