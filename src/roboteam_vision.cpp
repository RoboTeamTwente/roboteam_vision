#include <string>
#include <signal.h>
#include <chrono>
#include <memory>
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/GeometryData.h"
#include "roboteam_msgs/RefereeData.h"
#include "roboteam_msgs/GameEvent.h"

#include "net/robocup_ssl_client.h"
#include "roboteam_utils/messages_robocup_ssl_wrapper.pb.h"
#include "roboteam_utils/messages_robocup_ssl_wrapper_legacy.pb.h"

#include "roboteam_utils/normalize.h"
#include "roboteam_utils/constants.h"
#include "roboteam_utils/Vector2.h"

#include "roboteam_vision/roboteam_vision.h"
#include "roboteam_vision/convert/convert_detection.h"
#include "roboteam_vision/convert/convert_geometry_current.h"
#include "roboteam_vision/convert/convert_geometry_legacy.h"
#include "roboteam_vision/convert/convert_referee.h"
#include "roboteam_vision/transform.h"

constexpr int DEFAULT_VISION_PORT = 10006;
constexpr int DEFAULT_REFEREE_PORT =  10003;

const std::string SSL_VISION_SOURCE_IP = "224.5.23.2";
const std::string SSL_REFEREE_SOURCE_IP= "224.5.23.1";

const std::string LOCAL_SOURCE_IP = "127.0.0.1";

std::string VISION_SOURCE_IP  = LOCAL_SOURCE_IP;
std::string REFEREE_SOURCE_IP = SSL_REFEREE_SOURCE_IP;

// const std::string VISION_SOURCE_IP = SSL_VISION_SOURCE_IP;
// const std::string REFEREE_SOURCE_IP = SSL_REFEREE_SOURCE_IP;

// Our name as specified by ssl-refbox : https://github.com/RoboCup-SSL/ssl-refbox/blob/master/referee.conf
const std::string ROBOTEAM_TWENTE = "RoboTeam Twente";

// Keeps track of which team is us.
// True is yellow, false is blue.
bool us_is_yellow = true;
// Wether to use the legacy SSL protobuf packets.
bool use_legacy_packets = false;
bool our_side_is_left = true;
bool normalize_field = true;

// Field size used for calculating the transformation scale.
// Is updated when a GeometryData packet arrives.
rtt::Vector2 field_size = rtt::Vector2(9, 12);  // Does this thing even do anything?

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

    // Check the ROS parameters for which side we are on
    if (rtt::has_PARAM_OUR_SIDE()) {
        std::string our_side;
        rtt::get_PARAM_OUR_SIDE(our_side);

        if (our_side == "left") {
            if(!our_side_is_left) ROS_INFO_STREAM("Changed side to left");
            our_side_is_left = true;
        } else if (our_side == "right") {
            if(our_side_is_left) ROS_INFO_STREAM("Changed side to right");
            our_side_is_left = false;
        } else {
            ROS_WARN_STREAM("Parameter for side invalid : '" << our_side << "' ! Defaulting to left");
            rtt::set_PARAM_OUR_SIDE("left");
        }
    } else {
        ROS_WARN_STREAM("Parameter for side not set! Defaulting to left");
        rtt::set_PARAM_OUR_SIDE("left");
    }

    // Check the ROS parameters for which color we are
    if (rtt::has_PARAM_OUR_COLOR()) {
        std::string our_color;
        rtt::get_PARAM_OUR_COLOR(our_color);

        if (our_color == "yellow") {
            if(!us_is_yellow) ROS_INFO_STREAM("Changed color to yellow");
            us_is_yellow = true;
        } else if (our_color == "blue") {
            if(us_is_yellow) ROS_INFO_STREAM("Changed color to blue");
            us_is_yellow = false;
        } else {
            ROS_WARN_STREAM("Parameter for color invalid : '" << our_color << "' ! Defaulting to yellow");
            rtt::set_PARAM_OUR_COLOR("yellow");
        }
    } else {
        ROS_WARN_STREAM("Parameter for side not set! Defaulting to yellow");
        rtt::set_PARAM_OUR_COLOR("yellow");
    }

    // Check if we should use legacy packets
    if (ros::param::has("use_legacy_packets")) {
        bool _use_legacy_packets;
        ros::param::get("use_legacy_packets", _use_legacy_packets);
        if(_use_legacy_packets && !use_legacy_packets)
            ROS_INFO_STREAM("Switching to using legacy packets");
        use_legacy_packets = _use_legacy_packets;
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

    // Vision
    if (ros::param::has("vision_source_port")) {
        int currentPort;
        ros::param::get("vision_source_port", currentPort);
        if (lastKnownVisionPort && currentPort != *lastKnownVisionPort) {
            ROS_INFO_STREAM("vision port changed to " << currentPort);
            // Vision port changed; reset the client
            lastKnownVisionPort = currentPort;
            if (vision_client) {
                vision_client->close();
                delete vision_client.release();
            }
            vision_client = std::make_unique<RoboCupSSLClient>(currentPort, VISION_SOURCE_IP);
            vision_client->open(false);
        } else if (!lastKnownVisionPort) {
            lastKnownVisionPort = currentPort;
        }
    } else {
        ros::param::set("vision_source_port", DEFAULT_VISION_PORT);
    }

    // Referee
    if (ros::param::has("referee_source_port")) {
        int currentPort;
        ros::param::get("referee_source_port", currentPort);
        if (lastKnownRefereePort && currentPort != *lastKnownRefereePort) {
            ROS_INFO_STREAM("referee port changed to " << currentPort);
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

    // Init ros
    ros::init(argc, argv, "roboteam_msgs", ros::init_options::NoSigintHandler); // NoSigintHandler gives the ability to override ROS sigint handler
    // Flower power!
    signal(SIGINT, sigIntHandler);
    ros::NodeHandle n;

    // Run at 80 hz.
    int rate = 80;
    ros::Rate loop_rate(rate);
    ROS_INFO_STREAM("Running at " << rate << " Hz");

    // Create the publishers.
    // The `true` means that the messages will be latched.
    // When a new node subscribes, it will automatically get the latest message of the topic,
    // even if that was published a minute ago.
//    ros::Publisher detection_pub = n.advertise<roboteam_msgs::DetectionFrame>("vision_detection", 1000, true);
//    ros::Publisher geometry_pub = n.advertise<roboteam_msgs::GeometryData>("vision_geometry", 1000, true);
    ros::Publisher refbox_pub = n.advertise<roboteam_msgs::RefereeData>("vision_refbox", 1000, true);

    ROS_INFO("Publishing to 'vision_detection', 'vision_geometry', 'vision_refbox'");

    int initialVisionPort = DEFAULT_VISION_PORT;
    if (ros::param::has("vision_source_port")) {
        ros::param::get("vision_source_port", initialVisionPort);
    }

    int initialRefereePort = DEFAULT_REFEREE_PORT;
    if (ros::param::has("referee_source_port")) {
        ros::param::get("referee_source_port", initialRefereePort);
    }

    // If Serial, assume real game. If Grsim, assume local testing
    std::string robot_output_target = "grsim";
    if(ros::param::has("robot_output_target")){
        ros::param::get("robot_output_target", robot_output_target);
    }else{
        ROS_WARN("parameter 'robot_output_target' not specified. Assuming grsim");
    }

    // Set ip addresses accordingly
    if(robot_output_target == "serial"){
        VISION_SOURCE_IP = SSL_VISION_SOURCE_IP;
        REFEREE_SOURCE_IP = SSL_REFEREE_SOURCE_IP;
    }else
    if(robot_output_target == "grsim"){
        VISION_SOURCE_IP = LOCAL_SOURCE_IP;
        REFEREE_SOURCE_IP = LOCAL_SOURCE_IP;
    }else{
        ROS_WARN_STREAM("Parameter 'robot_output_target' has unknown value '" << robot_output_target << "'! Assuming grsim");
    }
    ROS_INFO_STREAM("Parameters set for '" << robot_output_target << "'");

    vision_client = std::make_unique<RoboCupSSLClient>(initialVisionPort, VISION_SOURCE_IP);
    refbox_client = std::make_unique<RoboCupSSLClient>(initialRefereePort, REFEREE_SOURCE_IP);

	ROS_INFO_STREAM("Vision  : " << VISION_SOURCE_IP  << ":" << initialVisionPort);
	ROS_INFO_STREAM("Referee : " << REFEREE_SOURCE_IP << ":" << initialRefereePort);

    if(VISION_SOURCE_IP != SSL_VISION_SOURCE_IP)
        ROS_WARN_STREAM("Watch out! Current VISION_SOURCE_IP will not work for the competition! Remember to set it to " << SSL_VISION_SOURCE_IP);
    if(REFEREE_SOURCE_IP != SSL_REFEREE_SOURCE_IP)
        ROS_WARN_STREAM("Watch out! Current REFEREE_SOURCE_IP will not work for the competition! Remember to set it to " << SSL_REFEREE_SOURCE_IP);

    // Open the clients, blocking = false.
//    vision_client->open(false);
    refbox_client->open(false);

    std::chrono::high_resolution_clock::time_point last_parameter_update_time = std::chrono::high_resolution_clock::now();

    update_parameters_from_ros();

    ROS_INFO("Vision ready");

    SSL_WrapperPacket vision_packet;
    RoboCup2014Legacy::Wrapper::SSL_WrapperPacket vision_packet_legacy;
    SSL_Referee refbox_packet;

    // int detectionPackets = 0;
    // using namespace std::chrono;
    // auto lastStatistics = std::chrono::high_resolution_clock::now();

//    int vision_packets_received = 0;
    int referee_packets_received = 0;
    int last_received_gameEvent = 0;

    while (ros::ok() && !shuttingDown) {
        // If the vision client is running
        if (vision_client) {
//            // Receive current version packets.
//            while (vision_client->receive(vision_packet)) {
//
//                // Detection frame.
//                if (vision_packet.has_detection()) {
//                    send_detection_frame(vision_packet.detection(), detection_pub, normalize_field);
//                    vision_packets_received++;
//                }
//
//                if (vision_packet.has_geometry()) {
//                    // Convert the geometry frame.
//                    roboteam_msgs::GeometryData data = rtt::convert_geometry_data(vision_packet.geometry());
//
//                    field_size.x = data.field.field_length;
//                    field_size.y = data.field.field_width;
//
//                    if (transform_field) {
//                        rtt::scaleGeometryData(data, transform_scale);
//                    }
//
//                    // Publish the data.
//                    geometry_pub.publish(data);
//                }
//            }
        } else {
            ROS_ERROR("roboteam_vision: Vision disconnected!");
//            if (latestFrame) {
//                    detection_pub.publish(*latestFrame);
//                }
//                if (latestGeom) {
//                    geometry_pub.publish(*latestGeom);
//                }
        }

        // If the referee client is running
        if (refbox_client) {
            while (refbox_client->receive(refbox_packet)) {
                bool shouldUpdateParameters = false;

                // Convert the referee data.
                roboteam_msgs::RefereeData data = rtt::convert_referee_data(refbox_packet, us_is_yellow);

                if(data.gameEvent.event != 0 && data.gameEvent.event != last_received_gameEvent){
                    last_received_gameEvent = data.gameEvent.event;
                    ROS_INFO_STREAM("Game event occured! event=" << gameEventToString(last_received_gameEvent).c_str() << ", message='" << data.gameEvent.message << "', team:bot=" << data.gameEvent.originator.team << ":" << data.gameEvent.originator.botId);
                }

                // === Check if our color has changed === //
                // If the name of our team is not "RoboTeam Twente", but the opponents team is, we are set to the wrong color
                if(data.us.name != ROBOTEAM_TWENTE && data.them.name == ROBOTEAM_TWENTE){
                    ROS_WARN_STREAM("The name of our team is not " << ROBOTEAM_TWENTE << " ! The name of their team is! Switching colors..");
                    // If we are yellow, switch to blue. If we are blue, switch to yellow.
                    rtt::set_PARAM_OUR_COLOR(us_is_yellow ? "blue" : "yellow");
                    shouldUpdateParameters = true;
                }else
                // If none of the teams are named "RoboTeam Twente"
                if(data.us.name != ROBOTEAM_TWENTE && data.them.name != ROBOTEAM_TWENTE){
                    ROS_DEBUG_STREAM_THROTTLE(1, "None of the teams are named " << ROBOTEAM_TWENTE);
                }

                // === Check if our side has changed === //
                // blueTeamOnPositiveHalf=true means "the blue team will have it's goal on the positive x-axis of the ssl-vision coordinate system" : https://github.com/RoboCup-SSL/ssl-refbox/blob/master/referee.proto
                if(our_side_is_left == (us_is_yellow ^ data.blueTeamOnPositiveHalf)){
                    ROS_WARN("We are playing on the wrong side! Switching sides..");
                    // If we are left, switch to right. If we are right, switch to left
                    rtt::set_PARAM_OUR_SIDE(our_side_is_left ? "right" : "left");
                    shouldUpdateParameters = true;
                }

                // Update the ros_parameters if we changed color or sides
                if(shouldUpdateParameters) {
                    update_parameters_from_ros();
                }

                if (transform_field) {
                    rtt::transformRefereeData(data, transform_move, transform_rotate_right_angle);
                }

                if (normalize_field && !our_side_is_left) {
                    data = rtt::normalizeRefereeData(data);
                }

                // Publish the data
                refbox_pub.publish(data);
                referee_packets_received++;
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
        int timeDifference = std::chrono::duration_cast<std::chrono::milliseconds>(time_diff).count();
        if (timeDifference > 10000) {
            last_parameter_update_time = time_now;

            update_parameters_from_ros();

//            ROS_INFO("Vision  packets Hz : %.2f", vision_packets_received  * (1000.0 / timeDifference));
            ROS_INFO("Referee packets Hz : %.2f", referee_packets_received * (1000.0 / timeDifference));
//            vision_packets_received = 0;
            referee_packets_received = 0;
        }

        // ---- /Param updates ----

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Shutting down vision..");
    // Destructors do not call close properly. Just to be sure we do.
//    vision_client->close();
    refbox_client->close();

    // This is needed because we use our own SIGINT handler!
    // See crashtest.cpp for details
    ros::shutdown();

    return 0;
}

std::string gameEventToString(int gameEvent){

    switch(gameEvent){
        case SSL_Referee_Game_Event_GameEventType_UNKNOWN : return "UNKNOWN";
        case SSL_Referee_Game_Event_GameEventType_CUSTOM : return "CUSTOM";
        case SSL_Referee_Game_Event_GameEventType_NUMBER_OF_PLAYERS : return "NUMBER_OF_PLAYERS";
        case SSL_Referee_Game_Event_GameEventType_BALL_LEFT_FIELD : return "BALL_LEFT_FIELD";
        case SSL_Referee_Game_Event_GameEventType_GOAL : return "GOAL";
        case SSL_Referee_Game_Event_GameEventType_KICK_TIMEOUT : return "KICK_TIMEOUT";
        case SSL_Referee_Game_Event_GameEventType_NO_PROGRESS_IN_GAME : return "NO_PROGRESS_IN_GAME";
        case SSL_Referee_Game_Event_GameEventType_BOT_COLLISION : return "BOT_COLLISION";
        case SSL_Referee_Game_Event_GameEventType_MULTIPLE_DEFENDER : return "MULTIPLE_DEFENDER";
        case SSL_Referee_Game_Event_GameEventType_MULTIPLE_DEFENDER_PARTIALLY : return "MULTIPLE_DEFENDER_PARTIALLY";
        case SSL_Referee_Game_Event_GameEventType_ATTACKER_IN_DEFENSE_AREA : return "ATTACKER_IN_DEFENSE_AREA";
        case SSL_Referee_Game_Event_GameEventType_ICING : return "ICING";
        case SSL_Referee_Game_Event_GameEventType_BALL_SPEED : return "BALL_SPEED";
        case SSL_Referee_Game_Event_GameEventType_ROBOT_STOP_SPEED : return "ROBOT_STOP_SPEED";
        case SSL_Referee_Game_Event_GameEventType_BALL_DRIBBLING : return "BALL_DRIBBLING";
        case SSL_Referee_Game_Event_GameEventType_ATTACKER_TOUCH_KEEPER : return "ATTACKER_TOUCH_KEEPER";
        case SSL_Referee_Game_Event_GameEventType_DOUBLE_TOUCH : return "DOUBLE_TOUCH";
        case SSL_Referee_Game_Event_GameEventType_ATTACKER_TO_DEFENCE_AREA : return "ATTACKER_TO_DEFENCE_AREA";
        case SSL_Referee_Game_Event_GameEventType_DEFENDER_TO_KICK_POINT_DISTANCE : return "DEFENDER_TO_KICK_POINT_DISTANCE";
        case SSL_Referee_Game_Event_GameEventType_BALL_HOLDING : return "BALL_HOLDING";
        case SSL_Referee_Game_Event_GameEventType_INDIRECT_GOAL : return "INDIRECT_GOAL";
        case SSL_Referee_Game_Event_GameEventType_BALL_PLACEMENT_FAILED : return "BALL_PLACEMENT_FAILED";
        case SSL_Referee_Game_Event_GameEventType_CHIP_ON_GOAL : return "CHIP_ON_GOAL";
        default : return "Unknown gameEvent!";
    }
}