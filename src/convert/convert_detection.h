#pragma once

/**
 * Functions to convert SSL vision detection packets to the ROS format.
 * Works for legacy and current SSL vision formats.
 */


#include "messages_robocup_ssl_detection.pb.h"

#include "roboteam_msgs/DetectionFrame.h"

#include "convert_units.h"


namespace rtt {
    /**
     * Converts an SSL DetectionFrame to the ROS version.
     */
    roboteam_msgs::DetectionFrame convert_detection_frame(SSL_DetectionFrame protoFrame, bool us_is_yellow);


    /**
     * Converts a protoBuf DetectionBall to the ROS version.
     */
    roboteam_msgs::DetectionBall convert_detection_ball(SSL_DetectionBall protoBall);


    /**
     * Converts a protoBuf DetectionRobot to the ROS version.
     */
    roboteam_msgs::DetectionRobot convert_detection_robot(SSL_DetectionRobot protoBot);
}
