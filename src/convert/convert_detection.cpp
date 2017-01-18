/**
 * Functions to convert SSL vision detection packets to the ROS format.
 * Works for legacy and current SSL vision formats.
 */

#include "roboteam_vision/convert/convert_detection.h"


namespace rtt {

    /**
     * Converts a protoBuf DetectionFrame to the ROS version.
     */
    roboteam_msgs::DetectionFrame convert_detection_frame(SSL_DetectionFrame protoFrame, bool us_is_yellow) {
        roboteam_msgs::DetectionFrame rosFrame;

        rosFrame.frame_number = protoFrame.frame_number();
        rosFrame.t_capture = protoFrame.t_capture();
        rosFrame.t_sent = protoFrame.t_sent();
        rosFrame.camera_id = protoFrame.camera_id();


        for (int i = 0; i < protoFrame.balls().size(); ++i) {
            SSL_DetectionBall protoBall = protoFrame.balls().Get(i);
            roboteam_msgs::DetectionBall rosBall = convert_detection_ball(protoBall);
            rosFrame.balls.push_back(rosBall);
        }

        for (int i = 0; i < protoFrame.robots_yellow().size(); ++i) {
            SSL_DetectionRobot protoBot = protoFrame.robots_yellow().Get(i);
            roboteam_msgs::DetectionRobot rosBot = convert_detection_robot(protoBot);

            if (us_is_yellow) {
                rosFrame.us.push_back(rosBot);
            } else {
                rosFrame.them.push_back(rosBot);
            }
        }

        for (int i = 0; i < protoFrame.robots_blue().size(); ++i) {
            SSL_DetectionRobot protoBot = protoFrame.robots_blue().Get(i);
            roboteam_msgs::DetectionRobot rosBot = convert_detection_robot(protoBot);

            if (us_is_yellow) {
                rosFrame.them.push_back(rosBot);
            } else {
                rosFrame.us.push_back(rosBot);
            }
        }

        return rosFrame;
    }


    /**
     * Converts a protoBuf DetectionBall to the ROS version.
     */
    roboteam_msgs::DetectionBall convert_detection_ball(SSL_DetectionBall protoBall) {
        roboteam_msgs::DetectionBall rosBall;

        rosBall.confidence = protoBall.confidence();
        rosBall.area = protoBall.area();
        rosBall.pos.x = mm_to_m(protoBall.x());
        rosBall.pos.y = mm_to_m(protoBall.y());
        rosBall.z = mm_to_m(protoBall.z());
        rosBall.pixel_pos.x = protoBall.pixel_x();
        rosBall.pixel_pos.x = protoBall.pixel_y();

        return rosBall;
    }


    /**
     * Converts a protoBuf DetectionRobot to the ROS version.
     */
    roboteam_msgs::DetectionRobot convert_detection_robot(SSL_DetectionRobot protoBot) {
        roboteam_msgs::DetectionRobot rosBot;

        rosBot.confidence = protoBot.confidence();
        rosBot.robot_id = protoBot.robot_id();
        rosBot.pos.x = mm_to_m(protoBot.x());
        rosBot.pos.y = mm_to_m(protoBot.y());
        rosBot.orientation = protoBot.orientation();
        rosBot.pixel_pos.x = protoBot.pixel_x();
        rosBot.pixel_pos.y = protoBot.pixel_y();
        rosBot.height = mm_to_m(protoBot.height());

        return rosBot;
    }

}
