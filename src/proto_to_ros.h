#pragma once

#include "net/robocup_ssl_client.h"

#include "roboteam_vision/DetectionFrame.h"


/**
 * Converts a protoBuf DetectionBall to the ROS version.
 */
roboteam_vision::DetectionBall convert_detection_ball(SSL_DetectionBall protoBall) {
    roboteam_vision::DetectionBall rosBall;

    rosBall.confidence = protoBall.confidence();
    rosBall.area = protoBall.area();
    rosBall.x = protoBall.x();
    rosBall.y = protoBall.y();
    rosBall.z = protoBall.z();
    rosBall.pixel_x = protoBall.pixel_x();
    rosBall.pixel_y = protoBall.pixel_y();

    return rosBall;
}


/**
 * Converts a protoBuf DetectionRobot to the ROS version.
 */
roboteam_vision::DetectionRobot convert_detection_robot(SSL_DetectionRobot protoBot) {
    roboteam_vision::DetectionRobot rosBot;

    rosBot.confidence = protoBot.confidence();
    rosBot.robot_id = protoBot.robot_id();
    rosBot.x = protoBot.x();
    rosBot.y = protoBot.y();
    rosBot.orientation = protoBot.orientation();
    rosBot.pixel_x = protoBot.pixel_x();
    rosBot.pixel_y = protoBot.pixel_y();
    rosBot.height = protoBot.height();

    return rosBot;
}


/**
 * Converts a protoBuf DetectionFrame to the ROS version.
 */
roboteam_vision::DetectionFrame convert_detection_frame(SSL_DetectionFrame protoFrame) {
    roboteam_vision::DetectionFrame rosFrame;

    rosFrame.frame_number = protoFrame.frame_number();
    rosFrame.t_capture = protoFrame.t_capture();
    rosFrame.t_sent = protoFrame.t_sent();
    rosFrame.camera_id = protoFrame.camera_id();


    for (int i = 0; i < protoFrame.balls().size(); ++i) {
        SSL_DetectionBall protoBall = protoFrame.balls().Get(i);
        roboteam_vision::DetectionBall rosBall = convert_detection_ball(protoBall);
        rosFrame.balls.push_back(rosBall);
    }

    for (int i = 0; i < protoFrame.robots_yellow().size(); ++i) {
        SSL_DetectionRobot protoBot = protoFrame.robots_yellow().Get(i);
        roboteam_vision::DetectionRobot rosBot = convert_detection_robot(protoBot);
        rosFrame.robots_yellow.push_back(rosBot);
    }

    for (int i = 0; i < protoFrame.robots_blue().size(); ++i) {
        SSL_DetectionRobot protoBot = protoFrame.robots_blue().Get(i);
        roboteam_vision::DetectionRobot rosBot = convert_detection_robot(protoBot);
        rosFrame.robots_blue.push_back(rosBot);
    }

    return rosFrame;
}
