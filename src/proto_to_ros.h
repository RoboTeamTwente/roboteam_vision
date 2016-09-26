
#pragma once

#include "net/robocup_ssl_client.h"

#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/GeometryData.h"


/**
 * Converts a protoBuf DetectionBall to the ROS version.
 */
roboteam_msgs::DetectionBall convert_detection_ball(SSL_DetectionBall protoBall) {
    roboteam_msgs::DetectionBall rosBall;

    rosBall.confidence = protoBall.confidence();
    rosBall.area = protoBall.area();
    rosBall.pos.x = protoBall.x();
    rosBall.pos.y = protoBall.y();
    rosBall.z = protoBall.z();
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
    rosBot.pos.x = protoBot.x();
    rosBot.pos.y = protoBot.y();
    rosBot.orientation = protoBot.orientation();
    rosBot.pixel_pos.x = protoBot.pixel_x();
    rosBot.pixel_pos.y = protoBot.pixel_y();
    rosBot.height = protoBot.height();

    return rosBot;
}


/**
 * Converts a protoBuf DetectionFrame to the ROS version.
 */
roboteam_msgs::DetectionFrame convert_detection_frame(SSL_DetectionFrame protoFrame) {
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
        rosFrame.robots_yellow.push_back(rosBot);
    }

    for (int i = 0; i < protoFrame.robots_blue().size(); ++i) {
        SSL_DetectionRobot protoBot = protoFrame.robots_blue().Get(i);
        roboteam_msgs::DetectionRobot rosBot = convert_detection_robot(protoBot);
        rosFrame.robots_blue.push_back(rosBot);
    }

    return rosFrame;
}


/**
 * Converts a protoBuf FieldLineSegment to the ROS version.
 */
roboteam_msgs::FieldLineSegment convert_geometry_field_line_segment(SSL_FieldLineSegment protoLine) {
    roboteam_msgs::FieldLineSegment rosLine;

    rosLine.name = protoLine.name();
    rosLine.x_begin = protoLine.p1().x();
    rosLine.y_begin = protoLine.p1().y();
    rosLine.x_end = protoLine.p2().x();
    rosLine.y_end = protoLine.p2().y();
    rosLine.thickness = protoLine.thickness();

    return rosLine;
}


/**
 * Converts a protoBuf FieldCicularArc to the ROS version.
 */
roboteam_msgs::FieldCicularArc convert_geometry_field_Cicular_arc(SSL_FieldCicularArc protoArc) {
    roboteam_msgs::FieldCicularArc rosArc;

    rosArc.name = protoArc.name();
    rosArc.x_center = protoArc.center().x();
    rosArc.y_center = protoArc.center().y();
    rosArc.radius = protoArc.radius();
    rosArc.a1 = protoArc.a1();
    rosArc.a2 = protoArc.a2();
    rosArc.thickness = protoArc.thickness();

    return rosArc;
}


/**
 * Converts a protoBuf GeometryFieldSize to the ROS version.
 */
roboteam_msgs::GeometryFieldSize convert_geometry_field_size(SSL_GeometryFieldSize protoSize) {
    roboteam_msgs::GeometryFieldSize rosSize;

    rosSize.field_length = protoSize.field_length();
    rosSize.field_width = protoSize.field_width();
    rosSize.goal_width = protoSize.goal_width();
    rosSize.goal_depth = protoSize.goal_depth();
    rosSize.boundary_width = protoSize.boundary_width();

    for (int i = 0; i < protoSize.field_lines_size(); ++i) {
        SSL_FieldLineSegment protoLine = protoSize.field_lines().Get(i);
        roboteam_msgs::FieldLineSegment rosLine = convert_geometry_field_line_segment(protoLine);
        rosSize.field_lines.push_back(rosLine);
    }

    for (int i = 0; i < protoSize.field_arcs_size(); ++i) {
        SSL_FieldCicularArc protoArc = protoSize.field_arcs().Get(i);
        roboteam_msgs::FieldCicularArc rosArc = convert_geometry_field_Cicular_arc(protoArc);
        rosSize.field_arcs.push_back(rosArc);
    }

    return rosSize;
}


/**
 * Converts a protoBuf GeometryCameraCalibration to the ROS version.
 */
roboteam_msgs::GeometryCameraCalibration convert_geometry_camera_calibration(SSL_GeometryCameraCalibration protoCal) {
    roboteam_msgs::GeometryCameraCalibration rosCal;

    return rosCal;
}


/**
 * Converts a protoBuf GeometryData to the ROS version.
 */
roboteam_msgs::GeometryData convert_geometry_data(SSL_GeometryData protoData) {
    roboteam_msgs::GeometryData rosData;

    rosData.field = convert_geometry_field_size(protoData.field());

    for (int i = 0; i < protoData.calib_size(); ++i) {
        SSL_GeometryCameraCalibration protoCal = protoData.calib().Get(i);
        roboteam_msgs::GeometryCameraCalibration rosCal = convert_geometry_camera_calibration(protoCal);
        rosData.calib.push_back(rosCal);
    }

    return rosData;
}
