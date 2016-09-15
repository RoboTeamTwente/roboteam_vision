#pragma once

#include "net/robocup_ssl_client.h"

#include "roboteam_vision/DetectionFrame.h"
#include "roboteam_vision/GeometryData.h"


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


/**
 * Converts a protoBuf Vector2f to the ROS version.
 */
roboteam_vision::Vector2f convert_vector2f(Vector2f protoVec) {
    roboteam_vision::Vector2f rosVec;

    rosVec.x = protoVec.x();
    rosVec.y = protoVec.y();

    return rosVec;
}


/**
 * Converts a protoBuf FieldLineSegment to the ROS version.
 */
roboteam_vision::FieldLineSegment convert_geometry_field_line_segment(SSL_FieldLineSegment protoLine) {
    roboteam_vision::FieldLineSegment rosLine;

    rosLine.name = protoLine.name();
    rosLine.p1 = convert_vector2f(protoLine.p1());
    rosLine.p2 = convert_vector2f(protoLine.p2());
    rosLine.thickness = protoLine.thickness();

    return rosLine;
}


/**
 * Converts a protoBuf FieldCicularArc to the ROS version.
 */
roboteam_vision::FieldCicularArc convert_geometry_field_Cicular_arc(SSL_FieldCicularArc protoArc) {
    roboteam_vision::FieldCicularArc rosArc;

    rosArc.name = protoArc.name();
    rosArc.center = convert_vector2f(protoArc.center());
    rosArc.radius = protoArc.radius();
    rosArc.a1 = protoArc.a1();
    rosArc.a2 = protoArc.a2();
    rosArc.thickness = protoArc.thickness();

    return rosArc;
}


/**
 * Converts a protoBuf GeometryFieldSize to the ROS version.
 */
roboteam_vision::GeometryFieldSize convert_geometry_field_size(SSL_GeometryFieldSize protoSize) {
    roboteam_vision::GeometryFieldSize rosSize;

    rosSize.field_length = protoSize.field_length();
    rosSize.field_width = protoSize.field_width();
    rosSize.goal_width = protoSize.goal_width();
    rosSize.goal_depth = protoSize.goal_depth();
    rosSize.boundary_width = protoSize.boundary_width();

    for (int i = 0; i < protoSize.field_lines_size(); ++i) {
        SSL_FieldLineSegment protoLine = protoSize.field_lines().Get(i);
        roboteam_vision::FieldLineSegment rosLine = convert_geometry_field_line_segment(protoLine);
        rosSize.field_lines.push_back(rosLine);
    }

    for (int i = 0; i < protoSize.field_arcs_size(); ++i) {
        SSL_FieldCicularArc protoArc = protoSize.field_arcs().Get(i);
        roboteam_vision::FieldCicularArc rosArc = convert_geometry_field_Cicular_arc(protoArc);
        rosSize.field_arcs.push_back(rosArc);
    }

    return rosSize;
}


/**
 * Converts a protoBuf GeometryCameraCalibration to the ROS version.
 */
roboteam_vision::GeometryCameraCalibration convert_geometry_camera_calibration(SSL_GeometryCameraCalibration protoCal) {
    roboteam_vision::GeometryCameraCalibration rosCal;

    return rosCal;
}


/**
 * Converts a protoBuf GeometryData to the ROS version.
 */
roboteam_vision::GeometryData convert_geometry_data(SSL_GeometryData protoData) {
    roboteam_vision::GeometryData rosData;

    rosData.field = convert_geometry_field_size(protoData.field());

    for (int i = 0; i < protoData.calib_size(); ++i) {
        SSL_GeometryCameraCalibration protoCal = protoData.calib().Get(i);
        roboteam_vision::GeometryCameraCalibration rosCal = convert_geometry_camera_calibration(protoCal);
        rosData.calib.push_back(rosCal);
    }

    return rosData;
}
