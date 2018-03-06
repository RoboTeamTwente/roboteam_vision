#pragma once

/**
 * Functions to convert the current SSL geometry packets to the ROS format.
 */


#include <map>
#include <string>

#include "roboteam_utils/messages_robocup_ssl_geometry.pb.h"

#include "roboteam_msgs/GeometryData.h"

#include "convert_units.h"


namespace rtt {

    // Map that converts SSL line and arc names to the more clear RoboTeam ones.
    static std::map<std::string, std::string> name_map = {
        std::make_pair("TopTouchLine", "top_line"),
        std::make_pair("BottomTouchLine", "bottom_line"),
        std::make_pair("LeftGoalLine", "left_line"),
        std::make_pair("RightGoalLine", "right_line"),
        std::make_pair("HalfwayLine", "half_line"),
        std::make_pair("CenterLine", "center_line"),
        std::make_pair("LeftPenaltyStretch", "left_penalty_line"),
        std::make_pair("RightPenaltyStretch", "right_penalty_line"),

        std::make_pair("LeftFieldLeftPenaltyArc", "top_left_penalty_arc"),
        std::make_pair("LeftFieldRightPenaltyArc", "bottom_left_penalty_arc"),
        std::make_pair("RightFieldLeftPenaltyArc", "top_right_penalty_arc"),
        std::make_pair("RightFieldRightPenaltyArc", "bottom_right_penalty_arc"),
        
        std::make_pair("LeftFieldLeftPenaltyStretch", "top_left_penalty_stretch"),
        std::make_pair("LeftFieldRightPenaltyStretch", "bottom_left_penalty_stretch"),
        std::make_pair("RightFieldLeftPenaltyStretch", "bottom_right_penalty_stretch"),
        std::make_pair("RightFieldRightPenaltyStretch", "top_right_penalty_stretch"),
        
        std::make_pair("CenterCircle", "center_circle"),
    };

    /**
     * Converts a protoBuf GeometryData to the ROS version.
     */
    roboteam_msgs::GeometryData convert_geometry_data(SSL_GeometryData protoData);

    /**
     * Converts a protoBuf GeometryCameraCalibration to the ROS version.
     */
    roboteam_msgs::GeometryCameraCalibration convert_geometry_camera_calibration(SSL_GeometryCameraCalibration protoCal);

    /**
     * Converts a protoBuf GeometryFieldSize to the ROS version.
     */
    roboteam_msgs::GeometryFieldSize convert_geometry_field_size(SSL_GeometryFieldSize protoSize);

    /**
     * Converts a protoBuf FieldLineSegment to the ROS version.
     */
    roboteam_msgs::FieldLineSegment convert_geometry_field_line_segment(SSL_FieldLineSegment protoLine);


    /**
     * Converts a protoBuf FieldCircularArc to the ROS version.
     */
    roboteam_msgs::FieldCircularArc convert_geometry_field_Circular_arc(SSL_FieldCicularArc protoArc);
}
