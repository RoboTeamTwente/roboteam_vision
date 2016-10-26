/**
 * Functions to convert the legacy SSL geometry packets to the ROS format.
 */


#include "convert_geometry_legacy.h"

// Needed because the geometry camera calibration is the same accross both.
#include "convert_geometry_current.h"


namespace rtt {
namespace legacy {

    /**
     * Converts a legacy protoBuf GeometryData to the ROS version.
     */
    roboteam_msgs::GeometryData convert_geometry_data(RoboCup2014Legacy::Geometry::SSL_GeometryData protoData) {
        roboteam_msgs::GeometryData rosData;

        rosData.field = legacy::convert_geometry_field_size(protoData.field());

        for (int i = 0; i < protoData.calib_size(); ++i) {
            SSL_GeometryCameraCalibration protoCal = protoData.calib().Get(i);
            // Calls the non-legacy `convert_geometry_camera_calibration` because it is the same as the legacy one.
            roboteam_msgs::GeometryCameraCalibration rosCal = convert_geometry_camera_calibration(protoCal);
            rosData.calib.push_back(rosCal);
        }

        return rosData;
    }

    /**
     * Converts a legacy GeometryFieldSize to the ROS version.
     */
    roboteam_msgs::GeometryFieldSize convert_geometry_field_size(RoboCup2014Legacy::Geometry::SSL_GeometryFieldSize protoSize) {
        roboteam_msgs::GeometryFieldSize rosSize;

        rosSize.field_length = mm_to_m(protoSize.field_length());
        rosSize.field_width = mm_to_m(protoSize.field_width());

        rosSize.goal_width = mm_to_m(protoSize.goal_width());
        rosSize.goal_depth = mm_to_m(protoSize.goal_depth());

        rosSize.boundary_width = mm_to_m(protoSize.boundary_width());

        float line_width = mm_to_m(protoSize.line_width());
        float half_length = rosSize.field_length/2;
        float half_width = rosSize.field_width/2;

        float defense_radius = mm_to_m(protoSize.defense_radius());
        float defense_stretch = mm_to_m(protoSize.defense_stretch());

        // Convert all the legacy values to the new line system.

        // Top field border.
        rosSize.field_lines.push_back(
            make_line(
                "TopTouchLine",
                -half_length,
                half_width,
                half_length,
                half_width,
                line_width
            )
        );

        // Bottom field border.
        rosSize.field_lines.push_back(
            make_line(
                "BottomTouchLine",
                -half_length,
                -half_width,
                half_length,
                -half_width,
                line_width
            )
        );

        // Left field border.
        rosSize.field_lines.push_back(
            make_line(
                "LeftGoalLine",
                -half_length,
                -half_width,
                -half_length,
                half_width,
                line_width
            )
        );

        // Right field border.
        rosSize.field_lines.push_back(
            make_line(
                "RightGoalLine",
                half_length,
                -half_width,
                half_length,
                half_width,
                line_width
            )
        );

        // Vertical halfway line.
        rosSize.field_lines.push_back(
            make_line(
                "HalfwayLine",
                0.0,
                -half_width,
                0.0,
                half_width,
                line_width
            )
        );

        // Horizontal halfway line.
        rosSize.field_lines.push_back(
            make_line(
                "CenterLine",
                -half_length,
                0.0,
                half_length,
                0.0,
                line_width
            )
        );

        // Left penalty line.
        rosSize.field_lines.push_back(
            make_line(
                "LeftPenaltyStretch",
                -half_length + defense_radius,
                -defense_stretch/2,
                -half_length + defense_radius,
                defense_stretch/2,
                line_width
            )
        );

        // Right penalty line.
        rosSize.field_lines.push_back(
            make_line(
                "RightPenaltyStretch",
                half_length - defense_radius,
                -defense_stretch/2,
                half_length - defense_radius,
                defense_stretch/2,
                line_width
            )
        );

        return rosSize;
    }


    /**
     * Convenience function to create FieldLines.
     */
    roboteam_msgs::FieldLineSegment make_line(
        std::string name,
        float x_begin,
        float y_begin,
        float x_end,
        float y_end,
        float thickness)
    {
        roboteam_msgs::FieldLineSegment line = roboteam_msgs::FieldLineSegment();

        line.name = name;
        line.x_begin = x_begin;
        line.y_begin = y_begin;
        line.x_end = x_end;
        line.y_end = y_end;
        line.thickness = thickness;

        return line;
    }

} // namespace legacy
} // namespace rtt
