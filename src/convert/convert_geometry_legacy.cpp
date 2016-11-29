/**
 * Functions to convert the legacy SSL geometry packets to the ROS format.
 */


#include "convert_geometry_legacy.h"

// Needed because the geometry camera calibration is the same accross both.
#include "convert_geometry_current.h"
#include "convert_units.h"


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

        // ---- Lines ----------------------------------------------------------

        // Top field border.
        rosSize.top_line = make_line(
            "top_line",
            -half_length,
            half_width,
            half_length,
            half_width,
            line_width
        );
        rosSize.field_lines.push_back(rosSize.top_line);

        // Bottom field border.
        rosSize.bottom_line = make_line(
            "bottom_line",
            -half_length,
            -half_width,
            half_length,
            -half_width,
            line_width
        );
        rosSize.field_lines.push_back(rosSize.bottom_line);

        // Left field border.
        rosSize.left_line = make_line(
            "left_line",
            -half_length,
            -half_width,
            -half_length,
            half_width,
            line_width
        );
        rosSize.field_lines.push_back(rosSize.left_line);

        // Right field border.
        rosSize.right_line = make_line(
            "right_line",
            half_length,
            -half_width,
            half_length,
            half_width,
            line_width
        );
        rosSize.field_lines.push_back(rosSize.right_line);

        // Vertical halfway line.
        rosSize.half_line = make_line(
            "half_line",
            0.0,
            -half_width,
            0.0,
            half_width,
            line_width
        );
        rosSize.field_lines.push_back(rosSize.half_line);

        // Horizontal halfway line.
        rosSize.center_line = make_line(
            "center_line",
            -half_length,
            0.0,
            half_length,
            0.0,
            line_width
        );
        rosSize.field_lines.push_back(rosSize.center_line);

        // Left penalty line.
        rosSize.left_penalty_line = make_line(
            "left_penalty_line",
            -half_length + defense_radius,
            -defense_stretch/2,
            -half_length + defense_radius,
            defense_stretch/2,
            line_width
        );
        rosSize.field_lines.push_back(rosSize.left_penalty_line);

        // Right penalty line.
        rosSize.right_penalty_line = make_line(
            "right_penalty_line",
            half_length - defense_radius,
            -defense_stretch/2,
            half_length - defense_radius,
            defense_stretch/2,
            line_width
        );
        rosSize.field_lines.push_back(rosSize.right_penalty_line);

        // ---- Arcs -----------------------------------------------------------

        // Left top penalty arc.
        rosSize.top_left_penalty_arc = make_arc(
            "top_left_penalty_arc",
            -half_length,
            defense_stretch/2,
            defense_radius,
            0.0,
            HALF_PI,
            line_width
        );
        rosSize.field_arcs.push_back(rosSize.top_left_penalty_arc);

        // Left bottom penalty arc.
        rosSize.bottom_left_penalty_arc = make_arc(
            "bottom_left_penalty_arc",
            -half_length,
            -defense_stretch/2,
            defense_radius,
            PI + HALF_PI,
            PI + PI,
            line_width
        );
        rosSize.field_arcs.push_back(rosSize.bottom_left_penalty_arc);

        // Right top penalty arc.
        rosSize.top_right_penalty_arc = make_arc(
            "top_right_penalty_arc",
            half_length,
            -defense_stretch/2,
            defense_radius,
            PI,
            PI + HALF_PI,
            line_width
        );
        rosSize.field_arcs.push_back(rosSize.top_right_penalty_arc);

        // Right bottom penalty arc.
        rosSize.bottom_right_penalty_arc = make_arc(
            "bottom_right_penalty_arc",
            half_length,
            defense_stretch/2,
            defense_radius,
            HALF_PI,
            PI,
            line_width
        );
        rosSize.field_arcs.push_back(rosSize.bottom_right_penalty_arc);

        // The center circle.
        rosSize.center_circle = make_arc(
            "center_circle",
            0.0,
            0.0,
            mm_to_m(protoSize.center_circle_radius()),
            0.0,
            PI + PI,
            line_width
        );
        rosSize.field_arcs.push_back(rosSize.center_circle);

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
        line.begin.x = x_begin;
        line.begin.y = y_begin;
        line.end.x = x_end;
        line.end.y = y_end;
        line.thickness = thickness;

        return line;
    }


    /**
     * Convenience function to create FieldArcs.
     */
    roboteam_msgs::FieldCircularArc make_arc(
        std::string name,
        float x_center,
        float y_center,
        float radius,
        float a1,
        float a2,
        float thickness)
    {
        roboteam_msgs::FieldCircularArc arc = roboteam_msgs::FieldCircularArc();

        arc.name = name;
        arc.center.x = x_center;
        arc.center.y = y_center;
        arc.radius = radius;
        arc.a1 = a1;
        arc.a2 = a2;
        arc.thickness = thickness;

        return arc;
    }

} // namespace legacy
} // namespace rtt
