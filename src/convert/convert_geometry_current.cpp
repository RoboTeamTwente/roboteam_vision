/**
 * Functions to convert the current SSL geometry packets to the ROS format.
 */

#include "roboteam_vision/convert/convert_geometry_current.h"


namespace rtt {
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


    /**
     * Converts a protoBuf GeometryCameraCalibration to the ROS version.
     */
    roboteam_msgs::GeometryCameraCalibration convert_geometry_camera_calibration(SSL_GeometryCameraCalibration protoCal) {
        roboteam_msgs::GeometryCameraCalibration rosCal;

        rosCal.camera_id = protoCal.camera_id();
        rosCal.focal_length = mm_to_m(protoCal.focal_length());
        rosCal.principal_point_x = mm_to_m(protoCal.principal_point_x());
        rosCal.principal_point_y = mm_to_m(protoCal.principal_point_y());
        rosCal.distortion = protoCal.distortion();
        rosCal.q0 = protoCal.q0();
        rosCal.q1 = protoCal.q1();
        rosCal.q2 = protoCal.q2();
        rosCal.q3 = protoCal.q3();
        rosCal.tx = mm_to_m(protoCal.tx());
        rosCal.ty = mm_to_m(protoCal.ty());
        rosCal.tz = mm_to_m(protoCal.tz());
        rosCal.derived_camera_world_tx = mm_to_m(protoCal.derived_camera_world_tx());
        rosCal.derived_camera_world_ty = mm_to_m(protoCal.derived_camera_world_ty());
        rosCal.derived_camera_world_tz = mm_to_m(protoCal.derived_camera_world_tz());

        return rosCal;
    }


    /**
     * Converts a protoBuf GeometryFieldSize to the ROS version.
     */
    roboteam_msgs::GeometryFieldSize convert_geometry_field_size(SSL_GeometryFieldSize protoSize) {
        roboteam_msgs::GeometryFieldSize rosSize;

        rosSize.field_length = mm_to_m(protoSize.field_length());
        rosSize.field_width = mm_to_m(protoSize.field_width());
        rosSize.goal_width = mm_to_m(protoSize.goal_width());
        rosSize.goal_depth = mm_to_m(protoSize.goal_depth());
        rosSize.boundary_width = mm_to_m(protoSize.boundary_width());

        for (int i = 0; i < protoSize.field_lines_size(); ++i) {
            SSL_FieldLineSegment protoLine = protoSize.field_lines(i);
            roboteam_msgs::FieldLineSegment rosLine = convert_geometry_field_line_segment(protoLine);

            rosSize.field_lines.push_back(rosLine);

            if (rosLine.name == "top_line") {
                rosSize.top_line = rosLine;
            } else if (rosLine.name == "bottom_line") {
                rosSize.bottom_line = rosLine;
            } else if (rosLine.name == "left_line") {
                rosSize.left_line = rosLine;
            } else if (rosLine.name == "right_line") {
                rosSize.right_line = rosLine;
            } else if (rosLine.name == "half_line") {
                rosSize.half_line = rosLine;
            } else if (rosLine.name == "center_line") {
                rosSize.center_line = rosLine;
            } else if (rosLine.name == "left_penalty_line") {
                rosSize.left_penalty_line = rosLine;
            } else if (rosLine.name == "right_penalty_line") {
                rosSize.right_penalty_line = rosLine;
            }
        }

        for (int i = 0; i < protoSize.field_arcs_size(); ++i) {
            SSL_FieldCicularArc protoArc = protoSize.field_arcs(i);
            roboteam_msgs::FieldCircularArc rosArc = convert_geometry_field_Circular_arc(protoArc);

            rosSize.field_arcs.push_back(rosArc);

            if (rosArc.name == "top_left_penalty_arc") {
                rosSize.top_left_penalty_arc = rosArc;
            } else if (rosArc.name == "bottom_left_penalty_arc") {
                rosSize.bottom_left_penalty_arc = rosArc;
            } else if (rosArc.name == "top_right_penalty_arc") {
                rosSize.top_right_penalty_arc = rosArc;
            } else if (rosArc.name == "bottom_right_penalty_arc") {
                rosSize.bottom_right_penalty_arc = rosArc;
            } else if (rosArc.name == "center_circle") {
                rosSize.center_circle = rosArc;
            }
        }

        return rosSize;
    }


    /**
     * Converts a protoBuf FieldLineSegment to the ROS version.
     */
    roboteam_msgs::FieldLineSegment convert_geometry_field_line_segment(SSL_FieldLineSegment protoLine) {
        roboteam_msgs::FieldLineSegment rosLine;

        rosLine.name = std::string(name_map[protoLine.name()]);
        rosLine.begin.x = mm_to_m(protoLine.p1().x());
        rosLine.begin.y = mm_to_m(protoLine.p1().y());
        rosLine.end.x = mm_to_m(protoLine.p2().x());
        rosLine.end.y = mm_to_m(protoLine.p2().y());
        rosLine.thickness = mm_to_m(protoLine.thickness());

        return rosLine;
    }


    /**
     * Converts a protoBuf FieldCircularArc to the ROS version.
     */
    roboteam_msgs::FieldCircularArc convert_geometry_field_Circular_arc(SSL_FieldCicularArc protoArc) {
        roboteam_msgs::FieldCircularArc rosArc;

        rosArc.name = std::string(name_map[protoArc.name()]);
        rosArc.center.x = mm_to_m(protoArc.center().x());
        rosArc.center.y = mm_to_m(protoArc.center().y());
        rosArc.radius = mm_to_m(protoArc.radius());
        rosArc.a1 = protoArc.a1();
        rosArc.a2 = protoArc.a2();
        rosArc.thickness = mm_to_m(protoArc.thickness());

        return rosArc;
    }

}
