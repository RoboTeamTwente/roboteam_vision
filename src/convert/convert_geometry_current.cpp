/**
 * Functions to convert the current SSL geometry packets to the ROS format.
 */

#include "convert_geometry_current.h"


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
     * TODO: Actually implement this one.
     */
    roboteam_msgs::GeometryCameraCalibration convert_geometry_camera_calibration(SSL_GeometryCameraCalibration protoCal) {
        roboteam_msgs::GeometryCameraCalibration rosCal;

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
        }

        for (int i = 0; i < protoSize.field_arcs_size(); ++i) {
            SSL_FieldCicularArc protoArc = protoSize.field_arcs(i);
            roboteam_msgs::FieldCicularArc rosArc = convert_geometry_field_Cicular_arc(protoArc);
            rosSize.field_arcs.push_back(rosArc);
        }

        return rosSize;
    }


    /**
     * Converts a protoBuf FieldLineSegment to the ROS version.
     */
    roboteam_msgs::FieldLineSegment convert_geometry_field_line_segment(SSL_FieldLineSegment protoLine) {
        roboteam_msgs::FieldLineSegment rosLine;

        rosLine.name = protoLine.name();
        rosLine.x_begin = mm_to_m(protoLine.p1().x());
        rosLine.y_begin = mm_to_m(protoLine.p1().y());
        rosLine.x_end = mm_to_m(protoLine.p2().x());
        rosLine.y_end = mm_to_m(protoLine.p2().y());
        rosLine.thickness = mm_to_m(protoLine.thickness());

        return rosLine;
    }


    /**
     * Converts a protoBuf FieldCicularArc to the ROS version.
     */
    roboteam_msgs::FieldCicularArc convert_geometry_field_Cicular_arc(SSL_FieldCicularArc protoArc) {
        roboteam_msgs::FieldCicularArc rosArc;

        rosArc.name = protoArc.name();
        rosArc.x_center = mm_to_m(protoArc.center().x());
        rosArc.y_center = mm_to_m(protoArc.center().y());
        rosArc.radius = mm_to_m(protoArc.radius());
        rosArc.a1 = protoArc.a1();
        rosArc.a2 = protoArc.a2();
        rosArc.thickness = mm_to_m(protoArc.thickness());

        return rosArc;
    }

}
