#pragma once

/**
 * Functions to convert the current SSL geometry packets to the ROS format.
 */


#include "messages_robocup_ssl_geometry.pb.h"

#include "roboteam_msgs/GeometryData.h"

#include "convert_units.h"


namespace rtt {

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
     * Converts a protoBuf FieldCicularArc to the ROS version.
     */
    roboteam_msgs::FieldCicularArc convert_geometry_field_Cicular_arc(SSL_FieldCicularArc protoArc);
}
