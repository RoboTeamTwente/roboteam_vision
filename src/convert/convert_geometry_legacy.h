#pragma once

/**
 * Functions to convert the legacy SSL geometry packets to the ROS format.
 */


#include "messages_robocup_ssl_geometry_legacy.pb.h"

#include "roboteam_msgs/GeometryData.h"

#include "convert_units.h"


namespace rtt {
namespace legacy {

    /**
     * Converts a protoBuf legacy GeometryData to the ROS version.
     */
    roboteam_msgs::GeometryData convert_geometry_data(RoboCup2014Legacy::Geometry::SSL_GeometryData protoData);

    /**
     * Converts a protoBuf GeometryFieldSize to the ROS version.
     */
    roboteam_msgs::GeometryFieldSize convert_geometry_field_size(RoboCup2014Legacy::Geometry::SSL_GeometryFieldSize protoSize);


    /**
     * Convenience function to create FieldLines.
     */
    roboteam_msgs::FieldLineSegment make_line(
        std::string name,
        float x_begin,
        float y_begin,
        float x_end,
        float y_end,
        float thickness);

} // namespace legacy
} // namespace rtt
