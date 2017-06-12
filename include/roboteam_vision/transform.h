#pragma once

#include <roboteam_msgs/DetectionFrame.h>
#include <roboteam_msgs/RobotCommand.h>
#include <roboteam_msgs/GeometryData.h>
#include <roboteam_msgs/RefereeData.h>
#include <roboteam_utils/Vector2.h>

namespace rtt {

using namespace roboteam_msgs;

DetectionFrame transformDetectionFrame(DetectionFrame& world, rtt::Vector2 move, rtt::Vector2 scale, bool rotate);

GeometryData transformGeometryData(GeometryData& world, rtt::Vector2 move, rtt::Vector2 scale, bool rotate);

RefereeData transformRefereeData(RefereeData& world, rtt::Vector2 move, rtt::Vector2 scale, bool rotate);

}
