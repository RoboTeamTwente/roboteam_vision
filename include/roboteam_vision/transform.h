#pragma once

#include <roboteam_msgs/DetectionFrame.h>
#include <roboteam_msgs/RobotCommand.h>
#include <roboteam_msgs/GeometryData.h>
#include <roboteam_msgs/RefereeData.h>
#include <roboteam_utils/Vector2.h>

namespace rtt {

using namespace roboteam_msgs;

DetectionFrame transformDetectionFrame(DetectionFrame& frame, rtt::Vector2 move, rtt::Vector2 scale, bool rotate);

GeometryData transformGeometryData(GeometryData& data, rtt::Vector2 move, rtt::Vector2 scale, bool rotate);

RefereeData transformRefereeData(RefereeData& data, rtt::Vector2 move, rtt::Vector2 scale, bool rotate);



DetectionBall transformBall(DetectionBall& ball, rtt::Vector2 move, rtt::Vector2 scale, bool rotate);

DetectionRobot transformRobot(DetectionRobot& bot, rtt::Vector2 move, rtt::Vector2 scale, bool rotate);

}
