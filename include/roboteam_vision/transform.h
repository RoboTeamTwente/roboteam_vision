#pragma once

#include <roboteam_msgs/DetectionFrame.h>
#include <roboteam_msgs/RobotCommand.h>
#include <roboteam_msgs/GeometryData.h>
#include <roboteam_msgs/RefereeData.h>
#include <roboteam_utils/Vector2.h>

namespace rtt {

using namespace roboteam_msgs;

/*
 * The functions here can be used to make the field appear differently
 * to the ai than it is in real live.
 * This is done by scaling the geometry data and moving and rotating
 * the detection data.
 * The split between scaling and the two other transformations is to
 * make sure the field the ai sees is always horizontal and centered
 * around (0,0)
 */

void dropBallsOutsideTransform(DetectionFrame& frame, Vector2 const & field_size, double top, double right, double bottom, double left);

void scaleGeometryData(GeometryData& data, rtt::Vector2 scale);

void scaleLine(FieldLineSegment& line, rtt::Vector2& scale);

void scaleArc(FieldCircularArc& arc, rtt::Vector2& scale);

void transformDetectionFrame(DetectionFrame& frame, rtt::Vector2& move, bool& rotate);

void transformRefereeData(RefereeData& data, rtt::Vector2& move, bool& rotate);

void transformRobot(DetectionRobot& bot, rtt::Vector2& move, bool& rotate);

void transformVector(roboteam_msgs::Vector2f& vec, rtt::Vector2& move, bool& rotate);

}
