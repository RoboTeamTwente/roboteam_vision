#include "roboteam_vision/transform.h"

namespace rtt {

using namespace roboteam_msgs;

DetectionFrame transformDetectionFrame(DetectionFrame& frame, rtt::Vector2 move, rtt::Vector2 scale, bool rotate) {
    DetectionFrame newFrame = frame;

    for (auto& ball : newFrame.balls) {
        ball = transformBall(ball, move, scale, rotate);
    }

    for (auto& bot : newFrame.us) {
        bot = transformRobot(bot, move, scale, rotate);
    }

    for (auto& bot : newFrame.them) {
        bot = transformRobot(bot, move, scale, rotate);
    }

    return newFrame;
}

GeometryData transformGeometryData(GeometryData& data, rtt::Vector2 move, rtt::Vector2 scale, bool rotate) {
    return data;
}

RefereeData transformRefereeData(RefereeData& data, rtt::Vector2 move, rtt::Vector2 scale, bool rotate) {
    return data;
}


DetectionBall transformBall(DetectionBall& ball, rtt::Vector2 move, rtt::Vector2 scale, bool rotate) {
    return ball;
}

DetectionRobot transformRobot(DetectionRobot& bot, rtt::Vector2 move, rtt::Vector2 scale, bool rotate) {
    return bot;
}


} // rtt
