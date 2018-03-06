#include <algorithm>

#include "roboteam_vision/transform.h"

namespace rtt {

using namespace roboteam_msgs;

void scaleGeometryData(GeometryData& data, rtt::Vector2 scale) {
    auto& field = data.field;
    field.field_length *= scale.x;
    field.field_width *= scale.y;
    field.goal_width *= scale.y;
    field.goal_depth *= scale.x;
    field.boundary_width *= scale.x;

    scaleLine(field.top_line, scale);
    scaleLine(field.bottom_line, scale);
    scaleLine(field.left_line, scale);
    scaleLine(field.right_line, scale);
    scaleLine(field.half_line, scale);
    scaleLine(field.center_line, scale);
    scaleLine(field.left_penalty_line, scale);
    scaleLine(field.right_penalty_line, scale);

    scaleArc(field.top_left_penalty_arc, scale);
    scaleArc(field.bottom_left_penalty_arc, scale);
    scaleArc(field.top_right_penalty_arc, scale);
    scaleArc(field.bottom_right_penalty_arc, scale);

    // add rectangle box lines
    scaleLine(field.top_left_penalty_stretch, scale);
    scaleLine(field.bottom_left_penalty_stretch, scale);
    scaleLine(field.top_right_penalty_stretch, scale);
    scaleLine(field.bottom_right_penalty_stretch, scale);

    scaleArc(field.center_circle, scale);

    for (auto& line : field.field_lines) {
        scaleLine(line, scale);
    }

    for (auto& arc : field.field_arcs) {
        scaleArc(arc, scale);
    }
}


void scaleLine(FieldLineSegment& line, rtt::Vector2& scale) {
    line.begin.x *= scale.x;
    line.begin.y *= scale.y;
    line.end.x *= scale.x;
    line.end.y *= scale.y;

    line.thickness *= scale.x;
}

void scaleArc(FieldCircularArc& arc, rtt::Vector2& scale) {
    arc.center.x *= scale.x;
    arc.center.y *= scale.y;

    arc.radius *= scale.x;
    arc.thickness *= scale.x;
}

void dropObjectsOutsideTransform(DetectionFrame& frame, Vector2 const & field_size, double top, double right, double bottom, double left) {

    left   =  - ( field_size.x / 2 ) + left;
    right  =    ( field_size.x / 2 ) - right;
    top    =    ( field_size.y / 2 ) - top;
    bottom =  - ( field_size.y / 2 ) + bottom;

    auto inRect = [=](double x, double y) {
        return x < left 
            || x > right 
            || y > top 
            || y < bottom
            ;
    } ;

    auto & balls = frame.balls;

    balls.erase(std::remove_if(balls.begin(), balls.end(), [&](roboteam_msgs::DetectionBall const & ball) {
        return inRect(ball.pos.x, ball.pos.y);               
    }), balls.end());

    auto & us = frame.us;

    us.erase(std::remove_if(us.begin(), us.end(), [&](roboteam_msgs::DetectionRobot const & bot) {
        return inRect(bot.pos.x, bot.pos.y);
    }), us.end());
    
    auto & them = frame.them;

    them.erase(std::remove_if(them.begin(), them.end(), [&](roboteam_msgs::DetectionRobot const & bot) {
        return inRect(bot.pos.x, bot.pos.y);
    }), them.end());
}

void transformDetectionFrame(DetectionFrame& frame, rtt::Vector2& move, bool& rotate) {
    for (auto& ball : frame.balls) {
        transformVector(ball.pos, move, rotate);
    }

    for (auto& bot : frame.us) {
        transformRobot(bot, move, rotate);
    }

    for (auto& bot : frame.them) {
        transformRobot(bot, move, rotate);
    }
}

void transformRefereeData(RefereeData& data, rtt::Vector2& move, bool& rotate) {
    transformVector(data.designated_position, move, rotate);
}

void transformRobot(DetectionRobot& bot, rtt::Vector2& move, bool& rotate) {
    transformVector(bot.pos, move, rotate);
    if (rotate) {
        bot.orientation += M_PI/2;
    }
}

void transformVector(roboteam_msgs::Vector2f& vec, rtt::Vector2& move, bool& rotate) {

    if (rotate) {
        float temp_x = vec.x;
        vec.x = -vec.y;
        vec.y = temp_x;

        vec.x += -move.y;
        vec.y += move.x;
    } else {
        vec.x += move.x;
        vec.y += move.y;
    }
}

} // rtt
