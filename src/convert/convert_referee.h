#pragma once

#include "messages_robocup_ssl_referee.pb.h"

#include "roboteam_msgs/RefereeData.h"


namespace rtt {
    /**
     * Converts protoBuf Referee messages to the ROS version.
     */
    roboteam_msgs::RefereeData convert_referee_data(SSL_Referee protoRef, bool us_is_yellow);

    /**
     * Converts protoBuf Referee states to the ROS version.
     */
    roboteam_msgs::RefereeStage convert_referee_stage(SSL_Referee_Stage protoStage);

    /**
     * Converts protoBuf Referee commands to the ROS version.
     */
    roboteam_msgs::RefereeCommand convert_referee_command(SSL_Referee_Command protoCommand, bool us_is_yellow);

    /**
     * Converts protoBuf TeamInfo to the ROS version.
     */
    roboteam_msgs::RefereeTeamInfo convert_referee_team_info(SSL_Referee_TeamInfo protoInfo);
}
