
#include "convert_referee.h"

#include "convert_units.h"


namespace rtt {

    /**
     * Converts protoBuf Referee messages to the ROS version.
     */
    roboteam_msgs::RefereeData convert_referee_data(SSL_Referee protoRef, bool us_is_yellow) {
        roboteam_msgs::RefereeData rosRef;

        rosRef.packet_timestamp = protoRef.packet_timestamp();

        rosRef.stage = convert_referee_stage(protoRef.stage());
        rosRef.stage_time_left = protoRef.stage_time_left();

        rosRef.command = convert_referee_command(protoRef.command(), us_is_yellow);
        rosRef.command_timestamp = protoRef.command_timestamp();

        if (us_is_yellow) {
            rosRef.us = convert_referee_team_info(protoRef.yellow());
            rosRef.them = convert_referee_team_info(protoRef.blue());
        } else {
            rosRef.us = convert_referee_team_info(protoRef.blue());
            rosRef.them = convert_referee_team_info(protoRef.yellow());
        }

        rosRef.designated_position.x = mm_to_m(protoRef.designated_position().x());
        rosRef.designated_position.y = mm_to_m(protoRef.designated_position().y());

        return rosRef;
    }


    /**
     * Converts protoBuf Referee states to the ROS version.
     */
    roboteam_msgs::RefereeStage convert_referee_stage(SSL_Referee_Stage protoStage) {
        int stage;

        switch (protoStage) {
            case SSL_Referee_Stage_NORMAL_FIRST_HALF_PRE:
                stage = roboteam_msgs::RefereeStage::NORMAL_FIRST_HALF_PRE;
            break;
            case SSL_Referee_Stage_NORMAL_FIRST_HALF:
                stage = roboteam_msgs::RefereeStage::NORMAL_FIRST_HALF;
            break;
            case SSL_Referee_Stage_NORMAL_HALF_TIME:
                stage = roboteam_msgs::RefereeStage::NORMAL_HALF_TIME;
            break;
            case SSL_Referee_Stage_NORMAL_SECOND_HALF_PRE:
                stage = roboteam_msgs::RefereeStage::NORMAL_SECOND_HALF_PRE;
            break;
            case SSL_Referee_Stage_NORMAL_SECOND_HALF:
                stage = roboteam_msgs::RefereeStage::NORMAL_SECOND_HALF;
            break;
            case SSL_Referee_Stage_EXTRA_TIME_BREAK:
                stage = roboteam_msgs::RefereeStage::EXTRA_TIME_BREAK;
            break;
            case SSL_Referee_Stage_EXTRA_FIRST_HALF_PRE:
                stage = roboteam_msgs::RefereeStage::EXTRA_FIRST_HALF_PRE;
            break;
            case SSL_Referee_Stage_EXTRA_FIRST_HALF:
                stage = roboteam_msgs::RefereeStage::EXTRA_FIRST_HALF;
            break;
            case SSL_Referee_Stage_EXTRA_HALF_TIME:
                stage = roboteam_msgs::RefereeStage::EXTRA_HALF_TIME;
            break;
            case SSL_Referee_Stage_EXTRA_SECOND_HALF_PRE:
                stage = roboteam_msgs::RefereeStage::EXTRA_SECOND_HALF_PRE;
            break;
            case SSL_Referee_Stage_EXTRA_SECOND_HALF:
                stage = roboteam_msgs::RefereeStage::EXTRA_SECOND_HALF;
            break;
            case SSL_Referee_Stage_PENALTY_SHOOTOUT_BREAK:
                stage = roboteam_msgs::RefereeStage::PENALTY_SHOOTOUT_BREAK;
            break;
            case SSL_Referee_Stage_PENALTY_SHOOTOUT:
                stage = roboteam_msgs::RefereeStage::PENALTY_SHOOTOUT;
            break;
            case SSL_Referee_Stage_POST_GAME:
                stage = roboteam_msgs::RefereeStage::POST_GAME;
            break;
        }

        roboteam_msgs::RefereeStage rosStage;
        rosStage.stage = stage;

        return rosStage;
    }


    /**
     * Converts protoBuf Referee commands to the ROS version.
     */
    roboteam_msgs::RefereeCommand convert_referee_command(SSL_Referee_Command protoCommand, bool us_is_yellow) {
        int command;

        switch (protoCommand) {
            case SSL_Referee_Command_HALT:
                command = roboteam_msgs::RefereeCommand::HALT;
            break;
            case SSL_Referee_Command_STOP:
                command = roboteam_msgs::RefereeCommand::STOP;
            break;
            case SSL_Referee_Command_NORMAL_START:
                command = roboteam_msgs::RefereeCommand::NORMAL_START;
            break;
            case SSL_Referee_Command_FORCE_START:
                command = roboteam_msgs::RefereeCommand::FORCE_START;
            break;
        }

        if (us_is_yellow) {
            switch (protoCommand) {
                case SSL_Referee_Command_PREPARE_KICKOFF_YELLOW:
                    command = roboteam_msgs::RefereeCommand::PREPARE_KICKOFF_US;
                break;
                case SSL_Referee_Command_PREPARE_KICKOFF_BLUE:
                    command = roboteam_msgs::RefereeCommand::PREPARE_KICKOFF_THEM;
                break;
                case SSL_Referee_Command_PREPARE_PENALTY_YELLOW:
                    command = roboteam_msgs::RefereeCommand::PREPARE_PENALTY_US;
                break;
                case SSL_Referee_Command_PREPARE_PENALTY_BLUE:
                    command = roboteam_msgs::RefereeCommand::PREPARE_PENALTY_THEM;
                break;
                case SSL_Referee_Command_DIRECT_FREE_YELLOW:
                    command = roboteam_msgs::RefereeCommand::DIRECT_FREE_US;
                break;
                case SSL_Referee_Command_DIRECT_FREE_BLUE:
                    command = roboteam_msgs::RefereeCommand::DIRECT_FREE_THEM;
                break;
                case SSL_Referee_Command_INDIRECT_FREE_YELLOW:
                    command = roboteam_msgs::RefereeCommand::INDIRECT_FREE_US;
                break;
                case SSL_Referee_Command_INDIRECT_FREE_BLUE:
                    command = roboteam_msgs::RefereeCommand::INDIRECT_FREE_THEM;
                break;
                case SSL_Referee_Command_TIMEOUT_YELLOW:
                    command = roboteam_msgs::RefereeCommand::TIMEOUT_US;
                break;
                case SSL_Referee_Command_TIMEOUT_BLUE:
                    command = roboteam_msgs::RefereeCommand::TIMEOUT_THEM;
                break;
                case SSL_Referee_Command_GOAL_YELLOW:
                    command = roboteam_msgs::RefereeCommand::GOAL_US;
                break;
                case SSL_Referee_Command_GOAL_BLUE:
                    command = roboteam_msgs::RefereeCommand::GOAL_THEM;
                break;
                case SSL_Referee_Command_BALL_PLACEMENT_YELLOW:
                    command = roboteam_msgs::RefereeCommand::BALL_PLACEMENT_US;
                break;
                case SSL_Referee_Command_BALL_PLACEMENT_BLUE:
                    command = roboteam_msgs::RefereeCommand::BALL_PLACEMENT_THEM;
                break;
            }
        } else {
            switch (protoCommand) {
                case SSL_Referee_Command_PREPARE_KICKOFF_BLUE:
                    command = roboteam_msgs::RefereeCommand::PREPARE_KICKOFF_US;
                break;
                case SSL_Referee_Command_PREPARE_KICKOFF_YELLOW:
                    command = roboteam_msgs::RefereeCommand::PREPARE_KICKOFF_THEM;
                break;
                case SSL_Referee_Command_PREPARE_PENALTY_BLUE:
                    command = roboteam_msgs::RefereeCommand::PREPARE_PENALTY_US;
                break;
                case SSL_Referee_Command_PREPARE_PENALTY_YELLOW:
                    command = roboteam_msgs::RefereeCommand::PREPARE_PENALTY_THEM;
                break;
                case SSL_Referee_Command_DIRECT_FREE_BLUE:
                    command = roboteam_msgs::RefereeCommand::DIRECT_FREE_US;
                break;
                case SSL_Referee_Command_DIRECT_FREE_YELLOW:
                    command = roboteam_msgs::RefereeCommand::DIRECT_FREE_THEM;
                break;
                case SSL_Referee_Command_INDIRECT_FREE_BLUE:
                    command = roboteam_msgs::RefereeCommand::INDIRECT_FREE_US;
                break;
                case SSL_Referee_Command_INDIRECT_FREE_YELLOW:
                    command = roboteam_msgs::RefereeCommand::INDIRECT_FREE_THEM;
                break;
                case SSL_Referee_Command_TIMEOUT_BLUE:
                    command = roboteam_msgs::RefereeCommand::TIMEOUT_US;
                break;
                case SSL_Referee_Command_TIMEOUT_YELLOW:
                    command = roboteam_msgs::RefereeCommand::TIMEOUT_THEM;
                break;
                case SSL_Referee_Command_GOAL_BLUE:
                    command = roboteam_msgs::RefereeCommand::GOAL_US;
                break;
                case SSL_Referee_Command_GOAL_YELLOW:
                    command = roboteam_msgs::RefereeCommand::GOAL_THEM;
                break;
                case SSL_Referee_Command_BALL_PLACEMENT_BLUE:
                    command = roboteam_msgs::RefereeCommand::BALL_PLACEMENT_US;
                break;
                case SSL_Referee_Command_BALL_PLACEMENT_YELLOW:
                    command = roboteam_msgs::RefereeCommand::BALL_PLACEMENT_THEM;
                break;
            }
        }

        roboteam_msgs::RefereeCommand rosCommand;
        rosCommand.command = command;

        return rosCommand;
    }


    /**
     * Converts protoBuf TeamInfo to the ROS version.
     */
    roboteam_msgs::RefereeTeamInfo convert_referee_team_info(SSL_Referee_TeamInfo protoInfo) {
        roboteam_msgs::RefereeTeamInfo rosInfo;

        rosInfo.name = protoInfo.name();
        rosInfo.score = protoInfo.score();
        rosInfo.red_cards = protoInfo.red_cards();

        for (int i = 0; i < protoInfo.yellow_card_times_size(); ++i) {
            rosInfo.yellow_card_times.push_back(protoInfo.yellow_card_times(i));
        }

        rosInfo.yellow_cards = protoInfo.yellow_cards();
        rosInfo.timeouts = protoInfo.timeouts();
        rosInfo.timeout_time = protoInfo.timeout_time();
        rosInfo.goalie = protoInfo.goalie();

        return rosInfo;
    }

}
