#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Constants.h>

using namespace monolithic_pr2_planner;

SearchRequest::SearchRequest(SearchRequestParamsPtr params){
    m_params = params;
}

bool SearchRequest::isValid(CSpaceMgrPtr& cspace){
    RobotState robot_start_pose(m_params->base_start,
                               m_params->right_arm_start,
                               m_params->left_arm_start);
    if (m_params->initial_epsilon < 1 || 
        m_params->final_epsilon < 1 || 
        m_params->decrement_epsilon < 0){
        ROS_ERROR_NAMED(INIT_LOG, "Epsilons in search request were set wrong!");
        return false;
    }
    if (m_params->planning_mode < 0 ||
        m_params->planning_mode > PlanningModes::TWO_ARM){
        ROS_ERROR_NAMED(INIT_LOG, "Planning mode specified doesn't make sense: %d",
                        m_params->planning_mode);
        return false;
    }


    // TODO add in a check for dual arm to make sure the left arm can reach the
    // object pose relative to the right arm

    if (!cspace->isValid(robot_start_pose)){
        ROS_ERROR_NAMED(INIT_LOG, "Starting pose is invalid.");
        return false;
    }
    return true;
}

GoalStatePtr SearchRequest::createGoalState(){
    ROS_DEBUG_NAMED(CONFIG_LOG, "Creating goal state from search request");
    std::shared_ptr<DiscObjectState> r_goal;
    std::shared_ptr<DiscObjectState> l_goal;
    if (m_params->r_obj_goal){
        ROS_DEBUG_NAMED(CONFIG_LOG, "setting right");
        r_goal = std::make_shared<DiscObjectState>(*m_params->r_obj_goal);
    }
    if (m_params->l_obj_goal){
        ROS_DEBUG_NAMED(CONFIG_LOG, "setting left");
        l_goal = std::make_shared<DiscObjectState>(*m_params->l_obj_goal);
    }
    return boost::make_shared<GoalState>(r_goal,
                                         l_goal,
                                         m_params->xyz_tolerance,
                                         m_params->roll_tolerance,
                                         m_params->pitch_tolerance,
                                         m_params->yaw_tolerance);
}
