#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Visualizer.h>

using namespace monolithic_pr2_planner;

//GoalState::GoalState(SearchRequestPtr search_request):
//    m_goal_state(search_request->m_params->obj_goal), m_tolerances(4,0){
//
//    m_tolerances[Tolerances::XYZ] = search_request->m_params->xyz_tolerance;
//    m_tolerances[Tolerances::ROLL] = search_request->m_params->roll_tolerance;
//    m_tolerances[Tolerances::PITCH] = search_request->m_params->pitch_tolerance;
//    m_tolerances[Tolerances::YAW] = search_request->m_params->yaw_tolerance;
//}

GoalState::GoalState(std::shared_ptr<DiscObjectState> r_obj_goal,
                     std::shared_ptr<DiscObjectState> l_obj_goal,
            double xyz_tol, double roll_tol, double pitch_tol, double yaw_tol)
    : m_r_goal_state(r_obj_goal),
      m_l_goal_state(l_obj_goal),
      m_tolerances(4,0)
{
    m_tolerances[Tolerances::XYZ] = xyz_tol;
    m_tolerances[Tolerances::ROLL] = roll_tol;
    m_tolerances[Tolerances::PITCH] = pitch_tol;
    m_tolerances[Tolerances::YAW] = yaw_tol;
}

// return true if either arm is within the tolerance.
bool GoalState::withinXYZTol(const GraphStatePtr& graph_state, bool right_arm) {
    // not sure why there's a .005 here. ask ben
    ContObjectState c_tol(m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005,
                          m_tolerances[Tolerances::ROLL],
                          m_tolerances[Tolerances::PITCH],
                          m_tolerances[Tolerances::YAW]);
    DiscObjectState d_tol = c_tol.getDiscObjectState();
    DiscObjectState r_obj = graph_state->getRightObjectStateRelMap();
    DiscObjectState l_obj = graph_state->getLeftObjectStateRelMap();

    bool within_xyz_tol = false;

    if (right_arm) {
      if (m_r_goal_state){
        within_xyz_tol = (abs(m_r_goal_state->x()-r_obj.x()) <= d_tol.x() &&
                          abs(m_r_goal_state->y()-r_obj.y()) <= d_tol.y() &&
                          abs(m_r_goal_state->z()-r_obj.z()) <= d_tol.z());
      }
    } else {
      if (m_l_goal_state){
        within_xyz_tol = (abs(m_l_goal_state->x()-l_obj.x()) <= d_tol.x() &&
                          abs(m_l_goal_state->y()-l_obj.y()) <= d_tol.y() &&
                          abs(m_l_goal_state->z()-l_obj.z()) <= d_tol.z());
      }
    }
    return within_xyz_tol;
}

// return true if either arm is within the tolerance.
bool GoalState::withinRPYTol(const GraphStatePtr& graph_state, bool right_arm) {
    // not sure why there's a .005 here. ask ben
    ContObjectState c_tol(m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005,
                          m_tolerances[Tolerances::ROLL],
                          m_tolerances[Tolerances::PITCH],
                          m_tolerances[Tolerances::YAW]);
    DiscObjectState d_tol = c_tol.getDiscObjectState();
    DiscObjectState r_obj = graph_state->getRightObjectStateRelMap();
    DiscObjectState l_obj = graph_state->getLeftObjectStateRelMap();

    bool within_rpy_tol = false;

    if (right_arm) {
      if (m_r_goal_state){
        within_rpy_tol = (abs(m_r_goal_state->roll()-r_obj.roll()) <= d_tol.roll() &&
                          abs(m_r_goal_state->pitch()-r_obj.pitch()) <= d_tol.pitch() &&
                          abs(m_r_goal_state->yaw()-r_obj.yaw()) <= d_tol.yaw());
      }
    } else {
      if (m_l_goal_state){
        within_rpy_tol = (abs(m_l_goal_state->roll()-l_obj.roll()) <= d_tol.roll() &&
                          abs(m_l_goal_state->pitch()-l_obj.pitch()) <= d_tol.pitch() &&
                          abs(m_l_goal_state->yaw()-l_obj.yaw()) <= d_tol.yaw());
      }
    }
    return within_rpy_tol;
}

bool GoalState::isSatisfiedBy(const GraphStatePtr& graph_state){
    bool right_arm = true;
    bool r_within_xyz = withinXYZTol(graph_state, right_arm);
    bool r_within_rpy = withinRPYTol(graph_state, right_arm);
    
    right_arm = false;
    bool l_within_xyz = withinXYZTol(graph_state, right_arm);
    bool l_within_rpy = withinRPYTol(graph_state, right_arm);

    return (r_within_xyz && r_within_rpy) && (l_within_xyz && l_within_rpy);
}

bool GoalState::isPartiallySatisfiedBy(const GraphStatePtr& graph_state)
{
    bool right_arm = true;
    bool r_within_xyz = withinXYZTol(graph_state, right_arm);
    bool r_within_rpy = withinRPYTol(graph_state, right_arm);
    ROS_DEBUG_NAMED(SEARCH_LOG, "r_within_xyz : %d, r_within_rpy : %d",
      r_within_xyz, r_within_rpy);
    
    right_arm = false;
    bool l_within_xyz = withinXYZTol(graph_state, right_arm);
    bool l_within_rpy = withinRPYTol(graph_state, right_arm);
    ROS_DEBUG_NAMED(SEARCH_LOG, "l_within_xyz : %d, l_within_rpy : %d",
      l_within_xyz, l_within_rpy);
    return ((r_within_xyz && r_within_rpy) || (l_within_xyz && l_within_rpy));
}


bool GoalState::isSolnStateID(int state_id){
    for (auto& goal : m_possible_goals){
        if (goal == state_id){
            return true;
        }
    }
    return false;
}
void GoalState::addPotentialSolnState(const GraphStatePtr& graph_state) { 
    m_possible_goals.push_back(graph_state->id());
}

void GoalState::visualize(){
  if (m_r_goal_state) {
    ContObjectState r_cont_goal = ContObjectState(*m_r_goal_state);
    std::vector<double> pose;
    pose.push_back(r_cont_goal.x());
    pose.push_back(r_cont_goal.y());
    pose.push_back(r_cont_goal.z());
    pose.push_back(r_cont_goal.roll());
    pose.push_back(r_cont_goal.pitch());
    pose.push_back(r_cont_goal.yaw());
    Visualizer::pviz->visualizePose(pose, "right_goal");
  }

  if (m_l_goal_state) {
    ContObjectState l_cont_goal = ContObjectState(*m_l_goal_state);
    std::vector<double> pose;
    pose.push_back(l_cont_goal.x());
    pose.push_back(l_cont_goal.y());
    pose.push_back(l_cont_goal.z());
    pose.push_back(l_cont_goal.roll());
    pose.push_back(l_cont_goal.pitch());
    pose.push_back(l_cont_goal.yaw());
    Visualizer::pviz->visualizePose(pose, "left_goal");
  }
}
