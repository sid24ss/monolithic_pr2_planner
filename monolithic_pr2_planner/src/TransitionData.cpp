#include <monolithic_pr2_planner/TransitionData.h>
#include <monolithic_pr2_planner/LoggerNames.h>

using namespace monolithic_pr2_planner;

TransitionData TransitionData::combineTData(const TransitionData t_data1,
               const TransitionData t_data2)
{
    TransitionData t_data;

    // the id of the end robot state, which comes after t_data2.
    int successor_id = t_data2.successor_id();
    t_data.successor_id(successor_id);
    
    // set the motion type. We want this to default to base when there is a base
    // movement
    bool isBaseType = (t_data1.motion_type() == MPrim_Types::BASE) ||
                        (t_data2.motion_type() == MPrim_Types::BASE);
    if (isBaseType) {
        t_data.motion_type(MPrim_Types::BASE);
    } else {
        t_data.motion_type(t_data2.motion_type());
    }

    // set the intermediate robot steps
    // ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "setting robot steps");
    std::vector<RobotState> steps(t_data1.interm_robot_steps());
    for (auto step : t_data2.interm_robot_steps()) {
        steps.push_back(step);
    }
    t_data.interm_robot_steps(steps);

    // set the base steps
    // ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "setting base steps");
    std::vector<ContBaseState> base_steps(t_data1.cont_base_interm_steps());
    for (auto step : t_data2.cont_base_interm_steps()) {
        base_steps.push_back(step);
    }
    t_data.cont_base_interm_steps(base_steps);

    // set the cost
    int total_cost = t_data1.cost();
    total_cost += t_data2.cost();
    t_data.cost(total_cost);
    
    return t_data;
}