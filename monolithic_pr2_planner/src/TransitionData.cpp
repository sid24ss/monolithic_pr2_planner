#include <monolithic_pr2_planner/TransitionData.h>

using namespace monolithic_pr2_planner;

TransitionData TransitionData::combineTData(const TransitionData t_data1,
               const TransitionData t_data2)
{
    TransitionData t_data;
    int successor_id = t_data2.successor_id();
    t_data.successor_id(successor_id);
    bool isBaseType = (t_data1.motion_type() == MPrim_Types::BASE) ||
                        (t_data2.motion_type() == MPrim_Types::BASE);
    if (isBaseType) {
        t_data.motion_type(MPrim_Types::BASE);
    } else {
        t_data.motion_type(t_data2.motion_type());
    }

    std::vector<RobotState> steps(t_data1.interm_robot_steps());
    steps.insert(steps.end(), t_data2.interm_robot_steps().begin(), t_data2.
        interm_robot_steps().end());
    t_data.interm_robot_steps(steps);

    std::vector<ContBaseState> base_steps(t_data1.cont_base_interm_steps());
    base_steps.insert(base_steps.end(), t_data2.cont_base_interm_steps().begin(), t_data2.
        cont_base_interm_steps().end());
    t_data.cont_base_interm_steps(base_steps);

    int total_cost = t_data1.cost();
    total_cost += t_data2.cost();
    t_data.cost(total_cost);
    
    return t_data;
}