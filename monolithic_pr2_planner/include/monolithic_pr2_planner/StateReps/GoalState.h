#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <vector>

namespace monolithic_pr2_planner {
    // TODO: generalize this state . also have free angle defaults?
    // TODO: implement setgoal
    class GoalState {
        public:
            GoalState(){ };
            GoalState(
                std::shared_ptr<DiscObjectState> r_obj_goal,
                std::shared_ptr<DiscObjectState> l_obj_goal,
                double xyz_tol, double roll_tol, double pitch_tol,
                double yaw_tol);
            bool isSatisfiedBy(const GraphStatePtr& graph_state);
            bool isPartiallySatisfiedBy(const GraphStatePtr& graph_state);
            void storeAsSolnState(const GraphStatePtr& state){ m_full_goal_state = state; };
            GraphStatePtr getSolnState(){ return m_full_goal_state; };
            bool isSolnStateID(int state_id);
            void addPotentialSolnState(const GraphStatePtr& graph_state);
            std::shared_ptr<DiscObjectState> getRightObjectState() const { return m_r_goal_state;};
            std::shared_ptr<DiscObjectState> getLeftObjectState() const { return m_l_goal_state;};
            void setRightGoal(DiscObjectState r_goal_state){ m_r_goal_state =
                std::make_shared<DiscObjectState>(r_goal_state);};
            void setLeftGoal(DiscObjectState l_goal_state){ m_l_goal_state =
                std::make_shared<DiscObjectState>(l_goal_state);};
            bool withinXYZTol(const GraphStatePtr& graph_state, bool right_arm
                = true);
            bool withinRPYTol(const GraphStatePtr& graph_state, bool right_arm
                = true);
            void visualize();
        private:
            vector<int> m_possible_goals;
            std::shared_ptr<DiscObjectState> m_r_goal_state;
            std::shared_ptr<DiscObjectState> m_l_goal_state;
            GraphStatePtr m_full_goal_state;
            std::vector<double> m_tolerances;
            double l_free_angle;
            double r_free_angle;
    };
    typedef boost::shared_ptr<GoalState> GoalStatePtr;
}
