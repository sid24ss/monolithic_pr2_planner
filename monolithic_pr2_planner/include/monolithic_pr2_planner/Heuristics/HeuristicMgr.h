#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Heuristics/AbstractHeuristic.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <memory>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <monolithic_pr2_planner/Visualizer.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>


#define NUM_MHA_BASE_HEUR 1


namespace monolithic_pr2_planner {
    /*! \brief The manager class that handles all the heuristics.
     */
    class HeuristicMgr : public OccupancyGridUser{
        public:
            HeuristicMgr();
            // HeuristicMgr(CSpaceMgrPtr cspace_mgr);

            // The master function that initializes all the heuristics you
            // want.
            void initializeHeuristics();

            // Add methods for all possible kinds of heuristics. Whenever a new
            // heuristic type is added, a corresponding add<type>Heur() method
            // needs to be added here. Returns the id of the heuristic in the
            // internal m_heuristics vector.
            int add3DHeur(const int cost_multiplier=1);
            int add2DHeur(const int cost_multiplier=1, const double radius_m = 0);
            int addMHABaseHeur(const int cost_multiplier=1);
            int addEndEffHeur(const int cost_multiplier=1);
            int addArmAnglesHeur(const int cost_multiplier=1);

            // Updates the collision map for the heuristics that need them.
            // Doesn't take in an argument because each 3D heuristic shares the
            // occupancy grid singleton.
            void update3DHeuristicMaps();

            // Updates the 2D map for the heuristics that need them
            void update2DHeuristicMaps(const std::vector<signed char>& data);

            // TODO: Multiple goals should just take the goal state and the heuristic ID.
            void setGoal(GoalState& state);

            // Get the heuristic value
            std::vector<int> getGoalHeuristic(const GraphStatePtr& state);

            // MHA stuff

            // Takes in the base heuristic id, and samples the circle for
            // m_num_mha_heuristics number of points. Adds as many 2D heuristics with 0
            // radius
            void initializeMHAHeuristics(const int base_heur_id, const int cost_multiplier = 1); // No radius support as of now

            // void numberOfMHAHeuristics(int num_mha_heuristics){ m_num_mha_heuristics
            //     = num_mha_heuristics;};

            void reset();

            // int numberOfMHAHeuristics(){ return m_num_mha_heuristics;};
	    inline void setCollisionSpaceMgr(CSpaceMgrPtr cspace_mgr){ m_cspace_mgr = cspace_mgr;};
 
        private:
            bool isValidIKForGoalState(int g_x, int g_y);
            bool checkIKAtPose(int g_x, int g_y, RobotPosePtr&
                final_pose);
            RightContArmState getRightArmIKSol(int g_x, int g_y);
            void initNewMHABaseHeur(int g_x, int g_y, RightContArmState& r_arm_state, const int
                cost_multiplier);

            GoalState m_goal;
            std::vector<AbstractHeuristicPtr> m_heuristics;

            inline double randomDouble(double min, double max){
                return min + (max-min) * ( double(rand()) / RAND_MAX );
            }
            
            // MHA stuff
            int m_num_mha_heuristics;
            std::vector<int> m_mha_heur_ids;
            int m_base_heur_id;
            int m_arm_angles_heur_id;
            
            // Saving the goal and the grid for MHA heuristics
            unsigned char** m_grid;
            std::vector<signed char> m_grid_data;

            CSpaceMgrPtr m_cspace_mgr;
    };
    typedef boost::shared_ptr<HeuristicMgr> HeuristicMgrPtr;
}
