#include <monolithic_pr2_planner/Environment.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <assert.h>

#define GOAL_STATE 1
using namespace monolithic_pr2_planner;
using namespace boost;

// stateid2mapping pointer inherited from sbpl interface. needed for planner.
Environment::Environment(ros::NodeHandle nh) : 
    m_hash_mgr(new HashManager(&StateID2IndexMapping)), 
    m_nodehandle(nh), m_mprims(m_goal), 
    m_heur_mgr(new HeuristicMgr()) {
    m_param_catalog.fetch(nh);
    configurePlanningDomain();
}

/**
 * @brief Resets the environment.
 * @details Intended to be used between calls to subsequent planning
 * requests.
 */
void Environment::reset(){
    m_heur_mgr->reset();
    // m_heur_mgr->setCollisionSpaceMgr(m_cspace_mgr);
    m_hash_mgr.reset(new HashManager(&StateID2IndexMapping));

    // Fetch params again, in case they're being modified between calls.
    // m_param_catalog.fetch(m_nodehandle);
}

bool Environment::configureRequest(SearchRequestParamsPtr search_request_params,
                                   int& start_id, int& goal_id){
    SearchRequestPtr search_request = SearchRequestPtr(new SearchRequest(search_request_params));
    configureQuerySpecificParams(search_request);
    if (!setStartGoal(search_request, start_id, goal_id)){
        return false;
    }
    return true;
}

int Environment::GetGoalHeuristic(int stateID){
    // For now, return the max of all the heuristics
    return GetGoalHeuristic(stateID, 0);
}

int Environment::GetGoalHeuristic(int stateID, int goal_id){
    // For now, return the max of all the heuristics
    // This vector is of size NUM_MHA_BASE_HEUR + 2
    // Eg, if NUM_MHA_BASE_HEUR is 2, that means there are 2 additional base
    // heuristics. So, the values will be endEff, Base, Base1, Base2
    std::vector<int> values = m_heur_mgr->getGoalHeuristic(m_hash_mgr->getGraphState(stateID));
    // SMHA*
    ROS_DEBUG_NAMED(HEUR_LOG, "Heuristic values: Arm: %d\t Base : %d\t", values[0], values[2]);
    switch(goal_id){
        case 0: //Anchor
            return std::max(values[0], values[1]);
        case 1: // base
            return values[2];
        case 2: // Base + arm
            return static_cast<int>(0.5f*values[2] +
                0.5f*values[0]);
    }

    return std::max(values[0],values[1]);
}


/*
 * This is a tricky function. we have to know when to run a simple collision
 * check, when it's a real true cost, and when to inflate. this is the truth
 * table behind it
 *
 *                    (using CC 
 *                    specified 
 *                     on left)
 *                                
 * using simple CC? | valid CC?  | istruecost?  | cost value? | inflate?
 *      T               T        |       T              >0          F
 *      T               F        |       F              >0          T
 *      F               T        |       T              >0          F
 *      F               F        |       T              -1          F
 *
 */

int Environment::EvaluateCost(int parentID, int childID, bool& isTrueCost){
    bool USE_RESEARCH = false;
    GraphStatePtr child = m_hash_mgr->getGraphState(childID);
    TransitionData t_data;

    size_t hashKey = m_hasher(Edge(parentID, childID));
    if (m_edges.find(Edge(parentID, childID)) == m_edges.end()){
        ROS_ERROR("transition hasn't been found??");
        assert(false);
    }

    //ROS_INFO("stored at %lu is %x", hashKey, m_edges[hashKey].get());
    vector<MotionPrimitivePtr> small_mprims;
    small_mprims.push_back(m_edges[Edge(parentID, childID)]);
    //small_mprims.push_back(m_edges[hashKey]);


    PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr);
    bool valid_cc = false;
    bool simple_check = child->checkSimpleCollisionModel();

    if (USE_RESEARCH){
        postprocessor.findBestTransition(parentID, childID, t_data, 
                                        small_mprims,
                                         //m_mprims.getMotionPrims(),
                                         valid_cc,
                                         simple_check);
        // huge hack. findbesttransition should work using the motion primitive
        // that's stored in the hash table. however, for some reason, sometimes
        // the mtprims don't match up???
        //if (simple_check && t_data.cost() < 0){
        //    postprocessor.findBestTransition(parentID, childID, t_data, 
        //                                     m_mprims.getMotionPrims(),
        //                                     valid_cc,
        //                                     simple_check);
        //}



        // this should give us the third column in the above table
        isTrueCost = ((!simple_check && valid_cc) || 
                      !(simple_check ^ valid_cc));

        // if the simple check failed, we need to run the full check the next time
        // around
        //ROS_INFO("simple check %d valid_cc %d cost %d", simple_check, valid_cc, t_data.cost());
        bool doFullCCNextTime = (simple_check && !valid_cc );
        if (doFullCCNextTime){
            child->checkSimpleCollisionModel(false);
            assert(t_data.cost() > 0);
            assert(isTrueCost == false);
        }
        if (simple_check){
            assert(t_data.cost() > 0);
        }
        if (!simple_check && valid_cc){
            assert(t_data.cost() > 0);
        }
        if (!simple_check && !valid_cc){
            assert(t_data.cost() == -1);
        }
    } else {
        isTrueCost = true;
        bool found_transition = postprocessor.findBestTransition(parentID, childID, t_data, 
                                         small_mprims);
                                         //m_mprims.getMotionPrims());
    }
    return t_data.cost();
}

int Environment::GetTrueCost(int parentID, int childID){
    // currently a hack for getting the cost between the two states
    PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr);
    TransitionData t_data;
    postprocessor.findBestTransition(parentID, childID, t_data, 
                                     m_mprims.getMotionPrims());
    return t_data.cost();

}

void Environment::GetSuccs(int sourceStateID, vector<int>* succIDs, 
                           vector<int>* costs, std::vector<bool>* isTrueCost){
    GetSuccs(sourceStateID, succIDs, costs);
    for (size_t i=0; i < costs->size(); i++){
        bool val = (*succIDs)[i] == GOAL_STATE;
        isTrueCost->push_back(val);
    }
}

void Environment::GetSuccs(int sourceStateID, vector<int>* succIDs, 
                           vector<int>* costs){
    static double get_succs_time = 0;
    static int get_succs_counts = 0;
    double temptime = clock();

    static double mprim_time = 0;
    static int mprim_counts = 0;

    static double hash_time = 0;
    static int hash_counts = 0;

    vector<MotionPrimitivePtr> all_mprims = m_mprims.getMotionPrims();
    assert(sourceStateID != GOAL_STATE);
    //ROS_DEBUG_NAMED(SEARCH_LOG, 
    //        "==================Expanding state %d==================", 
    //                sourceStateID);
    succIDs->clear();
    succIDs->reserve(all_mprims.size());
    costs->clear();
    costs->reserve(all_mprims.size());

    GraphStatePtr source_state = m_hash_mgr->getGraphState(sourceStateID);
    //ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
    //source_state->robot_pose().printToDebug(SEARCH_LOG);
    if(m_param_catalog.m_visualization_params.expansions){
        source_state->robot_pose().visualize();
        ContBaseState test = source_state->robot_pose().base_state();
        double height = .5+test.z();
        double simple_radius = .8;
        Visualizer::pviz->visualizeSphere(test.x(), 
                                          test.y(),
                                          height, simple_radius, 100, "simple_check", 1);
        usleep(5000);
    }

    int mprim_id = 0;
    for (auto mprim : all_mprims){
        //ROS_DEBUG_NAMED(SEARCH_LOG, "Applying motion:");
        //mprim->printEndCoord();
        GraphStatePtr successor;
        TransitionData t_data;

        double temptime_mprim = clock();
        
        if (mprim->motion_type() == MPrim_Types::ARM){
            successor.reset(new GraphState(*source_state));
            successor->lazyApplyMPrim(mprim->getEndCoord());
            //ROS_INFO("source/successor");
            //mprim->printEndCoord();
            //source_state->printToInfo(MPRIM_LOG);
            //successor->printToInfo(MPRIM_LOG);
            //ROS_INFO("done");
        } else {
            if (!mprim->apply(*source_state, successor, t_data)){
                //ROS_DEBUG_NAMED(MPRIM_LOG, "couldn't apply mprim");
                continue;
            }
        }

        //if (!mprim->apply(*source_state, successor, t_data)){
        //    //ROS_DEBUG_NAMED(MPRIM_LOG, "couldn't apply mprim");
        //    continue;
        //}

        mprim_time += (clock()-temptime_mprim)/(double)CLOCKS_PER_SEC;
        mprim_counts++;
        if (mprim_counts % 10000 == 0){
            ROS_WARN("mprim time %f", mprim_time);
        }

        m_hash_mgr->save(successor);

        double temptime_hash = clock();
        if (m_goal->isSatisfiedBy(successor)){
            m_goal->storeAsSolnState(successor);
            succIDs->push_back(GOAL_STATE);
        } else {
            succIDs->push_back(successor->id());
        }
        hash_time += (clock()-temptime_hash)/(double)CLOCKS_PER_SEC;
        hash_counts++;
        if (hash_counts % 10000 == 0){
            ROS_WARN("hash time %f counts %d", hash_time, hash_counts);
        }

        Edge key = Edge(sourceStateID, successor->id());
        //ROS_INFO("hashed edge between %d %d to %lu", sourceStateID, 
        //                                             successor->id(), hashKey);
        //m_edges[hashKey] = mprim;

        if (m_edges.find(key) == m_edges.end()){ // not found
            //m_edges[key] = mprim_id;
        } else {
            //m_edges[key] = mprim_id;
        }
        m_edges.insert(map<Edge, MotionPrimitivePtr>::value_type(key, mprim));
        //if (sourceStateID == 11181 && successor->id() == 11246){
        //    source_state->robot_pose().printToInfo(SEARCH_LOG);
        //    successor->robot_pose().printToInfo(SEARCH_LOG);
        //    ROS_INFO("using mprim %x", mprim.get());
        //    ROS_INFO("stored is %x", m_edges[key].get());
        //}

        costs->push_back(mprim->cost());

        //if (m_cspace_mgr->isValidSuccessor(*successor, t_data) && 
        //    m_cspace_mgr->isValidTransitionStates(t_data)){
        //    ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
        //    source_state->printToDebug(SEARCH_LOG);
        //    m_hash_mgr->save(successor);
        //    ROS_DEBUG_NAMED(MPRIM_LOG, "successor state with id %d is:", 
        //                    successor->id());
        //    successor->printToDebug(MPRIM_LOG);

        //    if (m_goal->isSatisfiedBy(successor)){
        //        m_goal->storeAsSolnState(successor);
        //        ROS_INFO_NAMED(SEARCH_LOG, "Found potential goal at state %d %d", successor->id(),
        //            mprim->cost());
        //        succIDs->push_back(GOAL_STATE);
        //    } else {
        //        succIDs->push_back(successor->id());
        //    }
        //    costs->push_back(mprim->cost());
        //    ROS_DEBUG_NAMED(SEARCH_LOG, "motion succeeded with cost %d", mprim->cost());
        //} else {
        //    //successor->robot_pose().visualize();
        //    ROS_DEBUG_NAMED(SEARCH_LOG, "successor failed collision checking");
        //}
        mprim_id++;
    }

    get_succs_time += (clock()-temptime)/(double)CLOCKS_PER_SEC;
    get_succs_counts++;
    if (get_succs_counts % 100 == 0){
        ROS_WARN("time spent expanding %f count %d", get_succs_time, get_succs_counts);
    }
}

bool Environment::setStartGoal(SearchRequestPtr search_request,
                               int& start_id, int& goal_id){
    RobotState start_pose(search_request->m_params->base_start, 
                         search_request->m_params->right_arm_start,
                         search_request->m_params->left_arm_start);
    ContObjectState obj_state = start_pose.getObjectStateRelMap();
    obj_state.printToInfo(SEARCH_LOG);

    if (!search_request->isValid(m_cspace_mgr)){
        obj_state.printToInfo(SEARCH_LOG);
        start_pose.visualize();
        return false;
    }

    GraphStatePtr start_graph_state = make_shared<GraphState>(start_pose);
    m_hash_mgr->save(start_graph_state);
    start_id = start_graph_state->id();
    assert(m_hash_mgr->getGraphState(start_graph_state->id()) == start_graph_state);

    ROS_INFO_NAMED(SEARCH_LOG, "Start state set to:");
    start_pose.printToInfo(SEARCH_LOG);
    obj_state.printToInfo(SEARCH_LOG);
    // start_pose.visualize();


    m_goal = search_request->createGoalState();

    if (m_hash_mgr->size() < 2){
        goal_id = saveFakeGoalState(start_graph_state);
    } else {
        goal_id = 1;
    }

    ROS_INFO_NAMED(SEARCH_LOG, "Goal state created:");
    ContObjectState c_goal = m_goal->getObjectState();
    c_goal.printToInfo(SEARCH_LOG);
    m_goal->visualize();

    // This informs the adaptive motions about the goal.
    ArmAdaptiveMotionPrimitive::goal(*m_goal);
    BaseAdaptiveMotionPrimitive::goal(*m_goal);

    // informs the heuristic about the goal
    m_heur_mgr->setGoal(*m_goal);

    return true;
}

// a hack to reserve a goal id in the hash so that no real graph state is ever
// saved as the goal state id
int Environment::saveFakeGoalState(const GraphStatePtr& start_graph_state){
    GraphStatePtr fake_goal = make_shared<GraphState>(*start_graph_state);
    RobotState fake_robot_state = fake_goal->robot_pose();
    DiscBaseState fake_base = fake_robot_state.base_state();
    fake_base.x(0); fake_base.y(0); fake_base.z(0);
    fake_robot_state.base_state(fake_base);
    fake_goal->robot_pose(fake_robot_state);
    m_hash_mgr->save(fake_goal);
    int goal_id = fake_goal->id();
    assert(goal_id == GOAL_STATE);
    return goal_id;
}

// this sets up the environment for things that are query independent.
void Environment::configurePlanningDomain(){
    // used for collision space and discretizing plain xyz into grid world 
    OccupancyGridUser::init(m_param_catalog.m_occupancy_grid_params,
                        m_param_catalog.m_robot_resolution_params);
    

    // used for discretization of robot movements
    ContArmState::setRobotResolutionParams(m_param_catalog.m_robot_resolution_params);

#ifdef USE_IKFAST_SOLVER
    ROS_DEBUG_NAMED(CONFIG_LOG, "Using IKFast");
#endif
#ifdef USE_KDL_SOLVER
    ROS_DEBUG_NAMED(CONFIG_LOG, "Using KDL");
#endif

    // Initialize the heuristics. The (optional) parameter defines the cost multiplier.

    m_heur_mgr->initializeHeuristics();

    // used for arm kinematics
    LeftContArmState::initArmModel(m_param_catalog.m_left_arm_params);
    RightContArmState::initArmModel(m_param_catalog.m_right_arm_params);

    // collision space mgr needs arm models in order to do collision checking
    // have to do this funny thing  of initializing an object because of static
    // variable + inheritance (see ContArmState for details)
    LeftContArmState l_arm;
    RightContArmState r_arm;
    m_cspace_mgr = make_shared<CollisionSpaceMgr>(r_arm.getArmModel(),
                                                  l_arm.getArmModel());
    m_heur_mgr->setCollisionSpaceMgr(m_cspace_mgr);
    // load up motion primitives
    m_mprims.loadMPrims(m_param_catalog.m_motion_primitive_params);

    // load up static pviz instance for visualizations. 
    Visualizer::createPVizInstance();
    Visualizer::setReferenceFrame(std::string("/map"));
}

// sets parameters for query specific things
void Environment::configureQuerySpecificParams(SearchRequestPtr search_request){
    // sets the location of the object in the frame of the wrist
    // have to do this funny thing  of initializing an object because of static
    // variable + inheritance (see ContArmState for details)
    LeftContArmState l_arm;
    RightContArmState r_arm;
    l_arm.setObjectOffset(search_request->m_params->left_arm_object);
    r_arm.setObjectOffset(search_request->m_params->right_arm_object);

    // this specifically sets which arm RobotState will generate IK solutions
    // for
    RobotState::setPlanningMode(search_request->m_params->planning_mode);
    m_mprims.loadMPrimSet(search_request->m_params->planning_mode);
}

/*! \brief Given the solution path containing state IDs, reconstruct the
 * actual corresponding robot states. This also makes the path smooth in between
 * each state id because we add in the intermediate states given by the
 * transition data.
 */
vector<FullBodyState> Environment::reconstructPath(vector<int> soln_path){
    PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr);
    vector<FullBodyState> final_path = postprocessor.reconstructPath(soln_path, 
                                                                     *m_goal, 
                                                                     m_mprims.getMotionPrims());
    if(m_param_catalog.m_visualization_params.final_path){
        postprocessor.visualizeFinalPath(final_path);
    }
    return final_path;
}
