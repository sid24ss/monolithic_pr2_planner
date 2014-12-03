#include <monolithic_pr2_planner_node/EnvInterfaces.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <kdl/frames.hpp>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Pose.h>
#include <leatherman/utils.h>
#include <LinearMath/btVector3.h>
#include <limits>
#include <algorithm>

using namespace monolithic_pr2_planner_node;
using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;
using namespace KDL;

// constructor automatically launches the collision space interface, which only
// loads it up with a pointer to the collision space mgr. it doesn't bind to any
// topic.

EnvInterfaces::EnvInterfaces(boost::shared_ptr<monolithic_pr2_planner::Environment> env, ros::NodeHandle nh) :
    m_nodehandle(nh),
    m_env(env), m_collision_space_interface(new CollisionSpaceInterface(env->getCollisionSpace(), env->getHeuristicMgr()))
    // m_generator(new StartGoalGenerator(env->getCollisionSpace()))
    // m_rrt(new OMPLPR2Planner(env->getCollisionSpace(), RRT)),
    // m_prm(new OMPLPR2Planner(env->getCollisionSpace(), PRM_P)),
    // m_rrtstar(new OMPLPR2Planner(env->getCollisionSpace(), RRTSTAR)),
    // m_rrtstar_first_sol(new OMPLPR2Planner(env->getCollisionSpace(),
        // RRTSTARFIRSTSOL))
{

  m_collision_space_interface->mutex = &mutex;

        getParams();
    bool forward_search = true;
    m_ara_planner.reset(new ARAPlanner(m_env.get(), forward_search));
    m_mha_planner.reset(new MHAPlanner(m_env.get(), NUM_SMHA_HEUR, forward_search));
    m_costmap_pub = m_nodehandle.advertise<nav_msgs::OccupancyGrid>("costmap_pub", 1);
    m_costmap_publisher.reset(new
        costmap_2d::Costmap2DPublisher(m_nodehandle,1,"/map"));

    interrupt_sub_ = nh.subscribe("/sbpl_planning/interrupt", 1, &EnvInterfaces::interruptPlannerCallback,this);
}

void EnvInterfaces::interruptPlannerCallback(std_msgs::EmptyConstPtr){
  ROS_WARN("Planner interrupt received!");
  m_mha_planner->interrupt();
}

void EnvInterfaces::getParams(){
    m_nodehandle.param<string>("reference_frame", m_params.ref_frame, 
                                    string("map"));
    m_nodehandle.param<bool>("run_trajectory", m_params.run_trajectory, false);
    m_nodehandle.param<string>("controller_service",
        m_params.controller_service, "/monolithic_controller/execute_path");
}

void EnvInterfaces::bindPlanPathToEnv(string service_name){
    m_plan_service = m_nodehandle.advertiseService(service_name, 
                                                   &EnvInterfaces::planPathCallback,
                                                   this);
}

bool EnvInterfaces::runMHAPlanner(
    GetMobileArmPlan::Request &req,
    GetMobileArmPlan::Response &res,
    SearchRequestParamsPtr search_request,
    int counter,
    std::vector<FullBodyState>& states) {
    // std::cin.get();
    int start_id, goal_id;
    bool return_first_soln = true;
    bool forward_search = true;
    clock_t total_planning_time;
    bool isPlanFound;
    vector<double> stats;
    vector<string> stat_names;

    int planner_queues = 3;

    printf("\n");
    ROS_INFO("Initialize environment");
    m_env->reset();
    m_mha_planner.reset(new MHAPlanner(m_env.get(), planner_queues, forward_search));
    total_planning_time = clock();
    ROS_INFO("configuring request");
    if (!m_env->configureRequest(search_request, start_id, goal_id)){
        ROS_ERROR("Unable to configure request! Trial ID: %d", counter);

        total_planning_time = -1;
        int soln_cost = -1;
        packageMHAStats(stat_names, stats, soln_cost, 0, total_planning_time);
        for(unsigned int i=0; i<stats.size(); i++)
          stats[i] = -1;
        res.stats_field_names = stat_names;
        res.stats = stats;
        return true;
    }

      m_mha_planner->set_start(start_id);
      ROS_INFO("setting goal id to %d", goal_id);
      m_mha_planner->set_goal(goal_id);
      m_mha_planner->force_planning_from_scratch();
      vector<int> soln;
      int soln_cost;
      ROS_INFO("allocated time is %f",req.allocated_planning_time);
      MHAReplanParams replan_params(req.allocated_planning_time);
      replan_params.inflation_eps = EPS1;
      replan_params.anchor_eps = EPS2;
      replan_params.use_anchor = true;
      replan_params.return_first_solution = false;
      replan_params.final_eps = EPS1;

      replan_params.meta_search_type = static_cast<mha_planner::MetaSearchType>(req.meta_search_type);
      replan_params.planner_type = static_cast<mha_planner::PlannerType>(req.planner_type);
      isPlanFound = m_mha_planner->replan(&soln, replan_params, &soln_cost);

      if (isPlanFound) {
          ROS_INFO("Plan found. Moving on to reconstruction.");
          states =  m_env->reconstructPath(soln);
          total_planning_time = clock() - total_planning_time;
          packageMHAStats(stat_names, stats, soln_cost, states.size(),
              total_planning_time);
          res.stats_field_names = stat_names;
          res.stats = stats;
      } else {
          packageMHAStats(stat_names, stats, soln_cost, states.size(),
              total_planning_time);
          res.stats_field_names = stat_names;
          res.stats = stats;
          ROS_INFO("No plan found!");
      }
      // if(m_params.run_trajectory) {
      //     ROS_INFO("Running trajectory!");
      //     runTrajectory(states);
      // }
      return true;
}

bool EnvInterfaces::planPathCallback(GetMobileArmPlan::Request &req, 
                                     GetMobileArmPlan::Response &res)
{
    boost::unique_lock<boost::mutex> lock(mutex);

    SearchRequestParamsPtr search_request = make_shared<SearchRequestParams>();
    search_request->initial_epsilon = req.initial_eps;
    search_request->final_epsilon = req.final_eps;
    search_request->decrement_epsilon = req.dec_eps;
    search_request->base_start = req.body_start;
    search_request->left_arm_start = LeftContArmState(req.larm_start);
    search_request->right_arm_start = RightContArmState(req.rarm_start);
    search_request->underspecified_start = req.underspecified_start;

    search_request->base_goal = req.body_goal;
    search_request->left_arm_goal = LeftContArmState(req.larm_goal);
    search_request->right_arm_goal = RightContArmState(req.rarm_goal);

    KDL::Frame rarm_offset, larm_offset;
    rarm_offset.p.x(req.rarm_object.pose.position.x);
    rarm_offset.p.y(req.rarm_object.pose.position.y);
    rarm_offset.p.z(req.rarm_object.pose.position.z);
    larm_offset.p.x(req.larm_object.pose.position.x);
    larm_offset.p.y(req.larm_object.pose.position.y);
    larm_offset.p.z(req.larm_object.pose.position.z);

    rarm_offset.M = Rotation::Quaternion(req.rarm_object.pose.orientation.x, 
                                         req.rarm_object.pose.orientation.y, 
                                         req.rarm_object.pose.orientation.z, 
                                         req.rarm_object.pose.orientation.w);
    larm_offset.M = Rotation::Quaternion(req.larm_object.pose.orientation.x, 
                                         req.larm_object.pose.orientation.y, 
                                         req.larm_object.pose.orientation.z, 
                                         req.larm_object.pose.orientation.w);
    search_request->left_arm_object = larm_offset;
    search_request->right_arm_object = rarm_offset;
    search_request->xyz_tolerance = req.xyz_tolerance;
    search_request->roll_tolerance = req.roll_tolerance;
    search_request->pitch_tolerance = req.pitch_tolerance;
    search_request->yaw_tolerance = req.yaw_tolerance;
    search_request->planning_mode = req.planning_mode;

    RobotState start_pose(search_request->base_start, 
                          search_request->right_arm_start,
                          search_request->left_arm_start);

    // bins
    ContObjectState right_bin(1.5, 0.4, 0.6, 0.0, M_PI/2, 0.0);
    ContObjectState left_bin(1.5, 1.6, 0.6, 0.0, M_PI/2, 0.0);
    bool right_toward_goal = true;
    bool left_toward_goal = true;
    // TODO: greedy algorithm
    // set the object goal for each arm.
    assert(req.goal.size());

    // initialize the goal list
    m_goal_list.clear();
    m_goal_achieved = std::vector<bool>(req.goal.size(), false);
    for (auto& g : req.goal) {
        m_goal_list.push_back(ContObjectState(g));
    }

    int current_right_goal, current_left_goal;
    // current_right_goal = getClosestGoal(start_pose, true);
    // current_left_goal = getClosestGoal(start_pose, false, current_right_goal);
    getClosestGoalAssignment(start_pose, current_right_goal, current_left_goal);
    while(true) {
        ROS_INFO("Setting right goal id : %d, left goal id : %d",
            current_right_goal, current_left_goal);
        std::cin.get();
        if (current_right_goal >=0 && current_right_goal < m_goal_list.size()){
            search_request->r_obj_goal= std::make_shared<ContObjectState>(m_goal_list[current_right_goal]);
        }
        if (current_left_goal >=0 && current_left_goal < m_goal_list.size()){
            search_request->l_obj_goal= std::make_shared<ContObjectState>(m_goal_list[current_left_goal]);
        }
        if (!right_toward_goal)
            search_request->r_obj_goal= std::make_shared<ContObjectState>(right_bin);
        if (!left_toward_goal)
            search_request->l_obj_goal= std::make_shared<ContObjectState>(left_bin);
        res.stats_field_names.resize(18);
        res.stats.resize(18);
        int start_id, goal_id;
        static int counter = 0;
        bool isPlanFound;
        double total_planning_time = clock();
        bool forward_search = true;
        std::vector<FullBodyState> states;
        isPlanFound = runMHAPlanner(req, res, search_request, counter, states);
        counter++;
        if(!isPlanFound){
            ROS_ERROR("Holy shit!");
        }

        // get which one was achieved and mark it as done
        int goal_side_achieved = m_env->getGoalSideAchieved();
        if (goal_side_achieved == 1){ // right
            if (right_toward_goal){  // was an object
                m_goal_achieved[current_right_goal] = true;
            }
            right_toward_goal = !right_toward_goal;
        }
        else if (goal_side_achieved == 2){ // left
            if (left_toward_goal){ // was an object
                m_goal_achieved[current_left_goal] = true;
            }
            left_toward_goal = !left_toward_goal;
        }
        else
            ROS_ERROR("NO GOAL ACHIEVED?!?!?!?!");
        ROS_INFO("Achieved goal : %s", (goal_side_achieved==1)?"right":"left");

        if(std::all_of(m_goal_achieved.begin(), m_goal_achieved.end(),
                        [](bool ach){ return ach; })
            && left_toward_goal && right_toward_goal) {
            break;
        }

        RobotState end_pose = PathPostProcessor::createRobotState(states.back());
        getClosestGoalAssignment(start_pose, current_right_goal, current_left_goal);
        // release the objects
        search_request->r_obj_goal.reset();
        search_request->l_obj_goal.reset();
        search_request->base_start = end_pose.getContBaseState();
        search_request->right_arm_start = end_pose.right_arm();
        search_request->left_arm_start = end_pose.left_arm();
    }
    // for underspecified start
    // search_request->r_obj_start = ContObjectState(req.start);

    return true;
}

void EnvInterfaces::getClosestGoalAssignment(const
              monolithic_pr2_planner::RobotState& state,
              int& current_right_goal, int& current_left_goal)
{
    current_right_goal = -1;
    current_left_goal = -1;
    // returns the index of the closest *incomplete* goal in the m_goal_list
    // array
    ContObjectState r_arm_state = state.getRightObjectStateRelMap();
    ContObjectState l_arm_state = state.getLeftObjectStateRelMap();
    
    double min_dist_r = std::numeric_limits<double>::infinity();
    double min_dist_l = std::numeric_limits<double>::infinity();
    for (int i = 0; i < m_goal_list.size(); i++) {
        if (m_goal_achieved[i] == true)
            continue;
        double r_dist_to_goal = ContObjectState::distance(r_arm_state,
            m_goal_list[i]);
        double l_dist_to_goal = ContObjectState::distance(l_arm_state,
            m_goal_list[i]);
        if (r_dist_to_goal <= l_dist_to_goal && r_dist_to_goal < min_dist_r) {
            min_dist_r = r_dist_to_goal;
            current_right_goal = i;
        } else if (l_dist_to_goal < r_dist_to_goal && l_dist_to_goal <
            min_dist_l) {
            min_dist_l = l_dist_to_goal;
            current_left_goal = i;
        }
    }
}


void EnvInterfaces::packageStats(vector<string>& stat_names, 
                                 vector<double>& stats,
                                 int solution_cost,
                                 size_t solution_size,
                                 double total_planning_time)
{
    
    stat_names.resize(10);
    stats.resize(10);
    stat_names[0] = "total plan time";
    stat_names[1] = "initial solution planning time";
    stat_names[2] = "initial epsilon";
    stat_names[3] = "initial solution expansions";
    stat_names[4] = "final epsilon planning time";
    stat_names[5] = "final epsilon";
    stat_names[6] = "solution epsilon";
    stat_names[7] = "expansions";
    stat_names[8] = "solution cost";
    stat_names[9] = "path length";

    // TODO fix the total planning time
    //stats[0] = totalPlanTime;
    // TODO: Venkat. Handle the inital/final solution eps correctly when this becomes anytime someday.
    /*
    stats[0] = total_planning_time/static_cast<double>(CLOCKS_PER_SEC);
    stats[1] = m_ara_planner->get_initial_eps_planning_time();
    stats[2] = m_ara_planner->get_initial_eps();
    stats[3] = m_ara_planner->get_n_expands_init_solution();
    stats[4] = m_ara_planner->get_final_eps_planning_time();
    stats[5] = m_ara_planner->get_final_epsilon();
    stats[6] = m_ara_planner->get_solution_eps();
    stats[7] = m_ara_planner->get_n_expands();
    stats[8] = static_cast<double>(solution_cost);
    stats[9] = static_cast<double>(solution_size);
    */
    vector<PlannerStats> planner_stats;
    m_mha_planner->get_search_stats(&planner_stats);
    // Take stats only for the first solution, since this is not anytime currently
    stats[0] = planner_stats[0].time;
    stats[1] = stats[0];
    stats[2] = EPS1*EPS2;
    stats[3] = planner_stats[0].expands;
    stats[4] = stats[0];
    stats[5] = stats[2];
    stats[6] = stats[2];
    stats[7] = stats[3];
    stats[8] = static_cast<double>(planner_stats[0].cost);
    stats[9] = static_cast<double>(solution_size);
}

void EnvInterfaces::packageMHAStats(vector<string>& stat_names,
                                 vector<double>& stats,
                                 int solution_cost,
                                 size_t solution_size,
                                 double total_planning_time){
    stat_names.resize(10);
    stats.resize(10);
    stat_names[0] = "total plan time";
    stat_names[1] = "initial solution planning time";
    stat_names[2] = "epsilon 1";
    stat_names[3] = "initial solution expansions";
    stat_names[4] = "final epsilon planning time";
    stat_names[5] = "epsilon 2";
    stat_names[6] = "solution epsilon";
    stat_names[7] = "expansions";
    stat_names[8] = "solution cost";
    stat_names[9] = "path length";

    vector<PlannerStats> planner_stats;
    m_mha_planner->get_search_stats(&planner_stats);
    if(planner_stats.empty()){
      stats[0] = -1;
      stats[1] = -1;
      stats[2] = -1;
      stats[3] = -1;
      stats[4] = -1;
      stats[5] = -1;
      stats[6] = -1;
      stats[7] = -1;
      stats[8] = -1;
      stats[9] = -1;
    }
    else{
      // Take stats only for the first solution, since this is not anytime currently
      stats[0] = planner_stats[0].time;
      stats[1] = stats[0];
      stats[2] = EPS1*EPS2;
      stats[3] = planner_stats[0].expands;
      stats[4] = stats[0];
      stats[5] = stats[2];
      stats[6] = stats[2];
      stats[7] = stats[3];
      stats[8] = static_cast<double>(planner_stats[0].cost);
      stats[9] = static_cast<double>(solution_size);
    }
}

bool EnvInterfaces::bindCollisionSpaceToTopic(string topic_name){
    m_collision_space_interface->bindCollisionSpaceToTopic(topic_name, 
                                                          m_tf, 
                                                          m_params.ref_frame);
    return true;
}

void EnvInterfaces::bindNavMapToTopic(string topic){
    sleep(3.0);//TODO: ??!!*U8084u
    m_nav_map = m_nodehandle.subscribe(topic, 1, &EnvInterfaces::loadNavMap, this);
}

void EnvInterfaces::crop2DMap(const nav_msgs::MapMetaData& map_info, const
    std::vector<unsigned char>& v, double new_origin_x, double new_origin_y,
                              double width, double height){
    ROS_DEBUG_NAMED(CONFIG_LOG, "to be cropped to : %f (width), %f (height)", width, height);
    vector<vector<unsigned char> > tmp_map(map_info.height);
    for (unsigned int i=0; i < map_info.height; i++){
        for (unsigned int j=0; j < map_info.width; j++){
            tmp_map[i].push_back(v[i*map_info.width+j]);
        }
    }

    double res = map_info.resolution;
    ROS_DEBUG_NAMED(CONFIG_LOG, "resolution : %f", res);
    int new_origin_x_idx = (new_origin_x-map_info.origin.position.x)/res;
    int new_origin_y_idx = (new_origin_y-map_info.origin.position.y)/res;
    int new_width = static_cast<int>((width/res) + 1 + 0.5);
    int new_height = static_cast<int>((height/res) + 1 + 0.5);
    ROS_DEBUG_NAMED(HEUR_LOG, "new origin: %d %d, new_width and new_height: %d %d",
                              new_origin_x_idx, new_origin_y_idx, new_width, 
                              new_height);
    ROS_DEBUG_NAMED(HEUR_LOG, "size of map %lu %lu", tmp_map.size(), 
                                                     tmp_map[0].size());

    vector<vector<unsigned char> > new_map(new_height);
    int row_count = 0;
    for (int i=new_origin_y_idx; i < new_origin_y_idx + new_height; i++){
        for (int j=new_origin_x_idx; j < new_origin_x_idx + new_width; j++){
            new_map[row_count].push_back(tmp_map[i][j]);
        }
        row_count++;
    }
    m_final_map.clear();
    m_cropped_map.clear();
    // m_final_map.resize(new_width * new_height);
    for (size_t i=0; i < new_map.size(); i++){
        for (size_t j=0; j < new_map[i].size(); j++){
            m_final_map.push_back(static_cast<signed char>(double(new_map[i][j])/255.0*100.0));
            m_cropped_map.push_back(new_map[i][j]);
        }
    }
    ROS_DEBUG_NAMED(HEUR_LOG, "size of final map: %lu", m_final_map.size());
}

void EnvInterfaces::loadNavMap(const nav_msgs::OccupancyGridPtr& map){
    boost::unique_lock<boost::mutex> lock(mutex);
    ROS_DEBUG_NAMED(CONFIG_LOG, "received navmap of size %u %u, resolution %f",
                    map->info.width, map->info.height, map->info.resolution);
    ROS_DEBUG_NAMED(CONFIG_LOG, "origin is at %f %f", map->info.origin.position.x,
                                                      map->info.origin.position.y);

    // look up the values from the occup grid parameters
    // This stuff is in cells.
    int dimX, dimY, dimZ;
    m_collision_space_interface->getOccupancyGridSize(dimX, dimY,
        dimZ);
    ROS_DEBUG_NAMED(CONFIG_LOG, "Size of OccupancyGrid : %d %d %d", dimX, dimY,
        dimZ);
    // This costmap_ros object listens to the map topic as defined
    // in the costmap_2d.yaml file.
    m_costmap_ros.reset(new costmap_2d::Costmap2DROS("costmap_2d", m_tf));

    // Get the underlying costmap in the cost_map object.
    // Publish for visualization. Publishing is done for the entire (uncropped) costmap.
    costmap_2d::Costmap2D cost_map;
    m_costmap_ros->getCostmapCopy(cost_map);

    // Normalize and convert to array.
    for (unsigned int j = 0; j < cost_map.getSizeInCellsY(); ++j)
    {
        for (unsigned int i = 0; i < cost_map.getSizeInCellsX(); ++i)
        {
            // Row major. X is row wise, Y is column wise.
            int c = cost_map.getCost(i,j);

            // Set unknowns to free space (we're dealing with static maps for
            // now)
            if (c == costmap_2d::NO_INFORMATION) {
                c = costmap_2d::FREE_SPACE;
            }
            // c = (c == (costmap_2d::NO_INFORMATION)) ? (costmap_2d::FREE_SPACE) : (c);

            // Re-set the cost.
            cost_map.setCost(i,j,c);
        }
    }

    // Re-inflate because we modified the unknown cells to be free space.
    // API : center point of window x, center point of window y, size_x ,
    // size_y
    cost_map.reinflateWindow(dimX*map->info.resolution/2, dimY*map->info.resolution/2, dimX*map->info.resolution, dimY*map->info.resolution);

    std::vector<unsigned char> uncropped_map;
    for (unsigned int j = 0; j < cost_map.getSizeInCellsY(); ++j)
    {
        for (unsigned int i = 0; i < cost_map.getSizeInCellsX(); ++i)
        {
          /*
            // Normalize the values from 0 to 100.
            // makes life easier when dealing with the heuristic later.
            uncropped_map.push_back(
                static_cast<unsigned char>(
                    static_cast<double>(cost_map.getCost(i,j))/UCHAR_MAX*100.0f)
                );
          */
          uncropped_map.push_back(cost_map.getCost(i,j));

        }
    }

    m_costmap_publisher->updateCostmapData(cost_map, m_costmap_ros->getRobotFootprint());

    // Publish the full costmap
    // topic : /monolithic_pr2_planner_node/inflated_obstacles (RViz: Grid
    // Cells)
    m_costmap_publisher->publishCostmap();
    // topic : /monolithic_pr2_planner_node/robot_footprint (RViz: polygon)
    m_costmap_publisher->publishFootprint();

    // TODO: Check if this is the right thing to do : Take the resolution from
    // the map for the occupancy grid's values.
    double width = dimX*map->info.resolution;
    double height = dimY*map->info.resolution;
    
    crop2DMap(map->info, uncropped_map, 0, 0, width, height);
    
    // Don't want to publish this.
    nav_msgs::OccupancyGrid costmap_pub;
    costmap_pub.header.frame_id = "/map";
    costmap_pub.header.stamp = ros::Time::now();
    costmap_pub.info.map_load_time = ros::Time::now();
    costmap_pub.info.resolution = map->info.resolution;
    // done in the crop function too.
    costmap_pub.info.width = (width/map->info.resolution+1 + 0.5);
    costmap_pub.info.height = (height/map->info.resolution+1 + 0.5);
    costmap_pub.info.origin.position.x = 0;
    costmap_pub.info.origin.position.y = 0;
    costmap_pub.data = m_final_map;

    // Publish the cropped version of the costmap; publishes
    // /monolithic_pr2_planner/costmap_pub
    ROS_INFO_NAMED(CONFIG_LOG, "Publishing the final map that's supposed to fit"
        " within the occupancy grid.");
    m_costmap_pub.publish(costmap_pub);

    m_collision_space_interface->update2DHeuristicMaps(m_cropped_map);

}

void EnvInterfaces::getRobotState(tf::TransformListener &tf_, BodyPose &body_pos, std::vector<double> &rangles, std::vector<double> &langles){
  tf::StampedTransform base_map_transform;
  sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage < sensor_msgs::JointState > ("joint_states");
  rangles.resize(7);
  langles.resize(7);

  rangles[0] = getJointAngle("r_shoulder_pan_joint",state);
  rangles[1] = getJointAngle("r_shoulder_lift_joint",state);
  rangles[2] = getJointAngle("r_upper_arm_roll_joint",state);
  rangles[3] = getJointAngle("r_elbow_flex_joint",state);
  rangles[4] = getJointAngle("r_forearm_roll_joint",state);
  rangles[5] = getJointAngle("r_wrist_flex_joint",state);
  rangles[6] = getJointAngle("r_wrist_roll_joint",state);

  langles[0] = getJointAngle("l_shoulder_pan_joint",state);
  langles[1] = getJointAngle("l_shoulder_lift_joint",state);
  langles[2] = getJointAngle("l_upper_arm_roll_joint",state);
  langles[3] = getJointAngle("l_elbow_flex_joint",state);
  langles[4] = getJointAngle("l_forearm_roll_joint",state);
  langles[5] = getJointAngle("l_wrist_flex_joint",state);
  langles[6] = getJointAngle("l_wrist_roll_joint",state);

  body_pos.z = getJointAngle("torso_lift_joint",state);

  bool done = false;
  while(!done){
    try {
      tf_.lookupTransform("map", "base_footprint", ros::Time(0), base_map_transform);
      body_pos.x = base_map_transform.getOrigin().x();
      body_pos.y = base_map_transform.getOrigin().y();
      body_pos.theta = 2 * atan2(base_map_transform.getRotation().getZ(), base_map_transform.getRotation().getW());
      done = true;
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("[EnvInterfaces] Is there a map? The map-robot transform failed. (%s)", ex.what());
      sleep(1);
    }
  }
}

double EnvInterfaces::getJointAngle(std::string name, sensor_msgs::JointStateConstPtr msg){
  for(unsigned int i=0; i<msg->name.size(); i++){
    if(msg->name[i] == name)
      return msg->position[i];
  }
  ROS_ERROR("joint doesn't exist! (exit)\n");
  exit(1);
}

/**
 * @brief calls the monolithic_trajectory controller
 * @details Converts the FullBodyState objects to a full_body_controller
 * message and subsequently calls the service given by the parameter.
 * 
 * @param controller_service The name of the service to be called
 * @param states The final path
 * @return status of the call
 */
void EnvInterfaces::runTrajectory(std::vector<FullBodyState>& states) {

    // Create the messages from the full body states
    trajectory_msgs::JointTrajectory arms_trajectory;
    trajectory_msgs::JointTrajectory body_trajectory;
    trajectory_msgs::JointTrajectory gripper_trajectory;

    // prepare arms trajectory
    arms_trajectory.joint_names.push_back("r_shoulder_pan_joint");
    arms_trajectory.joint_names.push_back("r_shoulder_lift_joint");
    arms_trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    arms_trajectory.joint_names.push_back("r_elbow_flex_joint");
    arms_trajectory.joint_names.push_back("r_forearm_roll_joint");
    arms_trajectory.joint_names.push_back("r_wrist_flex_joint");
    arms_trajectory.joint_names.push_back("r_wrist_roll_joint");
    arms_trajectory.joint_names.push_back("l_shoulder_pan_joint");
    arms_trajectory.joint_names.push_back("l_shoulder_lift_joint");
    arms_trajectory.joint_names.push_back("l_upper_arm_roll_joint");
    arms_trajectory.joint_names.push_back("l_elbow_flex_joint");
    arms_trajectory.joint_names.push_back("l_forearm_roll_joint");
    arms_trajectory.joint_names.push_back("l_wrist_flex_joint");
    arms_trajectory.joint_names.push_back("l_wrist_roll_joint");

    arms_trajectory.points.resize(states.size());
    body_trajectory.points.resize(states.size());
    gripper_trajectory.points.resize(states.size());
    for (size_t i = 0; i < states.size(); ++i) {
        // // Insert the right arm joint angles
        // auto it = arms_trajectory.points[i].positions.begin();
        // arms_trajectory.points[i].positions.insert(it,
        //     states[i].right_arm.begin(), states[i].right_arm.end());
        // // insert the left arm joint angles
        // it = arms_trajectory.points[i].positions.end();
        // arms_trajectory.points[i].positions.insert(it,
        //     states[i].left_arm.begin(), states[i].left_arm.end());

        // Insert the right arm joint angles
        for (int r_arm = 0; r_arm < 7; ++r_arm) {
            arms_trajectory.points[i].positions.push_back(states[i].right_arm[r_arm]);
        }
        // insert the left arm joint angles
        for (int l_arm = 0; l_arm < 7; ++l_arm) {
            arms_trajectory.points[i].positions.push_back(states[i].left_arm[l_arm]);
        }
        // ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f",
        //         arms_trajectory.points[i].positions[0],
        //         arms_trajectory.points[i].positions[1],
        //         arms_trajectory.points[i].positions[2],
        //         arms_trajectory.points[i].positions[3],
        //         arms_trajectory.points[i].positions[4],
        //         arms_trajectory.points[i].positions[5],
        //         arms_trajectory.points[i].positions[6],
        //         arms_trajectory.points[i].positions[7],
        //         arms_trajectory.points[i].positions[8],
        //         arms_trajectory.points[i].positions[9],
        //         arms_trajectory.points[i].positions[10],
        //         arms_trajectory.points[i].positions[11],
        //         arms_trajectory.points[i].positions[12],
        //         arms_trajectory.points[i].positions[13]
        //         );

        // insert the base X, Y, Z, Theta
        body_trajectory.points[i].positions.assign(states[i].base.begin(),
            states[i].base.end());

        // Set the gripper poses
        gripper_trajectory.points[i].positions.resize(2,0);
    }

    // Package into full body message
    full_body_controller::ExecutePath::Request req;
    full_body_controller::ExecutePath::Response res;

    req.trajectory = arms_trajectory;
    req.body_trajectory = body_trajectory;
    req.gripper_trajectory= gripper_trajectory;

    // Check if service has been advertised
    ros::service::waitForService(m_params.controller_service);
    ros::ServiceClient client =
    m_nodehandle.serviceClient<full_body_controller::ExecutePath>(m_params.controller_service,
        true);
    if (client.call(req, res)) {
        ROS_INFO("It ran! Oh yeah!!");
    } else {
        ROS_INFO("Trajectory failed.");
    }
}
