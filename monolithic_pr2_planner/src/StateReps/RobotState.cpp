#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <kdl/frames.hpp>
#include <vector>

using namespace monolithic_pr2_planner;
using namespace boost;

IKFastPR2 RobotState::m_ikfast_solver;
int RobotState::ik_calls;
int RobotState::ik_time;
int RobotState::m_planning_mode;

bool RobotState::operator==(const RobotState& other){
    return (m_base_state == other.m_base_state &&
            m_right_arm == other.m_right_arm &&
            m_left_arm == other.m_left_arm);
}

bool RobotState::operator!=(const RobotState& other){
    return !(*this == other);
}

RobotState::RobotState(ContBaseState base_state, RightContArmState r_arm, 
                     LeftContArmState l_arm):
    m_base_state(base_state), 
    m_right_arm(r_arm), 
    m_left_arm(l_arm){
    bool left_arm_dominant = (m_planning_mode == PlanningModes::LEFT_ARM ||
                              m_planning_mode == PlanningModes::LEFT_ARM_MOBILE);
    if (left_arm_dominant){
        m_obj_state = l_arm.getObjectStateRelBody();
    } else {
        m_obj_state = r_arm.getObjectStateRelBody();
    }

    std::vector<double> angles;
    SBPLArmModelPtr arm_model;
    KDL::Frame offset;
    if (left_arm_dominant){
        m_left_arm.getAngles(&angles);
        arm_model = m_left_arm.getArmModel();
        offset = m_left_arm.getObjectOffset().Inverse();
    } else {
        m_right_arm.getAngles(&angles);
        arm_model = m_right_arm.getArmModel();
        offset = m_right_arm.getObjectOffset().Inverse();
    }

    // 10 is the link number for the wrist
    KDL::Frame to_wrist;
    arm_model->computeFK(angles, m_base_state.body_pose(), 10, &to_wrist);
    KDL::Frame f = to_wrist * offset;

    double wr,wp,wy;
    f.M.GetRPY(wr,wp,wy);

    
    m_obj_state_rel_map = ContObjectState(f.p.x(), f.p.y(), f.p.z(), wr, wp, wy);
}

DiscBaseState RobotState::base_state() const{
    return DiscBaseState(m_base_state);
}

void RobotState::base_state(const DiscBaseState& base_state) {
    m_base_state = ContBaseState(base_state); 
};

void RobotState::printToDebug(char* log_level) const {
    // ContBaseState base_state = m_base_state.getContBaseState();
    ROS_DEBUG_NAMED(log_level, "\tbase: %f %f %f %f", 
                   m_base_state.x(),
                   m_base_state.y(),
                   m_base_state.z(),
                   m_base_state.theta());
    std::vector<double> l_arm, r_arm;
    m_right_arm.getAngles(&r_arm);
    m_left_arm.getAngles(&l_arm);
    ROS_DEBUG_NAMED(log_level, "\tleft arm: %f %f %f %f %f %f %f",
                    l_arm[Joints::SHOULDER_PAN],
                    l_arm[Joints::SHOULDER_LIFT],
                    l_arm[Joints::UPPER_ARM_ROLL],
                    l_arm[Joints::ELBOW_FLEX],
                    l_arm[Joints::FOREARM_ROLL],
                    l_arm[Joints::WRIST_FLEX],
                    l_arm[Joints::WRIST_ROLL]);
    ROS_DEBUG_NAMED(log_level, "\tright arm: %f %f %f %f %f %f %f", 
                    r_arm[Joints::SHOULDER_PAN],
                    r_arm[Joints::SHOULDER_LIFT],
                    r_arm[Joints::UPPER_ARM_ROLL],
                    r_arm[Joints::ELBOW_FLEX],
                    r_arm[Joints::FOREARM_ROLL],
                    r_arm[Joints::WRIST_FLEX],
                    r_arm[Joints::WRIST_ROLL]);
}

void RobotState::printToInfo(char* log_level) const {
    // ContBaseState base_state = m_base_state.getContBaseState();
    ROS_INFO_NAMED(log_level, "\tbase: %f %f %f %f", 
                   m_base_state.x(),
                   m_base_state.y(),
                   m_base_state.z(),
                   m_base_state.theta());
    std::vector<double> l_arm, r_arm;
    m_right_arm.getAngles(&r_arm);
    m_left_arm.getAngles(&l_arm);
    ROS_INFO_NAMED(log_level, "\tleft arm: %f %f %f %f %f %f %f",
                    l_arm[0],
                    l_arm[1],
                    l_arm[2],
                    l_arm[3],
                    l_arm[4],
                    l_arm[5],
                    l_arm[6]);
    ROS_INFO_NAMED(log_level, "\tright arm: %f %f %f %f %f %f %f", 
                    r_arm[0],
                    r_arm[1],
                    r_arm[2],
                    r_arm[3],
                    r_arm[4],
                    r_arm[5],
                    r_arm[6]);
}

void RobotState::visualize(){
    std::vector<double> l_arm, r_arm;
    m_left_arm.getAngles(&l_arm);
    m_right_arm.getAngles(&r_arm);
    BodyPose body_pose = m_base_state.body_pose();
    Visualizer::pviz->visualizeRobot(r_arm, l_arm, body_pose, 150, 
                                    std::string("planner"), 0);
}



// this is a bit weird at the moment, but we use the arm angles as seed angles
// disc_obj_state is in body frame : Torso_lift_link?
bool RobotState::computeRobotPose(const DiscObjectState& disc_obj_state,
                                 const RobotState& seed_robot_pose,
                                 RobotPosePtr& new_robot_pose,
                                 bool free_angle_search){
    static double time = 0;
    static int counter = 0;

    ContObjectState obj_state = disc_obj_state.getContObjectState();

    KDL::Frame obj_frame;
    obj_frame.p.x(obj_state.x());
    obj_frame.p.y(obj_state.y());
    obj_frame.p.z(obj_state.z());
    obj_frame.M = KDL::Rotation::RPY(obj_state.roll(), 
                                     obj_state.pitch(),
                                     obj_state.yaw());
    KDL::Frame r_obj_to_wrist_offset = seed_robot_pose.right_arm().getObjectOffset();
    KDL::Frame l_obj_to_wrist_offset = seed_robot_pose.left_arm().getObjectOffset();

    KDL::Frame r_wrist_frame = obj_frame * r_obj_to_wrist_offset;
    KDL::Frame l_wrist_frame = obj_frame * l_obj_to_wrist_offset;

    // store the seed angles as the actual angles in case we don't compute for
    // the other arm. otherwise, computing a new robot pose would reset the
    // unused arm to angles of 0
    vector<double> r_seed(7,0), r_angles(7,0), l_seed(7,0), l_angles(7,0);
    seed_robot_pose.right_arm().getAngles(&r_seed);
    r_angles = r_seed;
    seed_robot_pose.left_arm().getAngles(&l_seed);
    l_angles = l_seed;

    ik_calls++;

    // decide which arms we need to run IK for
    bool use_right_arm = (m_planning_mode == PlanningModes::RIGHT_ARM ||
                          m_planning_mode == PlanningModes::DUAL_ARM ||
                          m_planning_mode == PlanningModes::RIGHT_ARM_MOBILE ||
                          m_planning_mode == PlanningModes::DUAL_ARM_MOBILE);
    bool use_left_arm = (m_planning_mode == PlanningModes::LEFT_ARM ||
                          m_planning_mode == PlanningModes::DUAL_ARM ||
                          m_planning_mode == PlanningModes::LEFT_ARM_MOBILE ||
                          m_planning_mode == PlanningModes::DUAL_ARM_MOBILE);
    if (!(use_right_arm || use_left_arm)){
        ROS_ERROR("what! not using any arm for IK??");
    }

    double temptime = clock();
    // TODO make this work for the left arm as well
#ifdef USE_KDL_SOLVER
    if (use_right_arm){
        SBPLArmModelPtr arm_model = seed_robot_pose.m_right_arm.getArmModel();
        bool ik_success = arm_model->computeFastIK(r_wrist_frame, r_seed, r_angles);
        if (!ik_success){
            return false;
        }
        // this is for searching over free angle. we don't want this, because
        // this can result in the successor state being different than the
        // source_state + mprim. ex., i'm looking at a bug now where the
        // successor state isn't found in the heap because IK is returning a
        // different state that hasn't been generated yet.
        //if (!ik_success){
        //    if (!arm_model->computeIK(r_wrist_frame, r_seed, r_angles)){
        //        return false;
        //    }
        //}
    }

    if (use_left_arm){
        SBPLArmModelPtr arm_model = seed_robot_pose.m_right_arm.getArmModel();
        bool ik_success = arm_model->computeFastIK(l_wrist_frame, l_seed, l_angles);
        if (!ik_success){
            return false;
        }
        //if (!ik_success){
        //    if (!arm_model->computeIK(l_wrist_frame, l_seed, l_angles)){
        //        return false;
        //    }
        //}
    }
#endif
#ifdef USE_IKFAST_SOLVER
    if (use_right_arm){
        double r_free_angle = r_seed[Joints::UPPER_ARM_ROLL];
        if (!m_ikfast_solver.ikRightArm(r_wrist_frame, r_free_angle, &r_angles)){
            return false;
        }
    }
    
    if (use_left_arm){
        double l_free_angle = l_seed[Joints::UPPER_ARM_ROLL];
        if (!m_ikfast_solver.ikLeftArm(l_wrist_frame, l_free_angle, &l_angles)){
            return false;
        }
    }
#endif
    
    time += (clock()-temptime)/(double)CLOCKS_PER_SEC;
    new_robot_pose = make_shared<RobotState>(seed_robot_pose.base_state(),
                                            RightContArmState(r_angles),
                                            LeftContArmState(l_angles));
    counter++;
    if (counter % 1000 == 0){
        ROS_WARN("ik time is %f, counter is %d", time, counter);
    }

    return true;
}

ContObjectState RobotState::getObjectStateRelMap() const {
    return m_obj_state_rel_map;
}
/*! \brief Gets the pose of the object in map frame.
ContObjectState RobotState::getObjectStateRelMap() const {
    static double hash_time = 0;
    static int hash_counts = 0;
    double temptime = clock();

    std::vector<double> angles;
    SBPLArmModelPtr arm_model;
    KDL::Frame offset;
    bool left_arm_dominant = (m_planning_mode == PlanningModes::LEFT_ARM ||
                              m_planning_mode == PlanningModes::LEFT_ARM_MOBILE);
    if (left_arm_dominant){
        m_left_arm.getAngles(&angles);
        arm_model = m_left_arm.getArmModel();
        offset = m_left_arm.getObjectOffset().Inverse();
    } else {
        m_right_arm.getAngles(&angles);
        arm_model = m_right_arm.getArmModel();
        offset = m_right_arm.getObjectOffset().Inverse();
    }

    // 10 is the link number for the wrist
    KDL::Frame to_wrist;
    arm_model->computeFK(angles, m_base_state.body_pose(), 10, &to_wrist);
    KDL::Frame f = to_wrist * offset;

    double wr,wp,wy;
    f.M.GetRPY(wr,wp,wy);

    hash_time += (clock()-temptime)/(double)CLOCKS_PER_SEC;
    hash_counts++;
    if (hash_counts % 10000 == 0){
        ROS_WARN("getobjectstate time %f count %d", hash_time, hash_counts);
    }

    return ContObjectState(f.p.x(), f.p.y(), f.p.z(), wr, wp, wy);
}
 */

ContObjectState RobotState::getObjectStateRelMap(ContBaseState base) const {
    std::vector<double> angles;
    SBPLArmModelPtr arm_model;
    KDL::Frame offset;
    bool left_arm_dominant = (m_planning_mode == PlanningModes::LEFT_ARM ||
                              m_planning_mode == PlanningModes::LEFT_ARM_MOBILE);
    if (left_arm_dominant){
        m_left_arm.getAngles(&angles);
        arm_model = m_left_arm.getArmModel();
        offset = m_left_arm.getObjectOffset().Inverse();
    } else {
        m_right_arm.getAngles(&angles);
        arm_model = m_right_arm.getArmModel();
        offset = m_right_arm.getObjectOffset().Inverse();
    }

    // 10 is the link number for the wrist
    KDL::Frame to_wrist;
    arm_model->computeFK(angles, base.body_pose(), 10, &to_wrist);
    KDL::Frame f = to_wrist * offset;

    double wr,wp,wy;
    f.M.GetRPY(wr,wp,wy);

    return ContObjectState(f.p.x(), f.p.y(), f.p.z(), wr, wp, wy);
}

/*! \brief Gets the pose of the object in base link frame.
 */
DiscObjectState RobotState::getObjectStateRelBody() const {
    return m_obj_state;
}

/*! \brief Given two robot states, determine how many interpolation steps there
 * should be. The delta step size is based on the RPY resolution of the object
 * pose and the XYZ spatial resolution. 
 */
int RobotState::numInterpSteps(const RobotState& start, const RobotState& end){
    ContObjectState start_obj = start.getObjectStateRelBody();
    ContObjectState end_obj = end.getObjectStateRelBody();
    ContBaseState start_base = start.base_state();
    ContBaseState end_base = end.base_state();
    
    double droll = shortest_angular_distance(start_obj.roll(), end_obj.roll());
    double dpitch = shortest_angular_distance(start_obj.pitch(), end_obj.pitch());
    double dyaw = shortest_angular_distance(start_obj.yaw(), end_obj.yaw());
    double d_rot = max(fabs(droll), fabs(dpitch));
    double dbase_theta = shortest_angular_distance(start_base.theta(),
                                                   end_base.theta());

    ROS_DEBUG_NAMED(MPRIM_LOG, "droll %f, dpitch %f, dyaw %f, dbase_theta %f",
                               droll, dpitch, dyaw, dbase_theta);
    d_rot = max(d_rot, fabs(dyaw));
    d_rot = max(d_rot, fabs(dbase_theta));

    double d_object = ContObjectState::distance(start_obj, end_obj);
    double d_base = ContBaseState::distance(ContBaseState(start.base_state()), 
                                         ContBaseState(end.base_state()));
    ROS_DEBUG_NAMED(MPRIM_LOG, "dobject %f, dbase %f", d_object, d_base);
    double d_dist = max(d_object, d_base);

    int rot_steps = static_cast<int>(d_rot/ContObjectState::getRPYResolution());
    int dist_steps = static_cast<int>(d_dist/ContBaseState::getXYZResolution());

    ROS_DEBUG_NAMED(MPRIM_LOG, "rot steps %d, dist_steps %d", rot_steps, dist_steps);

    int num_interp_steps = max(rot_steps, dist_steps);
    return num_interp_steps;
}

/*! \brief Given two robot states, we interpolate all steps in between them. The
 * delta step size is based on the RPY resolution of the object pose and the XYZ
 * resolution of the base. This returns a vector of RobotStates, which are
 * discretized to the grid (meaning if the arms move a lot, but the base barely
 * moves, the base discrete values will likely stay the same). This determines
 * the number of interpolation steps based on which part of the robot moves more
 * (arms or base).
 * TODO need to test this a lot more
 */
bool RobotState::workspaceInterpolate(const RobotState& start, const RobotState& end,
                                      vector<RobotState>* interp_steps){
    ContObjectState start_obj = start.getObjectStateRelBody();
    ContObjectState end_obj = end.getObjectStateRelBody();
    ContBaseState start_base = start.base_state();
    ContBaseState end_base = end.base_state();

    int num_interp_steps = numInterpSteps(start, end);
    vector<ContObjectState> interp_obj_steps;
    vector<ContBaseState> interp_base_steps;
    ROS_DEBUG_NAMED(MPRIM_LOG, "start obj");
    start_obj.printToDebug(MPRIM_LOG);
    ROS_DEBUG_NAMED(MPRIM_LOG, "end obj");
    end_obj.printToDebug(MPRIM_LOG);
    interp_obj_steps = ContObjectState::interpolate(start_obj, end_obj, 
                                                    num_interp_steps);
    interp_base_steps = ContBaseState::interpolate(start_base, end_base, 
                                                   num_interp_steps);
    assert(interp_obj_steps.size() == interp_base_steps.size());
    ROS_DEBUG_NAMED(MPRIM_LOG, "size of returned interp %lu", interp_obj_steps.size());

    // should at least return the same start and end poses
    if (num_interp_steps < 2){
        assert(interp_obj_steps.size() == 2);
    } else {
        assert(interp_obj_steps.size() == static_cast<size_t>(num_interp_steps));
    }

    for (size_t i=0; i < interp_obj_steps.size(); i++){
        interp_obj_steps[i].printToDebug(MPRIM_LOG);
        RobotState seed(interp_base_steps[i], start.right_arm(), start.left_arm());
        RobotPosePtr new_robot_state;
        if (!computeRobotPose(interp_obj_steps[i], seed, new_robot_state)){
            return false;
        }
        interp_steps->push_back(*new_robot_state);
    }

    return true;
}
