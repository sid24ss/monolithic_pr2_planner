#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/scoped_ptr.hpp>

using namespace monolithic_pr2_planner;
using namespace boost;

GraphState::GraphState(std::vector<double> cont_state):m_coord(GRAPH_STATE_SIZE,0){
    ContObjectState r_c_obj(cont_state[GraphStateElement::R_OBJ_X],
                          cont_state[GraphStateElement::R_OBJ_Y],
                          cont_state[GraphStateElement::R_OBJ_Z],
                          cont_state[GraphStateElement::R_OBJ_ROLL],
                          cont_state[GraphStateElement::R_OBJ_PITCH],
                          cont_state[GraphStateElement::R_OBJ_YAW]);
    ContObjectState l_c_obj(cont_state[GraphStateElement::L_OBJ_X],
                          cont_state[GraphStateElement::L_OBJ_Y],
                          cont_state[GraphStateElement::L_OBJ_Z],
                          cont_state[GraphStateElement::L_OBJ_ROLL],
                          cont_state[GraphStateElement::L_OBJ_PITCH],
                          cont_state[GraphStateElement::L_OBJ_YAW]);
    ContBaseState c_base(cont_state[GraphStateElement::BASE_X],
                         cont_state[GraphStateElement::BASE_Y],
                         cont_state[GraphStateElement::BASE_Z],
                         cont_state[GraphStateElement::BASE_THETA]);
    vector<double> arm(7,0);
    arm[Joints::UPPER_ARM_ROLL] = cont_state[GraphStateElement::R_FA];
    RightContArmState right_arm(arm);
    arm[Joints::UPPER_ARM_ROLL] = cont_state[GraphStateElement::L_FA];
    LeftContArmState left_arm(arm);

    RobotState robot_state(c_base, right_arm, left_arm);
    RobotPosePtr new_state;
    // bool ik_success = RobotState::computeRobotPose(c_obj, robot_state, new_state);
    bool ik_success = RobotState::computeRobotPose(r_c_obj, l_c_obj,
                                                robot_state, new_state);
    if (!ik_success){
        ROS_ERROR("couldn't generate robot state from graph state!");
    }
    m_robot_pose = *new_state;
    updateStateFromRobotState();
}

GraphState::GraphState(RobotState robot_pose) :
    m_robot_pose(robot_pose),
    m_coord(GRAPH_STATE_SIZE,0)
{
    updateStateFromRobotState();
}

// GraphState::GraphState(DiscObjectState obj_state, RobotState robot_pose):
//  m_robot_pose(robot_pose), m_check_simple(true), m_coord(GRAPH_STATE_SIZE,0)
//  {updateStateFromRobotState();}

bool GraphState::operator==(const GraphState& other){
    bool t1 =  m_coord == other.m_coord;
    return t1;
}

bool GraphState::operator!=(const GraphState& other){
    return !(*this == other);
}

// only updates the graphstate info, not the robot state! causes a mismatch
// between the two that gets resolved in gettruecost
void GraphState::lazyApplyMPrim(const GraphStateMotion& mprim){
    // we need to let discobjectstate do value wrapping
    DiscObjectState r_tmp(
    m_coord[GraphStateElement::R_OBJ_X] + mprim[GraphStateElement::R_OBJ_X],
    m_coord[GraphStateElement::R_OBJ_Y] + mprim[GraphStateElement::R_OBJ_Y],
    m_coord[GraphStateElement::R_OBJ_Z] + mprim[GraphStateElement::R_OBJ_Z],
    m_coord[GraphStateElement::R_OBJ_ROLL] + mprim[GraphStateElement::R_OBJ_ROLL],
    m_coord[GraphStateElement::R_OBJ_PITCH] + mprim[GraphStateElement::R_OBJ_PITCH],
    m_coord[GraphStateElement::R_OBJ_YAW] + mprim[GraphStateElement::R_OBJ_YAW]
    );
    DiscObjectState l_tmp(
    m_coord[GraphStateElement::L_OBJ_X] + mprim[GraphStateElement::L_OBJ_X],
    m_coord[GraphStateElement::L_OBJ_Y] + mprim[GraphStateElement::L_OBJ_Y],
    m_coord[GraphStateElement::L_OBJ_Z] + mprim[GraphStateElement::L_OBJ_Z],
    m_coord[GraphStateElement::L_OBJ_ROLL] + mprim[GraphStateElement::L_OBJ_ROLL],
    m_coord[GraphStateElement::L_OBJ_PITCH] + mprim[GraphStateElement::L_OBJ_PITCH],
    m_coord[GraphStateElement::L_OBJ_YAW] + mprim[GraphStateElement::L_OBJ_YAW]
    );

    m_coord[GraphStateElement::R_OBJ_X] = r_tmp.x();
    m_coord[GraphStateElement::R_OBJ_Y] = r_tmp.y();
    m_coord[GraphStateElement::R_OBJ_Z] = r_tmp.z();
    m_coord[GraphStateElement::R_OBJ_ROLL] = r_tmp.roll();
    m_coord[GraphStateElement::R_OBJ_PITCH] = r_tmp.pitch();
    m_coord[GraphStateElement::R_OBJ_YAW] = r_tmp.yaw();
    
    m_coord[GraphStateElement::L_OBJ_X] = l_tmp.x();
    m_coord[GraphStateElement::L_OBJ_Y] = l_tmp.y();
    m_coord[GraphStateElement::L_OBJ_Z] = l_tmp.z();
    m_coord[GraphStateElement::L_OBJ_ROLL] = l_tmp.roll();
    m_coord[GraphStateElement::L_OBJ_PITCH] = l_tmp.pitch();
    m_coord[GraphStateElement::L_OBJ_YAW] = l_tmp.yaw();

    RightContArmState tmp1;
    tmp1.setDiscFreeAngle(m_coord[GraphStateElement::R_FA] + mprim[GraphStateElement::R_FA]);

    LeftContArmState tmp2;
    tmp2.setDiscFreeAngle(m_coord[GraphStateElement::L_FA] + mprim[GraphStateElement::L_FA]);

    m_coord[GraphStateElement::R_FA] = tmp1.getDiscFreeAngle();
    m_coord[GraphStateElement::L_FA] = tmp2.getDiscFreeAngle();

    DiscBaseState tmp3( 
    m_coord[GraphStateElement::BASE_X] + mprim[GraphStateElement::BASE_X],
    m_coord[GraphStateElement::BASE_Y] + mprim[GraphStateElement::BASE_Y],
    m_coord[GraphStateElement::BASE_Z] + mprim[GraphStateElement::BASE_Z],
    m_coord[GraphStateElement::BASE_THETA] + mprim[GraphStateElement::BASE_THETA]);

    m_coord[GraphStateElement::BASE_X] = tmp3.x();
    m_coord[GraphStateElement::BASE_Y] = tmp3.y();
    m_coord[GraphStateElement::BASE_Z] = tmp3.z();
    m_coord[GraphStateElement::BASE_THETA] = tmp3.theta();
}

// this updates the graph state to match the graph state
void GraphState::updateStateFromRobotState(){
    DiscObjectState r_obj = m_robot_pose.getRightObjectStateRelBody();
    DiscObjectState l_obj = m_robot_pose.getLeftObjectStateRelBody();

    m_coord[GraphStateElement::R_OBJ_X] = r_obj.x();
    m_coord[GraphStateElement::R_OBJ_Y] = r_obj.y();
    m_coord[GraphStateElement::R_OBJ_Z] = r_obj.z();
    m_coord[GraphStateElement::R_OBJ_ROLL] = r_obj.roll();
    m_coord[GraphStateElement::R_OBJ_PITCH] = r_obj.pitch();
    m_coord[GraphStateElement::R_OBJ_YAW] = r_obj.yaw();

    m_coord[GraphStateElement::L_OBJ_X] = l_obj.x();
    m_coord[GraphStateElement::L_OBJ_Y] = l_obj.y();
    m_coord[GraphStateElement::L_OBJ_Z] = l_obj.z();
    m_coord[GraphStateElement::L_OBJ_ROLL] = l_obj.roll();
    m_coord[GraphStateElement::L_OBJ_PITCH] = l_obj.pitch();
    m_coord[GraphStateElement::L_OBJ_YAW] = l_obj.yaw();

    RightContArmState right_arm(m_robot_pose.right_arm());
    int r_fa = right_arm.getDiscFreeAngle();
    m_coord[GraphStateElement::R_FA] = r_fa;

    LeftContArmState left_arm(m_robot_pose.left_arm());
    int l_fa = left_arm.getDiscFreeAngle();
    m_coord[GraphStateElement::L_FA] = l_fa;

    DiscBaseState base_state(m_robot_pose.base_state());
    m_coord[GraphStateElement::BASE_X] = base_state.x();
    m_coord[GraphStateElement::BASE_Y] = base_state.y();
    m_coord[GraphStateElement::BASE_Z] = base_state.z();
    m_coord[GraphStateElement::BASE_THETA] = base_state.theta();
}

// this runs IK and all that to make the robotstate for a given graph state
// match
bool GraphState::updateRobotStateFromGraphState(double r_arm, double l_arm){
    ContObjectState r_c_obj(m_coord[GraphStateElement::R_OBJ_X],
                          m_coord[GraphStateElement::R_OBJ_Y],
                          m_coord[GraphStateElement::R_OBJ_Z],
                          m_coord[GraphStateElement::R_OBJ_ROLL],
                          m_coord[GraphStateElement::R_OBJ_PITCH],
                          m_coord[GraphStateElement::R_OBJ_YAW]);
    ContObjectState l_c_obj(m_coord[GraphStateElement::L_OBJ_X],
                          m_coord[GraphStateElement::L_OBJ_Y],
                          m_coord[GraphStateElement::L_OBJ_Z],
                          m_coord[GraphStateElement::L_OBJ_ROLL],
                          m_coord[GraphStateElement::L_OBJ_PITCH],
                          m_coord[GraphStateElement::L_OBJ_YAW]);
    vector<double> arm(7,0);
    arm[Joints::UPPER_ARM_ROLL] = r_arm;
    RightContArmState right_arm(arm);
    arm[Joints::UPPER_ARM_ROLL] = l_arm;
    LeftContArmState left_arm(arm);

    ContBaseState base_state(m_coord[GraphStateElement::BASE_X],
                             m_coord[GraphStateElement::BASE_Y],
                             m_coord[GraphStateElement::BASE_Z],
                             m_coord[GraphStateElement::BASE_THETA]);

    RobotState robot_state(base_state, right_arm, left_arm);
    RobotPosePtr new_state;
    bool ik_success = RobotState::computeRobotPose(r_c_obj, l_c_obj, robot_state,
        new_state);
    if (!ik_success){
        ROS_ERROR("couldn't generate robot state from graph state! used %f %f", r_arm, l_arm);
    }
    m_robot_pose = *new_state;
    return ik_success;
}

/*! \brief applies a generic mprim vector to this graph state.
 */
bool GraphState::applyMPrim(const GraphStateMotion& mprim){
    // object state change
    static double time = 0;
    static int counter = 0;

    DiscObjectState r_obj_state(r_obj_x(), r_obj_y(), r_obj_z(), r_obj_roll(), r_obj_pitch(), r_obj_yaw());
    r_obj_state.x(r_obj_state.x() + mprim[GraphStateElement::R_OBJ_X]);
    r_obj_state.y(r_obj_state.y() + mprim[GraphStateElement::R_OBJ_Y]);
    r_obj_state.z(r_obj_state.z() + mprim[GraphStateElement::R_OBJ_Z]);
    r_obj_state.roll(r_obj_state.roll() + mprim[GraphStateElement::R_OBJ_ROLL]);
    r_obj_state.pitch(r_obj_state.pitch() + mprim[GraphStateElement::R_OBJ_PITCH]);
    r_obj_state.yaw(r_obj_state.yaw() + mprim[GraphStateElement::R_OBJ_YAW]);

    DiscObjectState l_obj_state(l_obj_x(), l_obj_y(), l_obj_z(), l_obj_roll(), l_obj_pitch(), l_obj_yaw());
    l_obj_state.x(l_obj_state.x() + mprim[GraphStateElement::L_OBJ_X]);
    l_obj_state.y(l_obj_state.y() + mprim[GraphStateElement::L_OBJ_Y]);
    l_obj_state.z(l_obj_state.z() + mprim[GraphStateElement::L_OBJ_Z]);
    l_obj_state.roll(l_obj_state.roll() + mprim[GraphStateElement::L_OBJ_ROLL]);
    l_obj_state.pitch(l_obj_state.pitch() + mprim[GraphStateElement::L_OBJ_PITCH]);
    l_obj_state.yaw(l_obj_state.yaw() + mprim[GraphStateElement::L_OBJ_YAW]);

    // free angle change
    double temptime = clock();
    RightContArmState right_arm(std::move(m_robot_pose.right_arm()));

    int r_fa = right_arm.getDiscFreeAngle() + mprim[GraphStateElement::R_FA];
    right_arm.setDiscFreeAngle(r_fa);
    m_robot_pose.right_arm(std::move(right_arm));
    r_fa = m_robot_pose.right_arm().getDiscFreeAngle();

    LeftContArmState left_arm(std::move(m_robot_pose.left_arm()));
    int l_fa = left_arm.getDiscFreeAngle() + mprim[GraphStateElement::L_FA];
    left_arm.setDiscFreeAngle(l_fa);
    m_robot_pose.left_arm(std::move(left_arm));

    // base change
    DiscBaseState base_state(std::move(m_robot_pose.base_state()));
    base_state.x(base_state.x() + mprim[GraphStateElement::BASE_X]);
    base_state.y(base_state.y() + mprim[GraphStateElement::BASE_Y]);
    base_state.z(base_state.z() + mprim[GraphStateElement::BASE_Z]);
    base_state.theta(base_state.theta() + mprim[GraphStateElement::BASE_THETA]);
    m_robot_pose.base_state(std::move(base_state));

    // compute the new pose (runs IK)
    bool ik_success = true;
    RobotPosePtr new_robot_pose;
    time += (clock()-temptime)/(double)CLOCKS_PER_SEC;
    ContObjectState tmp1(obj_state);

    ik_success = RobotState::computeRobotPose(r_obj_state, l_obj_state,
                                            m_robot_pose, new_robot_pose);

    if (ik_success){
        m_robot_pose = *new_robot_pose;
        updateStateFromRobotState();

        // DiscObjectState obj = m_robot_pose.getObjectStateRelBody();

        // assert(obj_state.x() == obj.x());
        // assert(obj_state.y() == obj.y());
        // assert(obj_state.z() == obj.z());
        // if (obj_state.roll() != obj.roll()) {
        //     ROS_ERROR("angles don't match %d %d", obj_state.roll(), obj.roll());
        //     return false;
        // }
        // assert(obj_state.roll() == obj.roll());
        // assert(obj_state.pitch() == obj.pitch());
        // assert(obj_state.yaw() == obj.yaw());
        // assert(r_fa == m_robot_pose.right_arm().getDiscFreeAngle());
        // assert(l_fa == m_robot_pose.left_arm().getDiscFreeAngle());
    }
    counter++;
    //if (counter % 1000 == 0){
    //    ROS_WARN("outer time is %f, counter is %d", time, counter);
    //}
    return ik_success;
}

vector<double> GraphState::getContCoords(){
    ContObjectState r_c_obj = getRightObjectStateRelBody();
    ContObjectState l_c_obj = getLeftObjectStateRelBody();
    double r_fa = m_robot_pose.right_arm().getUpperArmRollAngle();
    double l_fa = m_robot_pose.left_arm().getUpperArmRollAngle();
    ContBaseState c_base = m_robot_pose.base_state();
    vector<double> coord(GRAPH_STATE_SIZE, -1);
    coord[GraphStateElement::R_OBJ_X] = r_c_obj.x();
    coord[GraphStateElement::R_OBJ_Y] = r_c_obj.y();
    coord[GraphStateElement::R_OBJ_Z] = r_c_obj.z();
    coord[GraphStateElement::R_OBJ_ROLL] = r_c_obj.roll();
    coord[GraphStateElement::R_OBJ_PITCH] = r_c_obj.pitch();
    coord[GraphStateElement::R_OBJ_YAW] = r_c_obj.yaw();
    coord[GraphStateElement::R_FA] = r_fa;
    coord[GraphStateElement::L_OBJ_X] = l_c_obj.x();
    coord[GraphStateElement::L_OBJ_Y] = l_c_obj.y();
    coord[GraphStateElement::L_OBJ_Z] = l_c_obj.z();
    coord[GraphStateElement::L_OBJ_ROLL] = l_c_obj.roll();
    coord[GraphStateElement::L_OBJ_PITCH] = l_c_obj.pitch();
    coord[GraphStateElement::L_OBJ_YAW] = l_c_obj.yaw();
    coord[GraphStateElement::L_FA] = l_fa;
    coord[GraphStateElement::BASE_X] = c_base.x();
    coord[GraphStateElement::BASE_Y] = c_base.y();
    coord[GraphStateElement::BASE_Z] = c_base.z();
    coord[GraphStateElement::BASE_THETA] = c_base.theta();
    return coord;
}

void GraphState::printToDebug(char* logger) const {
    // DiscObjectState obj_state = m_robot_pose.getObjectStateRelBody();
    // DiscObjectState map_r_obj_state = m_robot_pose.getRightObjectStateRelMap();


    // ROS_DEBUG_NAMED(logger, "\tobject in map %d %d %d %d %d %d",
    //                 map_obj_state.x(),
    //                 map_obj_state.y(),
    //                 map_obj_state.z(),
    //                 map_obj_state.roll(),
    //                 map_obj_state.pitch(),
    //                 map_obj_state.yaw());

    ROS_DEBUG_NAMED(logger, "\t%d %d %d %d %d %d %d, %d %d %d %d %d %d %d, %d %d %d %d",
                    m_coord[GraphStateElement::R_OBJ_X],
                    m_coord[GraphStateElement::R_OBJ_Y],
                    m_coord[GraphStateElement::R_OBJ_Z],
                    m_coord[GraphStateElement::R_OBJ_ROLL],
                    m_coord[GraphStateElement::R_OBJ_PITCH],
                    m_coord[GraphStateElement::R_OBJ_YAW],
                    m_coord[GraphStateElement::R_FA],
                    m_coord[GraphStateElement::L_OBJ_X],
                    m_coord[GraphStateElement::L_OBJ_Y],
                    m_coord[GraphStateElement::L_OBJ_Z],
                    m_coord[GraphStateElement::L_OBJ_ROLL],
                    m_coord[GraphStateElement::L_OBJ_PITCH],
                    m_coord[GraphStateElement::L_OBJ_YAW],
                    m_coord[GraphStateElement::L_FA],
                    m_coord[GraphStateElement::BASE_X],
                    m_coord[GraphStateElement::BASE_Y],
                    m_coord[GraphStateElement::BASE_Z],
                    m_coord[GraphStateElement::BASE_THETA]);
}

void GraphState::printToInfo(char* logger) const {
    // DiscObjectState obj_state = m_robot_pose.getObjectStateRelBody();
    // DiscObjectState map_obj_state = m_robot_pose.getObjectStateRelMap();


    // ROS_INFO_NAMED(logger, "\tobject in map %d %d %d %d %d %d",
    //                 map_obj_state.x(),
    //                 map_obj_state.y(),
    //                 map_obj_state.z(),
    //                 map_obj_state.roll(),
    //                 map_obj_state.pitch(),
    //                 map_obj_state.yaw());

    ROS_INFO_NAMED(logger, "\t%d %d %d %d %d %d %d, %d %d %d %d %d %d %d, %d %d %d %d",
                    m_coord[GraphStateElement::R_OBJ_X],
                    m_coord[GraphStateElement::R_OBJ_Y],
                    m_coord[GraphStateElement::R_OBJ_Z],
                    m_coord[GraphStateElement::R_OBJ_ROLL],
                    m_coord[GraphStateElement::R_OBJ_PITCH],
                    m_coord[GraphStateElement::R_OBJ_YAW],
                    m_coord[GraphStateElement::R_FA],
                    m_coord[GraphStateElement::L_OBJ_X],
                    m_coord[GraphStateElement::L_OBJ_Y],
                    m_coord[GraphStateElement::L_OBJ_Z],
                    m_coord[GraphStateElement::L_OBJ_ROLL],
                    m_coord[GraphStateElement::L_OBJ_PITCH],
                    m_coord[GraphStateElement::L_OBJ_YAW],
                    m_coord[GraphStateElement::L_FA],
                    m_coord[GraphStateElement::BASE_X],
                    m_coord[GraphStateElement::BASE_Y],
                    m_coord[GraphStateElement::BASE_Z],
                    m_coord[GraphStateElement::BASE_THETA]);
}

void GraphState::printContToDebug(char* logger) const {
    ContObjectState r_obj_state = m_robot_pose.getRightObjectStateRelBody();
    ContObjectState r_map_obj_state = m_robot_pose.getRightObjectStateRelMap();
    ContObjectState l_obj_state = m_robot_pose.getLeftObjectStateRelBody();
    ContObjectState l_map_obj_state = m_robot_pose.getLeftObjectStateRelMap();
    ContBaseState base_state = m_robot_pose.base_state();
    ROS_DEBUG_NAMED(logger, "right object in map %f %f %f %f %f %f",
                    r_map_obj_state.x(),
                    r_map_obj_state.y(),
                    r_map_obj_state.z(),
                    r_map_obj_state.roll(),
                    r_map_obj_state.pitch(),
                    r_map_obj_state.yaw());
    ROS_DEBUG_NAMED(logger, "\tleft object in map %f %f %f %f %f %f",
                    l_map_obj_state.x(),
                    l_map_obj_state.y(),
                    l_map_obj_state.z(),
                    l_map_obj_state.roll(),
                    l_map_obj_state.pitch(),
                    l_map_obj_state.yaw());
                    
    ROS_DEBUG_NAMED(logger, "\t%f %f %f %f %f %f %f, %f %f %f %f %f %f %f, %f %f %f %f",
                    r_obj_state.x(),
                    r_obj_state.y(),
                    r_obj_state.z(),
                    r_obj_state.roll(),
                    r_obj_state.pitch(),
                    r_obj_state.yaw(),
                    m_robot_pose.right_arm().getUpperArmRollAngle(),
                    l_obj_state.x(),
                    l_obj_state.y(),
                    l_obj_state.z(),
                    l_obj_state.roll(),
                    l_obj_state.pitch(),
                    l_obj_state.yaw(),
                    m_robot_pose.left_arm().getUpperArmRollAngle(),
                    base_state.x(),
                    base_state.y(),
                    base_state.z(),
                    base_state.theta());
}

// DiscObjectState GraphState::getObjectStateRelMapFromState() {
//     DiscObjectState d_obj(obj_x(), obj_y(), obj_z(), obj_roll(), obj_pitch(), obj_yaw());
//     ContObjectState c_obj(d_obj);
    
//     KDL::Vector v(c_obj.x(), c_obj.y(), c_obj.z());
//     KDL::Rotation rot = KDL::Rotation::RPY(c_obj.roll(), c_obj.pitch(), c_obj.yaw());
//     KDL::Frame torso_to_link(rot, v);

//     DiscBaseState d_base(base_x(), base_y(), base_z(), base_theta());
//     ContBaseState c_base(d_base);
//     BodyPose bp;
//     bp.x = c_base.x();
//     bp.y = c_base.y();
//     bp.z = c_base.z();
//     bp.theta = c_base.theta();

//     KDL::Frame body_fk = m_robot_pose.right_arm().getArmModel()->computeBodyFK(bp);
//     KDL::Frame obj_in_map = body_fk * torso_to_link;

//     double roll, pitch, yaw;
//     obj_in_map.M.GetRPY(roll, pitch, yaw);
//     ContObjectState obj_map(obj_in_map.p.x(), obj_in_map.p.y(), obj_in_map.p.z(), roll, pitch, yaw);
//     return DiscObjectState(obj_map);
// }


DiscObjectState GraphState::getLeftObjectStateRelMap() const {
    return m_robot_pose.getLeftObjectStateRelMap();
}

DiscObjectState GraphState::getLeftObjectStateRelBody() const {
    return m_robot_pose.getLeftObjectStateRelBody();
}

DiscObjectState GraphState::getRightObjectStateRelMap() const {
    return m_robot_pose.getRightObjectStateRelMap();
}

DiscObjectState GraphState::getRightObjectStateRelBody() const {
    return m_robot_pose.getRightObjectStateRelBody();
}

