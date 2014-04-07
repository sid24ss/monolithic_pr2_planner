#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <boost/scoped_ptr.hpp>

using namespace monolithic_pr2_planner;
using namespace boost;

GraphState::GraphState(RobotState robot_pose) : m_robot_pose(robot_pose),
m_check_simple(true), m_coord(12,0){updateStateFromRobotState();}

GraphState::GraphState(DiscObjectState obj_state, RobotState robot_pose):
 m_robot_pose(robot_pose), m_check_simple(true), m_coord(12,0){updateStateFromRobotState();}

bool GraphState::operator==(const GraphState& other){
    bool t1 =  m_coord == other.m_coord;
    bool t2 = (m_robot_pose.base_state() == other.m_robot_pose.base_state() &&
            m_robot_pose.getObjectStateRelBody() == other.m_robot_pose.getObjectStateRelBody() &&
            m_robot_pose.left_free_angle() == other.m_robot_pose.left_free_angle() &&
            m_robot_pose.right_free_angle() == other.m_robot_pose.right_free_angle());
    //if (t1 != t2){
    //    ROS_INFO("%d %d %d %d %d %d %d %d %d %d %d %d",
    //            m_coord[0],
    //            m_coord[1],
    //            m_coord[2],
    //            m_coord[3],
    //            m_coord[4],
    //            m_coord[5],
    //            m_coord[6],
    //            m_coord[7],
    //            m_coord[8],
    //            m_coord[9],
    //            m_coord[10],
    //            m_coord[11]);
    //    ROS_INFO("%d %d %d %d", 
    //             m_robot_pose.base_state().x(),
    //             m_robot_pose.base_state().y(),
    //             m_robot_pose.base_state().z(),
    //             m_robot_pose.base_state().theta());
    //    m_robot_pose.getObjectStateRelBody().printToDebug(MPRIM_LOG);

    //    ROS_INFO("%d %d %d %d %d %d %d %d %d %d %d %d",
    //            other.m_coord[0],
    //            other.m_coord[1],
    //            other.m_coord[2],
    //            other.m_coord[3],
    //            other.m_coord[4],
    //            other.m_coord[5],
    //            other.m_coord[6],
    //            other.m_coord[7],
    //            other.m_coord[8],
    //            other.m_coord[9],
    //            other.m_coord[10],
    //            other.m_coord[11]);
    //    ROS_INFO("%d %d %d %d", 
    //             other.m_robot_pose.base_state().x(),
    //             other.m_robot_pose.base_state().y(),
    //             other.m_robot_pose.base_state().z(),
    //             other.m_robot_pose.base_state().theta());
    //    other.m_robot_pose.getObjectStateRelBody().printToDebug(MPRIM_LOG);
    //    ROS_INFO("wtf");
    //}
    return t1;
}

bool GraphState::operator!=(const GraphState& other){
    return !(*this == other);
}

void GraphState::lazyApplyMPrim(const GraphStateMotion& mprim){
    // we need to let discobjectstate do value wrapping
    DiscObjectState tmp(
    m_coord[GraphStateElement::OBJ_X] + mprim[GraphStateElement::OBJ_X],
    m_coord[GraphStateElement::OBJ_Y] + mprim[GraphStateElement::OBJ_Y],
    m_coord[GraphStateElement::OBJ_Z] + mprim[GraphStateElement::OBJ_Z],
    m_coord[GraphStateElement::OBJ_ROLL] + mprim[GraphStateElement::OBJ_ROLL],
    m_coord[GraphStateElement::OBJ_PITCH] + mprim[GraphStateElement::OBJ_PITCH],
    m_coord[GraphStateElement::OBJ_YAW] + mprim[GraphStateElement::OBJ_YAW]);

    m_coord[GraphStateElement::OBJ_X] = tmp.x();
    m_coord[GraphStateElement::OBJ_Y] = tmp.y();
    m_coord[GraphStateElement::OBJ_Z] = tmp.z();
    m_coord[GraphStateElement::OBJ_ROLL] = tmp.roll();
    m_coord[GraphStateElement::OBJ_PITCH] = tmp.pitch();
    m_coord[GraphStateElement::OBJ_YAW] = tmp.yaw();


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

void GraphState::updateStateFromRobotState(){
    DiscObjectState obj = m_robot_pose.getObjectStateRelBody();
    m_coord[GraphStateElement::OBJ_X] = obj.x();
    m_coord[GraphStateElement::OBJ_Y] = obj.y();
    m_coord[GraphStateElement::OBJ_Z] = obj.z();
    m_coord[GraphStateElement::OBJ_ROLL] = obj.roll();
    m_coord[GraphStateElement::OBJ_PITCH] = obj.pitch();
    m_coord[GraphStateElement::OBJ_YAW] = obj.yaw();

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

/*! \brief applies a generic mprim vector to this graph state.
 */
bool GraphState::applyMPrim(const GraphStateMotion& mprim){
    // object state change
    static double time = 0;
    static int counter = 0;

    DiscObjectState obj_state(obj_x(), obj_y(), obj_z(), obj_roll(), obj_pitch(), obj_yaw());
    obj_state.x(obj_state.x() + mprim[GraphStateElement::OBJ_X]);
    obj_state.y(obj_state.y() + mprim[GraphStateElement::OBJ_Y]);
    obj_state.z(obj_state.z() + mprim[GraphStateElement::OBJ_Z]);
    obj_state.roll(obj_state.roll() + mprim[GraphStateElement::OBJ_ROLL]);
    obj_state.pitch(obj_state.pitch() + mprim[GraphStateElement::OBJ_PITCH]);
    obj_state.yaw(obj_state.yaw() + mprim[GraphStateElement::OBJ_YAW]);


    // free angle change
    double temptime = clock();
    RightContArmState right_arm(std::move(m_robot_pose.right_arm()));

    int r_fa = right_arm.getDiscFreeAngle() + mprim[GraphStateElement::R_FA];
    right_arm.setDiscFreeAngle(r_fa);
    m_robot_pose.right_arm(std::move(right_arm));
    r_fa = m_robot_pose.right_arm().getDiscFreeAngle();
    double old_fa = right_arm.getUpperArmRollAngle();
             

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
    ik_success = RobotState::computeRobotPose(obj_state, m_robot_pose, new_robot_pose);


    if (ik_success){
        m_robot_pose = *new_robot_pose;
        updateStateFromRobotState();

        DiscObjectState obj = m_robot_pose.getObjectStateRelBody();
        assert(obj_state.x() == obj.x());
        assert(obj_state.y() == obj.y());
        assert(obj_state.z() == obj.z());
        assert(obj_state.roll() == obj.roll());
        assert(obj_state.pitch() == obj.pitch());
        assert(obj_state.yaw() == obj.yaw());
        if (r_fa != m_robot_pose.right_arm().getDiscFreeAngle()){
            ROS_ERROR("angles don't match %d %d %f %f", r_fa, m_robot_pose.right_arm().getDiscFreeAngle(), old_fa, m_robot_pose.right_arm().getUpperArmRollAngle());
        }
        assert(r_fa == m_robot_pose.right_arm().getDiscFreeAngle());
        assert(l_fa == m_robot_pose.left_arm().getDiscFreeAngle());
    }
    counter++;
    //if (counter % 1000 == 0){
    //    ROS_WARN("outer time is %f, counter is %d", time, counter);
    //}
    return ik_success;
}

void GraphState::printToDebug(char* logger) const {
    DiscObjectState obj_state = m_robot_pose.getObjectStateRelBody();
    DiscObjectState map_obj_state = m_robot_pose.getObjectStateRelMap();


    ROS_DEBUG_NAMED(logger, "\tobject in map %d %d %d %d %d %d",
                    map_obj_state.x(),
                    map_obj_state.y(),
                    map_obj_state.z(),
                    map_obj_state.roll(),
                    map_obj_state.pitch(),
                    map_obj_state.yaw());

    ROS_DEBUG_NAMED(logger, "\t%d %d %d %d %d %d %d %d %d %d %d %d",
                    m_coord[GraphStateElement::OBJ_X],
                    m_coord[GraphStateElement::OBJ_Y],
                    m_coord[GraphStateElement::OBJ_Z],
                    m_coord[GraphStateElement::OBJ_ROLL],
                    m_coord[GraphStateElement::OBJ_PITCH],
                    m_coord[GraphStateElement::OBJ_YAW],
                    m_coord[GraphStateElement::R_FA],
                    m_coord[GraphStateElement::L_FA],
                    m_coord[GraphStateElement::BASE_X],
                    m_coord[GraphStateElement::BASE_Y],
                    m_coord[GraphStateElement::BASE_Z],
                    m_coord[GraphStateElement::BASE_THETA]);
}

void GraphState::printToInfo(char* logger) const {
    DiscObjectState obj_state = m_robot_pose.getObjectStateRelBody();
    DiscObjectState map_obj_state = m_robot_pose.getObjectStateRelMap();


    ROS_INFO_NAMED(logger, "\tobject in map %d %d %d %d %d %d",
                    map_obj_state.x(),
                    map_obj_state.y(),
                    map_obj_state.z(),
                    map_obj_state.roll(),
                    map_obj_state.pitch(),
                    map_obj_state.yaw());

    ROS_INFO_NAMED(logger, "\t%d %d %d %d %d %d %d %d %d %d %d %d",
                    m_coord[GraphStateElement::OBJ_X],
                    m_coord[GraphStateElement::OBJ_Y],
                    m_coord[GraphStateElement::OBJ_Z],
                    m_coord[GraphStateElement::OBJ_ROLL],
                    m_coord[GraphStateElement::OBJ_PITCH],
                    m_coord[GraphStateElement::OBJ_YAW],
                    m_coord[GraphStateElement::R_FA],
                    m_coord[GraphStateElement::L_FA],
                    m_coord[GraphStateElement::BASE_X],
                    m_coord[GraphStateElement::BASE_Y],
                    m_coord[GraphStateElement::BASE_Z],
                    m_coord[GraphStateElement::BASE_THETA]);
}

void GraphState::printContToDebug(char* logger) const {
    ContObjectState obj_state = m_robot_pose.getObjectStateRelBody();
    ContObjectState map_obj_state = m_robot_pose.getObjectStateRelMap();
    ContBaseState base_state = m_robot_pose.base_state();
    ROS_DEBUG_NAMED(logger, "object in map %f %f %f %f %f %f",
                    map_obj_state.x(),
                    map_obj_state.y(),
                    map_obj_state.z(),
                    map_obj_state.roll(),
                    map_obj_state.pitch(),
                    map_obj_state.yaw());
                    
    ROS_DEBUG_NAMED(logger, "\t%f %f %f %f %f %f %f %f %f %f %f %f",
                    obj_state.x(),
                    obj_state.y(),
                    obj_state.z(),
                    obj_state.roll(),
                    obj_state.pitch(),
                    obj_state.yaw(),
                    m_robot_pose.right_arm().getUpperArmRollAngle(),
                    m_robot_pose.left_arm().getUpperArmRollAngle(),
                    base_state.x(),
                    base_state.y(),
                    base_state.z(),
                    base_state.theta());
}

DiscObjectState GraphState::getObjectStateRelMapFromState() {
    DiscObjectState d_obj(obj_x(), obj_y(), obj_z(), obj_roll(), obj_pitch(), obj_yaw());
    ContObjectState c_obj(d_obj);
    
    KDL::Vector v(c_obj.x(), c_obj.y(), c_obj.z());
    KDL::Rotation rot = KDL::Rotation::RPY(c_obj.roll(), c_obj.pitch(), c_obj.yaw());
    KDL::Frame torso_to_link(rot, v);

    DiscBaseState d_base(base_x(), base_y(), base_z(), base_theta());
    ContBaseState c_base(d_base);
    BodyPose bp;
    bp.x = c_base.x();
    bp.y = c_base.y();
    bp.z = c_base.z();
    bp.theta = c_base.theta();

    KDL::Frame body_fk = m_robot_pose.right_arm().getArmModel()->computeBodyFK(bp);
    KDL::Frame obj_in_map = body_fk * torso_to_link;

    double roll, pitch, yaw;
    obj_in_map.M.GetRPY(roll, pitch, yaw);
    ContObjectState obj_map(obj_in_map.p.x(), obj_in_map.p.y(), obj_in_map.p.z(), roll, pitch, yaw);
    return DiscObjectState(obj_map);
}


DiscObjectState GraphState::getObjectStateRelMap() const {
    return m_robot_pose.getObjectStateRelMap();
}

DiscObjectState GraphState::getObjectStateRelBody() const {
    return m_robot_pose.getObjectStateRelBody();
}

