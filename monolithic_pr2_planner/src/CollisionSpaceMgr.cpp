#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <boost/foreach.hpp>
#include <stdexcept>
#include <vector>
#include <Eigen/Core>

using namespace monolithic_pr2_planner;
using namespace pr2_collision_checker;
using namespace boost;
using namespace std;

CollisionSpaceMgr::CollisionSpaceMgr(SBPLArmModelPtr right_arm,
                                     SBPLArmModelPtr left_arm){
    m_cspace = make_shared<PR2CollisionSpace>(right_arm,
                                              left_arm,
                                              m_occupancy_grid);
    if (!m_cspace->init()){
        ROS_ERROR("cspace failed to initialize! Exiting.");
        exit(1);
    }
    ROS_DEBUG_NAMED(INIT_LOG, "Launched collision space manager");

    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;

    pose.orientation.w = 1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    m_cspace->attachCube("tray", "r_wrist_roll_link", pose, .5,.5,.1);
}

/*! \brief Updates the internal collision map of the collision checker.
 */
void CollisionSpaceMgr::updateMap(const arm_navigation_msgs::CollisionMap& map){
    std::vector<Eigen::Vector3d> points;
    double maxX=0;
    double maxY=0;
    double maxZ=0;
    for (int i=0; i < (int)map.boxes.size(); i++){
        Eigen::Vector3d vect;
        vect << map.boxes[i].center.x,
        map.boxes[i].center.y,
        map.boxes[i].center.z;
        points.push_back(vect);
        if (map.boxes[i].center.x > maxX){
            maxX = map.boxes[i].center.x;
        }
        if (map.boxes[i].center.y > maxY){
            maxX = map.boxes[i].center.y;
        }
        if (map.boxes[i].center.z > maxZ){
            maxZ = map.boxes[i].center.z;
        }
    }
    m_occupancy_grid->addPointsToField(points);
}


bool CollisionSpaceMgr::loadMap(const vector<Eigen::Vector3d>& points){
    m_occupancy_grid->addPointsToField(points);
    return true;
}


bool CollisionSpaceMgr::isValid(RobotState& robot_pose){
    vector<double> l_arm;
    vector<double> r_arm;
    robot_pose.left_arm().getAngles(&l_arm);
    robot_pose.right_arm().getAngles(&r_arm);
    DiscBaseState discbody_pose = robot_pose.base_state();
    BodyPose body_pose = robot_pose.base_state().getBodyPose();

    double dist_temp;
    int debug_code;
    ROS_DEBUG_NAMED(CSPACE_LOG, "collision checking pose");
    ROS_DEBUG_NAMED(CSPACE_LOG, "body pose is %f %f %f", body_pose.x, 
                                body_pose.y, body_pose.z);
    robot_pose.printToDebug(CSPACE_LOG);
    Visualizer::pviz->visualizeRobot(r_arm, l_arm, body_pose, 150, 
                                    std::string("planner"), 0);
    return m_cspace->checkAllMotion(l_arm, r_arm, body_pose, true, dist_temp, 
                                    debug_code);
}

bool CollisionSpaceMgr::isValid(ContBaseState& base, RightContArmState& r_arm, 
                                LeftContArmState& l_arm){
    vector<double> l_arm_v;
    vector<double> r_arm_v;
    l_arm.getAngles(&l_arm_v);
    r_arm.getAngles(&r_arm_v);
    double dist_temp;
    int debug_code;
    BodyPose bp = base.body_pose();
    return m_cspace->checkAllMotion(l_arm_v, r_arm_v, bp, false, dist_temp, 
                                    debug_code);
}

bool CollisionSpaceMgr::isValidSimpleCheck(RobotState& robot_state){

    BodyPose body_pose = robot_state.base_state().getBodyPose();
    return m_cspace->simpleCheck(body_pose);
}

/*! \brief Given the transition data from a state expansion, this does a smart
 * collision check on the successor.
 *
 * If the motion primitive that generated this successor only moves the base,
 * then we don't need to collision check the arms against each other. If only
 * the arm moves, then we don't need to collision check the base for anything.
 *
 * TODO bounds check spine, bounds check base
 */
bool CollisionSpaceMgr::isValidSuccessor(const GraphState& successor,
                                         const TransitionData& t_data){
    // run the simple check first
    RobotState pose = successor.robot_pose();
    //if (isValidSimpleCheck(pose)){
    //    return true;
    //}

    vector<double> r_arm(7), l_arm(7);
    pose.right_arm().getAngles(&r_arm);
    pose.left_arm().getAngles(&l_arm);
    BodyPose body_pose = pose.base_state().getBodyPose();

    bool verbose = false;
    double dist;
    int debug;

    bool onlyBaseMotion = (t_data.motion_type() == MPrim_Types::BASE ||
                           t_data.motion_type() == MPrim_Types::BASE_ADAPTIVE);
    bool onlyArmMotion = (t_data.motion_type() == MPrim_Types::ARM ||
                          t_data.motion_type() == MPrim_Types::ARM_ADAPTIVE);


    if (onlyBaseMotion){
        return m_cspace->checkBaseMotion(l_arm, r_arm, body_pose, verbose, dist,
                                         debug);
    } else if (onlyArmMotion){
        bool isvalid = m_cspace->checkArmsMotion(l_arm, r_arm, body_pose, 
                                                 verbose, dist, debug);
        return isvalid;
    } else if (t_data.motion_type() == MPrim_Types::TORSO){
        return m_cspace->checkSpineMotion(l_arm, r_arm, body_pose, verbose, 
                                          dist, debug);
    } else {
        throw std::invalid_argument("not a valid motion primitive type");
    }

    return true;
}

/*! \brief Given the transition data from a state expansion, this collision
 * checks all continuous, intermediate states.
 *
 * We cannot just collision check the base state stored in RobotState IF the
 * type of motion is a base motion because RobotState only stores the discrete
 * base state. Thus, we need to look at the continuous base state that is also
 * stored in the transition data.
 */
bool CollisionSpaceMgr::isValidTransitionStates(const TransitionData& t_data){
    bool onlyBaseMotion = (t_data.motion_type() == MPrim_Types::BASE ||
                           t_data.motion_type() == MPrim_Types::BASE_ADAPTIVE);
    bool onlyArmMotion = (t_data.motion_type() == MPrim_Types::ARM ||
                          t_data.motion_type() == MPrim_Types::ARM_ADAPTIVE);
    for (auto& robot_state : t_data.interm_robot_steps()){
        vector<double> r_arm(7), l_arm(7);
        robot_state.right_arm().getAngles(&r_arm);
        robot_state.left_arm().getAngles(&l_arm);
        bool verbose = false;
        double dist;
        int debug;
    
        // let's check the validity of all intermediate poses
        if (onlyBaseMotion){
            BodyPose body_pose(std::move(robot_state.base_state().getBodyPose()));
            if (!m_cspace->checkBaseMotion(l_arm, r_arm, body_pose, verbose, dist,
                debug)){
                return false;
            }
        } else if (onlyArmMotion){
            ROS_DEBUG_NAMED(CSPACE_LOG, "skipping the intermediate points for arms because there are none.");
        } else {
            throw std::invalid_argument("not a valid motion primitive type");
        }
    }
    return true;
}

bool CollisionSpaceMgr::isValidContState(std::vector<double>& l_arm, std::vector<double>&
    r_arm, const std::vector<double> body){
    BodyPose body_pose;
    body_pose.x = body[BodyDOF::X];
    body_pose.y = body[BodyDOF::Y];
    body_pose.z = body[BodyDOF::Z];
    body_pose.theta = body[BodyDOF::THETA];
    bool verbose = false;
    int debug;
    double dist;
    if(!m_cspace->checkBaseMotion(l_arm, r_arm, body_pose, verbose, dist, debug))
        return false;
    return true;
}

void CollisionSpaceMgr::addAttachedObject(){
    double box_x, box_y, box_z;
    box_x = 1; box_y = 1; box_z = .1;

    // location of the origin of the box
    double body_frame_x = 0;
    double body_frame_y = 0;
    double body_frame_z = 0;


    for (int i=0; i < static_cast<int>(box_x/box_z); i++){
        KDL::Vector v(body_frame_x, body_frame_y + box_z*i, body_frame_z);
        m_object_points.push_back(v);
    }
    for (int i=0; i < static_cast<int>(box_y/box_z); i++){
        KDL::Vector v(body_frame_x + box_z*i, body_frame_y, body_frame_z);
        m_object_points.push_back(v);
    }
    for (int i=0; i < static_cast<int>(box_y/box_z); i++){
        KDL::Vector v(body_frame_x + box_z*i, body_frame_y + box_y, body_frame_z);
        m_object_points.push_back(v);
    }
    for (int i=0; i < static_cast<int>(box_y/box_z); i++){
        KDL::Vector v(body_frame_x+box_x, body_frame_y + box_z*i, body_frame_z);
        m_object_points.push_back(v);
    }
}

void CollisionSpaceMgr::visualizeAttachedObject(RobotState& robot_state){
    vector<double> t1, t2;
    t2 = robot_state.right_arm().getAngles();
    t1 = robot_state.left_arm().getAngles();
    BodyPose bp = robot_state.base_state().getBodyPose();
    std::vector<std::vector<double> > spheres;
    m_cspace->getAttachedObjectSpheres(t1, t2, bp, spheres);
    Visualizer::pviz->visualizeSpheres(spheres, 100, "object", .04);
}


void CollisionSpaceMgr::visualizeRobotAndObject(){
    ROS_INFO("visualizing attached objects");
    sleep(2);
    ContBaseState tmp1;
    RightContArmState tmp2;
    LeftContArmState tmp3;
    RobotState robot(tmp1, tmp2, tmp3);
    robot.visualize();
    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;

    pose.orientation.w = 1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;

    m_cspace->attachCube("tray", "r_wrist_roll_link", pose, .5,.5,.5);
    //visualizeAttachedObject(robot, m_object_points);
    vector<double> t1, t2;
    t1 = tmp2.getAngles();
    t2 = tmp3.getAngles();
    BodyPose bp = tmp1.body_pose();

    m_cspace->visualizeRobotCollisionModel(t1, t2, bp, "collisionmodel", 1);
    std::vector<std::vector<double> > spheres;
    m_cspace->getAttachedObjectSpheres(t1, t2, bp, spheres);
    Visualizer::pviz->visualizeSpheres(spheres, 100, "object", .1);
}
