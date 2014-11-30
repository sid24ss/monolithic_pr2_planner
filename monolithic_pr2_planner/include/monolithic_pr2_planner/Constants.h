#pragma once
namespace monolithic_pr2_planner {
    const int GRAPH_STATE_SIZE = 18;
    class GraphStateElement {
        public:
            enum {R_OBJ_X,
                  R_OBJ_Y,
                  R_OBJ_Z,
                  R_OBJ_ROLL,
                  R_OBJ_PITCH,
                  R_OBJ_YAW,
                  R_FA,
                  L_OBJ_X,
                  L_OBJ_Y,
                  L_OBJ_Z,
                  L_OBJ_ROLL,
                  L_OBJ_PITCH,
                  L_OBJ_YAW,
                  L_FA,
                  BASE_X,
                  BASE_Y,
                  BASE_Z,
                  BASE_THETA};
    };
    class BodyDOF {
        public:
            enum {X, Y, Z, THETA};
    };

    class Joints { 
        public:
            enum {
                SHOULDER_PAN,
                SHOULDER_LIFT,
                UPPER_ARM_ROLL,
                ELBOW_FLEX,
                FOREARM_ROLL,
                WRIST_FLEX,
                WRIST_ROLL
            };
    };

    class ArmSide {
        public:
            enum {
                LEFT,
                RIGHT
            };
    };

    class ObjectPose {
        public:
            enum { X, Y, Z, ROLL, PITCH, YAW };
    };

    class Tolerances {
        public:
            enum { XYZ, ROLL, PITCH, YAW };
    };

    class MPrim_Types {
        public:
            enum { BASE, ARM, ARM_ADAPTIVE, BASE_ADAPTIVE, TORSO };
    };

    class PlanningModes {
        public:
            enum { BASE_ONLY, 
                   RIGHT_ARM, 
                   LEFT_ARM, 
                   DUAL_ARM, 
                   RIGHT_ARM_MOBILE, 
                   LEFT_ARM_MOBILE, 
                   DUAL_ARM_MOBILE,
                   TWO_ARM };
    };
}

