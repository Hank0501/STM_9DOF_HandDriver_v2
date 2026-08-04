#ifndef MY_FINGER_H
#define MY_FINGER_H

#include "stdbool.h"

#define MAX_JOINT_NUM 5

// #define CONTACT_TYPE_POINT_NO_FC 0   // coresponding to wrench basis :
//                                      // [0;
//                                      //  0;
//                                      //  1;
//                                      //  0;
//                                      //  0;
//                                      //  0]
// #define CONTACT_TYPE_POINT_WITH_FC 1 // coresponding to wrench basis :
//                                      // [[1 0 0];
//                                      //  [0 1 0];
//                                      //  [0 0 1];
//                                      //  [0 0 0];
//                                      //  [0 0 0];
//                                      //  [0 0 0]]

// #define CONTACT_TYPE_SOFT_FINGER 2 // coresponding to wrench basis :
//                                    // [[1 0 0 0];
//                                    //  [0 1 0 0];
//                                    //  [0 0 1 0];
//                                    //  [0 0 0 0];
//                                    //  [0 0 0 0];
//                                    //  [0 0 0 1]]
/*

*

*/
// Structure declaration
// ==================================================================================================================== //

// Structure of homogeneous transformation matrix
typedef struct Transformation
{
    float R[3][3]; // Rotation, dimensionless
    float p[3];    // Position, in mm
    // float param[6]; // < roll (x-axis), pitch (y-axis), yaw (z-axis) (in rad), x, y, z (in mm) >
    // // The rotation matrix R is constructed following the ZYX Euler angle convention:
    // // R = Rz(yaw) * Ry(pitch) * Rx(roll)

    // float param[6]; // <x, y, z (in mm), roll (x-axis), pitch (y-axis), yaw (z-axis) (in rad) >
    // The rotation matrix R is constructed following the ZYX Euler angle convention:
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
} Transformation;

// Structure of Jacobian matrix for motion.
typedef struct Jacobian
{
    float JP[3][MAX_JOINT_NUM]; // linear Jacobian, in mm
    float JO[3][MAX_JOINT_NUM]; // angular Jacobian, dimensionless
} Jacobian;

// Structure of sensor st the fingertip
typedef struct Sensor
{
    // Transformation tf_BC;    // Contact frame to Object frame
    Transformation tf_SensC; // Contact frame to Sensor frame
    int contact_f_num;       // the number of force that can be transfered based on the contact model, the maximum value is 6
    bool wrench_basis[6];    // Wrench basis array, each element is either 1 or 0
                             // Wrench basis array forms wrench basis matrix following the rules:
                             // 1. We only consider the i-th non-zero element
                             // 2. The i-th non-zero element corresponds to a 6-D column vector that has a i-th non-zero element
                             // 3. Concatenate these column vectors horizontally to form a matrix
                             // Ex. :
                             // wrench_basis = {1, 1, 1, 0, 0, 1}
                             // corresponds to matrix :
                             //                        [ [1 0 0 0];
                             //                          [0 1 0 0];
                             //                          [0 0 1 0];
                             //                          [0 0 0 0];
                             //                          [0 0 0 0];
                             //                          [0 0 0 1] ]
} Sensor;

// Finger structure
// Contains all parameters and states of a finger
// Note that all constant parameters should be defined as 'const' to store them in Flash memory
// Note that all the indeices related to the joint number start from the joint nearest to the palm, i.e, the base frame.
typedef struct Finger
{
    // Constant model parameter
    // ------------------------------------------------ //
    const int joint_num;                      // Actual number of joints
    const float homing_offset[MAX_JOINT_NUM]; // Homing offset angle of each joint, in rad
    const float joint_min_ang[MAX_JOINT_NUM]; // minimum joint angle, in rad
    const float joint_max_ang[MAX_JOINT_NUM]; // maximum joint angle, in rad
    const float DH_alpha[MAX_JOINT_NUM];      // DH parameter alpha, in rad
    const float DH_d[MAX_JOINT_NUM];          // DH parameter d, in mm
    const float DH_a[MAX_JOINT_NUM];          // DH parameter a, in mm
    const float mass_link[MAX_JOINT_NUM];     // mass of each link, in kg
    const float P_link_bdf[MAX_JOINT_NUM][3]; // Position of the center of the mass of the link in link body frame
    const Transformation tf_PS;               // Transformation from Base frame of the finger to Palm frame
    const Transformation tf_FSens;            // Transformation from Sensor frame to Fingertip frame

    // Inertial of each link, [Ixx, Iyy, Izz, Ixy, Iyz, Ixz], in kg*(mm^2)
    // Note that here
    // inertia = [ [Ixx, Ixy, Ixz];
    //             [Iyx, Iyy, Iyz];
    //             [Izx, Izy, Izz] ]
    // Ixx = integral[ (y^2 + z^2)*dm ]
    // Iyy = integral[ (x^2 + z^2)*dm ]
    // Izz = integral[ (x^2 + y^2)*dm ]
    // Ixy = Iyx = -integral( x*y*dm )
    // Iyz = Izy = -integral( y*z*dm )
    // Ixz = Izx = -integral( x*z*dm )
    const float inertia_param[MAX_JOINT_NUM][6];

    // Variables related to joint servo.
    // ------------------------------------------------ //
    // volatile ServoXM4340 servo_list[MAX_JOINT_NUM]; // Joint servos
    float cur_q[MAX_JOINT_NUM];       // Current joint angles, in rad
    float cur_q_dot[MAX_JOINT_NUM];   // Current joint velocities, in rad/s
    float cur_current[MAX_JOINT_NUM]; // Current joint currents, in mA

    float prev_q[MAX_JOINT_NUM]; // q[n-1],上一拍位置,in rad
    float cur_q_dot_raw[MAX_JOINT_NUM];
    float prev_q_dot_raw[MAX_JOINT_NUM];         // x[n-1],上一拍濾波前的原始差分速度,in rad/s
    const float vel_filter_alpha[MAX_JOINT_NUM]; // 濾波器係數 α,依關節分開設定

    // Dynamic terms related to joint angle.
    // ------------------------------------------------ //
    // Note that finger has N joints, so there are
    //  - N+1 frames (frame 0 to frame N)
    //  - N+1 z-axis unit vectors (z0 to zN)
    //  - N links (link 1 to link N)
    //  - N homogeneous transformation matrices from frame i to frame i-1 (i=1~N)
    Transformation tf_joint[MAX_JOINT_NUM]; // Homogeneous transformation from frame i to frame i-1
    float z_axis[MAX_JOINT_NUM + 1][3];     // The z-axis unit vectors of frame 0 to frame N expressed in Base frame of the finger, dimensionless
    float P_frame[MAX_JOINT_NUM + 1][3];    // Position vectors of origins of frame 0 to frame N expressed in Base frame of the finger, in mm
    float P_link[MAX_JOINT_NUM][3];         // Position vectors of centers of mass of each link expressed in Base frame of the finger, in mm
    Jacobian jacob_link[MAX_JOINT_NUM];     // Geometry Jacobian at center of mass of each link
    Jacobian jacob_fingertip;               // Geometry Jacobian at origin of Fingertip frame
    float tau_grav[MAX_JOINT_NUM];          // Gravity compensation torque, in N-mm
    Transformation tf_SF;                   // Transformation from Fingertip frame to Base frame of the finger
    float dynamic_inertia_mat[MAX_JOINT_NUM][MAX_JOINT_NUM];

    // Variable related to fingertip sensor
    // ------------------------------------------------ //
    Sensor sensors;                        // Fingertip sensors
    Transformation tf_FC;                  // Transformation from Contact frame to Fingertip frame
    Transformation tf_PSens;               // Transformation from Sensor frame to Palm frame
    float hand_jacobian[6][MAX_JOINT_NUM]; // Hand Jacobian matrix
                                           // Note that only m*n is useful, where m depends on contact_type, n is equal to joint_num.
                                           // the linear part is in mm, the rotational part is dimensionless

    // Others
    // ------------------------------------------------ //
    float g_base[3]; // Gravity vector in Base frame of the finger, in m/s²

    // bool isReady;

    // // Control flags
    // volatile bool controlFlag;
    // volatile bool start_control;

    // // Error code
    // int error_code;
} Finger;

/*

*

*/
// Function prototypes
// ==================================================================================================================== //

void updateFingerDynamics(Finger *finger);

void updateFingerGravity(Finger *finger, const float g_palm[3]);

// void updateFingerSensor(Finger *finger, const float tf_BC_param[6], const float tf_SensC_param[6]);

void getFingerHandJacobian(Finger *finger);

void getFingerGravComp(Finger *finger);

void getFingerLinkState(Finger *finger);

void getInverseDynamics(const Finger *finger, const float *q_cmd_dot2, float *torque);

void getInertia(const float inertia_param[6], float inertia_matrix[3][3]);

bool jointLimitCheck(const Finger *finger);

void applyTransformation(const Transformation *tf_AB, const float p_B[3], float p_A[3]);

void applyRotation(const float rot_AB[3][3], const float v_B[3], float v_A[3]);

void inverseTransform(const Transformation *tf, Transformation *result);

void multiplyTransform(const Transformation *tf1, const Transformation *tf2, Transformation *result);

void matMul3x3(const float mat1[3][3], const float mat2[3][3], float result[3][3]);

void vec2matSkew(const float vec[3], float skew_mat[3][3]);

void transform2Param(const Transformation *tf, float param[6]);

void param2Transform(const float param[6], Transformation *tf);

void matTrsp3x3(const float mat[3][3], float mat_trsp[3][3]);

void crossProduct(const float vec1[3], const float vec2[3], float result[3]);

float innerProduct(const float *vec1, const float *vec2, int size);

void getTransformMatrix(const float alpha, const float d, const float a, const float theta, Transformation *tf_matrix);

#endif