#ifndef FINGER_CONTROL_H
#define FINGER_CONTROL_H

#include "my_finger.h"
#include "my_trajectory_gen.h"
#include "my_force_gen.h"

// Different mode for variable: control_mode
#define FINGER_IDLE 0
#define FINGER_POS_CTRL 1
#define FINGER_FORCE_CTRL 2

// // Different states for variable: p_cmd_state
// #define POS_CMD_STATUS_UNDEFINED 0
// #define POS_CMD_STATUS_ERROR -1
// #define POS_CMD_STATUS_ACC 1
// #define POS_CMD_STATUS_CRUSE 2
// #define POS_CMD_STATUS_DEC 3
// #define POS_CMD_STATUS_END 4
// #define TRAJ_COMPLETED 5

#define WARNING_TRAJECTORY_QUEUE_FULL 0x01 // 0000 0000 0000 0001
#define WARNING_NO_TRAJ_IN_QUEUE 0x02      // 0000 0000 0000 0010
#define WARNING_TRAJ_ABORTED 0x04          // 0000 0000 0000 0100

#define BL_STATE_NORMAL 0
#define BL_STATE_CROSSING 1
#define DEADZONE_TRIG_CNTR 2

/*

*

*/
// Structure declaration
// ==================================================================================================================== //

// Structure for the finger controller, it's
//    (1). Contact force controller when it's in manipulation mode.
//    (2). Position controller when it's in reaching mode.
typedef struct FingerController
{
    // Force controller part
    // ---------------------------------- //
    // contact force vector, 6 dimension at most
    float cur_force[6];    // current fingertip force
    float f_cmd_buffer[6]; // desired force buffer, force_cmd received from CAN will save in here first
    float f_cmd_org[6];    // original desired force, only the first contact_f_num dimension is valid, in N or N-mm
    float f_cmd_proj[6];   // desired force vector after projection, only the first contact_f_num dimension is valid, in N or N-mm

    ForceVectorRamp f_ramp_force;  // 力通道 (fx,fy,fz) 的 ramp 狀態，dim <= 3
    ForceVectorRamp f_ramp_torque; // 力矩通道 (Taux,Tauy,Tauz) 的 ramp 狀態，dim <= 3
    float force_ramp_rate;         // 力的爬升速率, N/s
    float torque_ramp_rate;        // 力矩的爬升速率, N-mm/s

    int contact_f_num_force;  // f_cmd_org[] 前段有幾個是力通道 (<=3)
    int contact_f_num_torque; // f_cmd_org[] 後段有幾個是力矩通道 (<=3)
    // 注意: contact_f_num_force + contact_f_num_torque == sensors.contact_f_num

    float err_f[6];             // error of force controller
    float mat_force_proj[6][6]; // force projection matrix

    // Position controller part
    // ---------------------------------- //
    // Desired trajectory part
    // float x_d[MAX_TRAJ_NUM]; // desired x trajectory of end point, in mm
    // float y_d[MAX_TRAJ_NUM]; // desired y trajectory of end point, in mm
    // float z_d[MAX_TRAJ_NUM]; // desired z trajectory of end point, in mm
    // float t_d[MAX_TRAJ_NUM]; // timestamp of the desired trajectory, in s
    // float max_acc;           // maximum acceleration of end point, in mm/s^2
    // float cur_time;          // time start from the begining of the trajectory control, in s
    // int traj_d_num;          // number of point of desired trajectory
    // bool traj_d_isReady;     // desired trajectory ready flag

    // End point command part
    float p_cmd[3]; // position command of the origin of fingertip frame w.r.t palm frame
    // bool p_cmd_isUpdated; // position command ready flag
    // int p_cmd_state;      // status of position command generation

    // Angle command part
    TrajectoryInterpolator traj_intp[MAX_JOINT_NUM];
    float stalled_level[MAX_JOINT_NUM];
    float stalled_thrld_time;
    float stalled_thrld_err_q;
    float stalled_thrld_q_dot;
    float stalled_thrld_tau_m;
    // float q_cmd[MAX_JOINT_NUM];      // angle command of joint
    // float q_dot_cmd[MAX_JOINT_NUM];  // angular velocity command of joint
    // float q_dot2_cmd[MAX_JOINT_NUM]; // angular acceleration command of joint
    // bool ik_hasSolution; // true if inverse kinematic has solution

    // Controller gain part
    float K_d[MAX_JOINT_NUM];            // gontroller P gain
    float D_d[MAX_JOINT_NUM];            // gontroller D gain
    float G_i[MAX_JOINT_NUM];            // gontroller I gain
    float intergral_term[MAX_JOINT_NUM]; // I-controller term = G_i*err_sum

    float Ipos_offset[MAX_JOINT_NUM];
    float Ineg_offset[MAX_JOINT_NUM];
    // float I_viscous[MAX_JOINT_NUM];

    float ffd_acc[MAX_JOINT_NUM];
    float ffd_vel[MAX_JOINT_NUM];

    // error
    float err_q[MAX_JOINT_NUM];     // error of joint angle
    float err_q_dot[MAX_JOINT_NUM]; // error of angular velocity of joint
    float err_endPT[3];             // error of endpoint

    // Extended State Observer
    // ---------------------------------- //
    float org_control_law[MAX_JOINT_NUM];
    float new_control_law[MAX_JOINT_NUM];
    float est_z1[MAX_JOINT_NUM];
    float est_z2[MAX_JOINT_NUM];
    float est_z3[MAX_JOINT_NUM];
    float est_z1_dot[MAX_JOINT_NUM];
    float est_z2_dot[MAX_JOINT_NUM];
    float est_z3_dot[MAX_JOINT_NUM];
    float est_z1_dot_prev[MAX_JOINT_NUM];
    float est_z2_dot_prev[MAX_JOINT_NUM];
    float est_z3_dot_prev[MAX_JOINT_NUM];

    float est_err_z1[MAX_JOINT_NUM];
    float ESO_beta1;
    float ESO_beta2;
    float ESO_beta3;

    // backlash
    // ---------------------------------- //
    float bl_cycle[MAX_JOINT_NUM];
    float bl_ang[MAX_JOINT_NUM];
    float bl_current[MAX_JOINT_NUM]; // in mA
    int bl_state[MAX_JOINT_NUM];
    int bl_cntr[MAX_JOINT_NUM];
    int cur_sign[MAX_JOINT_NUM];
    int prev_sign[MAX_JOINT_NUM];
    int dir_changed[MAX_JOINT_NUM];
    int bl_trig_cntr[MAX_JOINT_NUM];

    // Shared variables
    // ---------------------------------- //
    float tau_d[MAX_JOINT_NUM]; // torque command, in N-mm
    float u_h[MAX_JOINT_NUM];
    // float tau_d_f[MAX_JOINT_NUM];      // torque command for force, in N-mm
    // float tau_d_p[MAX_JOINT_NUM];      // torque command for position, in N-mm
    float goal_current[MAX_JOINT_NUM]; // goal_current to send to the servo, in mA
    float prev_goal_current[MAX_JOINT_NUM];
    float control_period; // in s
    int control_mode;
    int control_mode_buffer;
    uint16_t warning_code[MAX_JOINT_NUM];
    // bool control_mode_trigger;

    int test_flag[MAX_JOINT_NUM];

} FingerController;

/*

*

*/
// Function prototypes
// ==================================================================================================================== //
void computeProjF_4x3_FR(const Finger *finger, FingerController *controller, float *force_input);

void fingerForceCtrl_Open(const Finger *finger, FingerController *controller);

void jointAngController_PID(const Finger *finger, FingerController *controller);

void jointAngController_InvDym(const Finger *finger, FingerController *controller);

void stalledLevelEval();

float falFunction_calculate(float error, float alpha, float delta);

float errFunction_calculate(float error, float sigma);

#endif