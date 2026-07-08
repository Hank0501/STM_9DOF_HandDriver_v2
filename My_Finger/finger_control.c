#include "finger_control.h"
#include "my_finger.h"
#include <math.h>
#include <string.h>

#define DEG_2_RAD 0.01745329f
#define RPM_2_RADS 0.10471976f

// Compute desired contact force projected to the column space of hand jacobian+
// J_H has dimension of (4 * 3) for each finger
// Note that this function only suits for the case where contact force num > joint num
// and hand jacobian is full rank
void computeProjF_4x3_FR(const Finger *finger, FingerController *controller, float *force_input)
{
    int i, j, k;
    float JTJ[3][3] = {0};
    float invJTJ[3][3];
    float pinvJ[3][4];
    float projMat[4][4];

    // Assume the finger.joint_num is equal to 3 and controller->contact_f_num is equal to 4

    /* JTJ = J^T * J  (joint_num x joint_num) */
    for (i = 0; i < finger->joint_num; i++)
    {
        for (j = 0; j < finger->joint_num; j++)
        {
            JTJ[i][j] = 0.0;
            for (k = 0; k < finger->sensors.contact_f_num; k++)
            {
                JTJ[i][j] += finger->hand_jacobian[k][i] * finger->hand_jacobian[k][j];
            }
        }
    }

    /* Inverse of (joint_num x joint_num) matrix JTJ */
    // only for 3x3 case
    float det =
        JTJ[0][0] * (JTJ[1][1] * JTJ[2][2] - JTJ[1][2] * JTJ[2][1]) -
        JTJ[0][1] * (JTJ[1][0] * JTJ[2][2] - JTJ[1][2] * JTJ[2][0]) +
        JTJ[0][2] * (JTJ[1][0] * JTJ[2][1] - JTJ[1][1] * JTJ[2][0]);

    /* note：be careful that det approaches to 0  */
    float invDet;
    if (det != 0.0f)
        invDet = 1.0 / det;

    invJTJ[0][0] = (JTJ[1][1] * JTJ[2][2] - JTJ[1][2] * JTJ[2][1]) * invDet;
    invJTJ[0][1] = -(JTJ[0][1] * JTJ[2][2] - JTJ[0][2] * JTJ[2][1]) * invDet;
    invJTJ[0][2] = (JTJ[0][1] * JTJ[1][2] - JTJ[0][2] * JTJ[1][1]) * invDet;

    invJTJ[1][0] = -(JTJ[1][0] * JTJ[2][2] - JTJ[1][2] * JTJ[2][0]) * invDet;
    invJTJ[1][1] = (JTJ[0][0] * JTJ[2][2] - JTJ[0][2] * JTJ[2][0]) * invDet;
    invJTJ[1][2] = -(JTJ[0][0] * JTJ[1][2] - JTJ[0][2] * JTJ[1][0]) * invDet;

    invJTJ[2][0] = (JTJ[1][0] * JTJ[2][1] - JTJ[1][1] * JTJ[2][0]) * invDet;
    invJTJ[2][1] = -(JTJ[0][0] * JTJ[2][1] - JTJ[0][1] * JTJ[2][0]) * invDet;
    invJTJ[2][2] = (JTJ[0][0] * JTJ[1][1] - JTJ[0][1] * JTJ[1][0]) * invDet;

    /* pinvJ = inv(JTJ) * JT  (joint_num x CONT_FORCE_NUM) */
    for (i = 0; i < finger->joint_num; i++)
    {
        for (j = 0; j < finger->sensors.contact_f_num; j++)
        {
            pinvJ[i][j] = 0.0;
            for (k = 0; k < finger->joint_num; k++)
            {
                pinvJ[i][j] += invJTJ[i][k] * finger->hand_jacobian[j][k];
            }
        }
    }

    /* projMat = J * pinvJ  (CONT_FORCE_NUM x CONT_FORCE_NUM) */
    for (i = 0; i < finger->sensors.contact_f_num; i++)
    {
        for (j = 0; j < finger->sensors.contact_f_num; j++)
        {
            projMat[i][j] = 0.0;
            for (k = 0; k < finger->joint_num; k++)
            {
                projMat[i][j] += finger->hand_jacobian[i][k] * pinvJ[k][j];
            }
        }
    }

    /* f_cmd_proj = projMat * f_cmd_org */
    for (i = 0; i < finger->sensors.contact_f_num; i++)
    {
        controller->f_cmd_proj[i] = 0.0f;
        for (j = 0; j < finger->sensors.contact_f_num; j++)
            // controller->f_cmd_proj[i] += projMat[i][j] * controller->f_cmd_org[j];
            controller->f_cmd_proj[i] += projMat[i][j] * force_input[j];
    }
}

// Open loop control
void fingerForceCtrl_Open(const Finger *finger, FingerController *controller)
{
    // tau_d =  trsp(hand_jacobian) * f_cmd_proj
    for (int i = 0; i < finger->joint_num; i++)
    {
        controller->tau_d[i] = 0.0f;
        for (int j = 0; j < finger->sensors.contact_f_num; j++)
            controller->tau_d[i] += finger->hand_jacobian[j][i] * controller->f_cmd_proj[j] + 0.2f * finger->tau_grav[i]; // in N-mm
    }
}

void jointAngController_PID(const Finger *finger, FingerController *controller)
{

    // (1). Compute error of joint and endpoint
    // ------------------------------------------------ //
    for (int i = 0; i < finger->joint_num; i++)
    {
        controller->intergral_term[i] += controller->G_i[i] * controller->err_q[i] *
                                         (controller->control_period);            // integral error, in rad*s
        controller->err_q[i] = controller->traj_intp[i].q_cmd - finger->cur_q[i]; // in rad
        controller->err_endPT[0] = controller->p_cmd[0] - finger->tf_SF.p[0];     // x, in mm
        controller->err_endPT[1] = controller->p_cmd[1] - finger->tf_SF.p[1];     // y, in mm
        controller->err_endPT[2] = controller->p_cmd[2] - finger->tf_SF.p[2];     // z, in mm
    }

    // (2). Compute control law, PID controller
    // ------------------------------------------------ //
    for (int i = 0; i < finger->joint_num; i++)
    {
        // with gravity compensation
        controller->tau_d[i] = controller->K_d[i] * controller->err_q[i] + controller->intergral_term[i] -
                               controller->D_d[i] * finger->cur_q_dot[i] + finger->tau_grav[i] * 0.001f; // convert to N-m
    }
}

void jointAngController_InvDym(const Finger *finger, FingerController *controller)
{
    // (1). Compute error of joint and endpoint
    // ------------------------------------------------ //
    for (int i = 0; i < finger->joint_num; i++)
    {
        controller->err_q[i] = controller->traj_intp[i].q_cmd - finger->cur_q[i];             // in rad
        controller->err_q_dot[i] = controller->traj_intp[i].q_dot_cmd - finger->cur_q_dot[i]; // in rad/s

        // if (fabs(controller->err_q[i]) > 0.01f)
        //     controller->intergral_term[i] += controller->G_i[i] * controller->err_q[i] *
        //                                      (controller->control_period); // integral error, in rad*s
    }
    controller->err_endPT[0] = controller->p_cmd[0] - finger->tf_SF.p[0]; // x, in mm
    controller->err_endPT[1] = controller->p_cmd[1] - finger->tf_SF.p[1]; // y, in mm
    controller->err_endPT[2] = controller->p_cmd[2] - finger->tf_SF.p[2]; // z, in mm

    // controller->err_q[0] = 0.0f;
    // controller->err_q_dot[0] = 0.0f;
    // controller->err_q[1] = 0.0f;
    // controller->err_q_dot[1] = 0.0f;

    // (2). Update ESO and Compute control law
    // ------------------------------------------------ //
    for (int i = 0; i < finger->joint_num; i++)
    {
        // controller->org_control_law[i] = controller->traj_intp[i].q_dot2_cmd;

        // controller->org_control_law[i] = controller->K_d[i] * controller->err_q[i] +
        //                                  controller->D_d[i] * controller->err_q_dot[i] +
        //                                  controller->traj_intp[i].q_dot2_cmd; // in rad/s^2

        float weight = errFunction_calculate(controller->err_q[i], controller->bl_ang[i]);

        controller->org_control_law[i] = weight * controller->K_d[i] * controller->err_q[i] +
                                         weight * controller->D_d[i] * controller->err_q_dot[i] +
                                         controller->traj_intp[i].q_dot2_cmd; // in rad/s^2

        // controller->org_control_law[i] = controller->K_d[i] * falFunction_calculate(controller->err_q[i], 0.5f, 0.15f) +
        //                                  controller->D_d[i] * falFunction_calculate(controller->err_q_dot[i], 0.25f, 0.15f) +
        //                                  controller->traj_intp[i].q_dot2_cmd; // in rad/s^2

        controller->est_err_z1[i] = finger->cur_q[i] - controller->est_z1[i];

        // 儲存上一步的 dot 值
        controller->est_z1_dot_prev[i] = controller->est_z1_dot[i];
        controller->est_z2_dot_prev[i] = controller->est_z2_dot[i];
        controller->est_z3_dot_prev[i] = controller->est_z3_dot[i];

        controller->est_z1_dot[i] = controller->ESO_beta1 * controller->est_err_z1[i] + controller->est_z2[i];
        controller->est_z2_dot[i] = controller->ESO_beta2 * controller->est_err_z1[i] + controller->est_z3[i] + controller->org_control_law[i];
        controller->est_z3_dot[i] = controller->ESO_beta3 * controller->est_err_z1[i];

        // controller->est_z1[i] += z1_dot * controller->control_period;
        // controller->est_z2[i] += z2_dot * controller->control_period;
        // controller->est_z3[i] += z3_dot * controller->control_period;

        // Tustin 積分（梯形法）
        controller->est_z1[i] += (controller->est_z1_dot_prev[i] + controller->est_z1_dot[i]) * 0.5f * controller->control_period;
        controller->est_z2[i] += (controller->est_z2_dot_prev[i] + controller->est_z2_dot[i]) * 0.5f * controller->control_period;
        controller->est_z3[i] += (controller->est_z3_dot_prev[i] + controller->est_z3_dot[i]) * 0.5f * controller->control_period;

        // controller->new_control_law[i] = controller->org_control_law[i] - controller->est_z3[i];
        controller->new_control_law[i] = controller->org_control_law[i];
    }

    // (3). Compute torque command, using inverse dynamic method
    // ------------------------------------------------ //

    // float q_ctrl_dot2[MAX_JOINT_NUM];
    float torque_cmd[MAX_JOINT_NUM];
    // for (int i = 0; i < finger->joint_num; i++)
    // {
    //     q_ctrl_dot2[i] = controller->K_d[i] * controller->err_q[i] +

    //                      controller->D_d[i] * controller->err_q_dot[i] +
    //                      controller->traj_intp[i].q_dot2_cmd; // in rad/s^2
    // }
    getInverseDynamics(finger, controller->new_control_law, torque_cmd); // torque_cmd in mN-mm
    for (int i = 0; i < finger->joint_num; i++)
    {
        for (int j = 0; j < finger->joint_num; j++)
        {
            torque_cmd[i] -= finger->dynamic_inertia_mat[i][j] * controller->new_control_law[j];
        }
        torque_cmd[i] += finger->dynamic_inertia_mat[i][i] * controller->new_control_law[i];
    }

    for (int i = 0; i < finger->joint_num; i++)
    {
        controller->u_h[i] = 0.0f;
        for (int j = 0; j < finger->joint_num; j++)
        {
            controller->u_h[i] += finger->dynamic_inertia_mat[i][j] * controller->err_q_dot[j];
        }
    }

    for (int i = 0; i < finger->joint_num; i++)
    {
        // int sign = (controller->err_q_dot[i] >= 0) - (controller->err_q_dot[i] < 0);
        // controller->tau_d[i] = 0.0f;
        // controller->tau_d[i] = torque_cmd[i] * 0.001f +
        //                        finger->tau_grav[i];

        // controller->K_d[i] * controller->err_q[i] +
        // controller->D_d[i] * controller->err_q_dot[i] +
        //    controller->intergral_term[i]; // convert to N-mm
        // controller->tau_d[i] = torque_cmd[i] * 0.001f + finger->tau_grav[i] + 0.0f * 0.001f * controller->u_h[i] + 0.0f * sign;
        // controller->tau_d[i] = finger->tau_grav[i];

        float tau_ctrl = torque_cmd[i] * 0.001f;
        float tau_g = 0.7f * finger->tau_grav[i];
        float abs_tau_ctrl = fabsf(tau_ctrl);
        float abs_tau_g = fabsf(tau_g);

        if (( // tau_ctrl < 150.0f &&
                tau_g * tau_ctrl < 0.0f &&
                abs_tau_ctrl <= abs_tau_g) //||
            /* i == finger->joint_num - 1*/)
        {
            // float alpha = 0.1f;
            // int sign = (tau_ctrl >= 0) - (tau_ctrl < 0);
            controller->tau_d[i] = 0.1f * tau_ctrl;
            controller->test_flag[i] = 1;
        }
        else
        {
            controller->tau_d[i] = tau_ctrl + tau_g;
            controller->test_flag[i] = -1;
        }
    }
    // controller->tau_d[finger->joint_num - 1] = 0.0f;
    // controller->tau_d[finger->joint_num - 1] = torque_cmd[finger->joint_num - 1] * 0.001f + finger->tau_grav[finger->joint_num - 1]; // convert to N-mm
}

float falFunction_calculate(float error, float alpha, float delta)
{
    float abs_err = fabsf(error);

    if (abs_err <= delta)
    {
        return error / (powf(delta, 1 - alpha));
    }
    else
    {
        int sign = (error > 0) - (error < 0);
        return pow(abs_err, alpha) * sign;
    }
}

// float errFunction_calculate(float error, float sigma)
// {
//     float abs_err = fabsf(error);
//     if (sigma != 0.0f)
//         return 1 - expf(-abs_err / sigma);
//     else
//         return 0.0f;
// }

float errFunction_calculate(float error, float sigma)
{
    float abs_err = fabsf(error);
    if (sigma > 0.0f)
        return 0.5f * tanhf((abs_err - sigma * 0.9f) / (sigma * 0.05f)) + 0.5f;
    else
        return 0.0f;
}
