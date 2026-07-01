#include <math.h>
#include <string.h>
#include "my_finger.h"

#define DEG_2_RAD 0.01745329f
#define RPM_2_RADS 0.10471976f

/**
 * @brief  Update all finger dynamic terms based on current angle and sensor information.
 * @param  finger Pointer to the Finger structure
 * @retval None
 * @note Note that hand Jacobian is coomputed only when the finger is in contact with object.
 */
void updateFingerDynamics(Finger *finger)
{
    getFingerLinkState(finger);
    getFingerGravComp(finger);

    // Compute tf_FC and hand Jacobian only when the finger is in contact with object
    if (finger->sensors.contact_f_num != 0)
    {
        // param2Transform(finger->sensors.tf_SensC.param, &finger->sensors.tf_SensC);      // update the tf_SensC based on the parameters
        multiplyTransform(&finger->tf_FSens, &finger->sensors.tf_SensC, &finger->tf_FC); // get the tf_FC
        getFingerHandJacobian(finger);
    }
}

/**
 * @brief  Update g_base.
 * @param  finger Pointer to the Finger structure
 * @param  g_palm Gravity vector in palm frame, in m/s²
 * @retval  None
 */
void updateFingerGravity(Finger *finger, const float g_palm[3])
{
    // Get transpose of rotation part of tf_PS
    float R_PS_trsp[3][3];
    matTrsp3x3(finger->tf_PS.R, R_PS_trsp);

    // Update g_base
    applyRotation(R_PS_trsp, g_palm, finger->g_base);
}

// /**
//  * @brief  Update the homogeneous transformation matrices in the Sensor structure of the finger.
//  * @param  finger Pointer to the Finger structure
//  * @param  tf_BC_param Transformation from Contact frame to Object frame parameters < roll, pitch, yaw (in rad), x, y, z (in mm) >
//  * @param  tf_SensC_param Transformation from Contact frame to Sensor frame parameters < roll, pitch, yaw (in rad), x, y, z (in mm) >
//  * @retval  None
//  */
// void updateFingerSensor(Finger *finger, const float tf_BC_param[6], const float tf_SensC_param[6])
// {
//     float Cr_BC = cosf(tf_BC_param[0]); // cos(roll angle) of tf_BC
//     float Sr_BC = sinf(tf_BC_param[0]); // sin(roll angle) of tf_BC
//     float Cp_BC = cosf(tf_BC_param[1]); // cos(pitch angle) of tf_BC
//     float Sp_BC = sinf(tf_BC_param[1]); // sin(pitch angle) of tf_BC
//     float Cy_BC = cosf(tf_BC_param[2]); // cos(yaw angle) of tf_BC
//     float Sy_BC = sinf(tf_BC_param[2]); // sin(yaw angle) of tf_BC

//     float Cr_SensC = cosf(tf_SensC_param[0]); // cos(roll angle) of tf_SensC
//     float Sr_SensC = sinf(tf_SensC_param[0]); // sin(roll angle) of tf_SensC
//     float Cp_SensC = cosf(tf_SensC_param[1]); // cos(pitch angle) of tf_SensC
//     float Sp_SensC = sinf(tf_SensC_param[1]); // sin(pitch angle) of tf_SensC
//     float Cy_SensC = cosf(tf_SensC_param[2]); // cos(yaw angle) of tf_SensC
//     float Sy_SensC = sinf(tf_SensC_param[2]); // sin(yaw angle) of tf_SensC

//     // Update parameters of tf_BC and tf_SensC
//     for (int i = 0; i < 6; i++)
//     {
//         finger->sensors.tf_BC.param[i] = tf_BC_param[i];
//         finger->sensors.tf_SensC.param[i] = tf_SensC_param[i];
//     }

//     // The rotation matrix R is constructed following the ZYX Euler angle convention:
//     // R = Rz(yaw) * Ry(pitch) * Rx(roll)
//     // Update tf_BC
//     finger->sensors.tf_BC.R[0][0] = Cy_BC * Cp_BC;
//     finger->sensors.tf_BC.R[0][1] = Cy_BC * Sp_BC * Sr_BC - Sy_BC * Cr_BC;
//     finger->sensors.tf_BC.R[0][2] = Cy_BC * Sp_BC * Cr_BC + Sy_BC * Sr_BC;
//     finger->sensors.tf_BC.R[1][0] = Sy_BC * Cp_BC;
//     finger->sensors.tf_BC.R[1][1] = Sy_BC * Sp_BC * Sr_BC + Cy_BC * Cr_BC;
//     finger->sensors.tf_BC.R[1][2] = Sy_BC * Sp_BC * Cr_BC - Cy_BC * Sr_BC;
//     finger->sensors.tf_BC.R[2][0] = -Sp_BC;
//     finger->sensors.tf_BC.R[2][1] = Cp_BC * Sr_BC;
//     finger->sensors.tf_BC.R[2][2] = Cp_BC * Cr_BC;
//     finger->sensors.tf_BC.p[0] = tf_BC_param[3];
//     finger->sensors.tf_BC.p[1] = tf_BC_param[4];
//     finger->sensors.tf_BC.p[2] = tf_BC_param[5];

//     // Update tf_SensC
//     finger->sensors.tf_SensC.R[0][0] = Cy_SensC * Cp_SensC;
//     finger->sensors.tf_SensC.R[0][1] = Cy_SensC * Sp_SensC * Sr_SensC - Sy_SensC * Cr_SensC;
//     finger->sensors.tf_SensC.R[0][2] = Cy_SensC * Sp_SensC * Cr_SensC + Sy_SensC * Sr_SensC;
//     finger->sensors.tf_SensC.R[1][0] = Sy_SensC * Cp_SensC;
//     finger->sensors.tf_SensC.R[1][1] = Sy_SensC * Sp_SensC * Sr_SensC + Cy_SensC * Cr_SensC;
//     finger->sensors.tf_SensC.R[1][2] = Sy_SensC * Sp_SensC * Cr_SensC - Cy_SensC * Sr_SensC;
//     finger->sensors.tf_SensC.R[2][0] = -Sp_SensC;
//     finger->sensors.tf_SensC.R[2][1] = Cp_SensC * Sr_SensC;
//     finger->sensors.tf_SensC.R[2][2] = Cp_SensC * Cr_SensC;
//     finger->sensors.tf_SensC.p[0] = tf_SensC_param[3];
//     finger->sensors.tf_SensC.p[1] = tf_SensC_param[4];
//     finger->sensors.tf_SensC.p[2] = tf_SensC_param[5];
// }

/**
 * @brief  Compute the hand Jacobian of the finger
 * @param  finger Pointer to the Finger structure
 * @retval  None
 */
void getFingerHandJacobian(Finger *finger)
{
    float R_FC_trsp[3][3] = {0}; // transpose of R_FC
    float p_FC_skew[3][3] = {0}; // skew matrix of vector p_BC
    float R_FS[3][3] = {0};      // R_FS, i.e., transpose of R_SF

    // Get transpose of R_FC_trsp, p_FC_skew and R_SF_trsp
    matTrsp3x3(finger->tf_FC.R, R_FC_trsp);
    vec2matSkew(finger->tf_FC.p, p_FC_skew);
    matTrsp3x3(finger->tf_SF.R, R_FS);

    float temp_result[6][MAX_JOINT_NUM] = {0};
    float temp1[3][3] = {0};
    float temp2[3][3] = {0};

    // temp1 = R_FC_trsp * R_FS
    // temp2 = R_FC_trsp * p_FC_skew * R_FS
    matMul3x3(R_FC_trsp, R_FS, temp1);
    matMul3x3(p_FC_skew, R_FS, temp2);
    matMul3x3(R_FC_trsp, temp2, temp2);

    // temp_result is the hand jacobian before multiplied by the wrench basis
    // Let i be the i-th joint, and the jacob_fingertip actually is jacob_fingertip = [JP; = [[JP1, JP2, JP3, ...,  JPi, ... ];
    //                                                                                 JO]     JO1, JO2, JO3, ...,  JOi, ... ]],
    // and
    // temp_result = [[M1, M2, M3, ..., Mi, ...];
    //                    [N1, N2, N3, ..., Mi, ...]],
    // where Mi = temp1 * JPi - temp2 * JOi
    //       Ni = temp1 * JOi
    for (int i = 0; i < finger->joint_num; i++)
    {
        temp_result[0][i] = temp1[0][0] * finger->jacob_fingertip.JP[0][i] + temp1[0][1] * finger->jacob_fingertip.JP[1][i] + temp1[0][2] * finger->jacob_fingertip.JP[2][i];
        temp_result[1][i] = temp1[1][0] * finger->jacob_fingertip.JP[0][i] + temp1[1][1] * finger->jacob_fingertip.JP[1][i] + temp1[1][2] * finger->jacob_fingertip.JP[2][i];
        temp_result[2][i] = temp1[2][0] * finger->jacob_fingertip.JP[0][i] + temp1[2][1] * finger->jacob_fingertip.JP[1][i] + temp1[2][2] * finger->jacob_fingertip.JP[2][i];

        temp_result[0][i] -= temp2[0][0] * finger->jacob_fingertip.JO[0][i] + temp2[0][1] * finger->jacob_fingertip.JO[1][i] + temp2[0][2] * finger->jacob_fingertip.JO[2][i];
        temp_result[1][i] -= temp2[1][0] * finger->jacob_fingertip.JO[0][i] + temp2[1][1] * finger->jacob_fingertip.JO[1][i] + temp2[1][2] * finger->jacob_fingertip.JO[2][i];
        temp_result[2][i] -= temp2[2][0] * finger->jacob_fingertip.JO[0][i] + temp2[2][1] * finger->jacob_fingertip.JO[1][i] + temp2[2][2] * finger->jacob_fingertip.JO[2][i];

        temp_result[3][i] = temp1[0][0] * finger->jacob_fingertip.JO[0][i] + temp1[0][1] * finger->jacob_fingertip.JO[1][i] + temp1[0][2] * finger->jacob_fingertip.JO[2][i];
        temp_result[4][i] = temp1[1][0] * finger->jacob_fingertip.JO[0][i] + temp1[1][1] * finger->jacob_fingertip.JO[1][i] + temp1[1][2] * finger->jacob_fingertip.JO[2][i];
        temp_result[5][i] = temp1[2][0] * finger->jacob_fingertip.JO[0][i] + temp1[2][1] * finger->jacob_fingertip.JO[1][i] + temp1[2][2] * finger->jacob_fingertip.JO[2][i];
    }

    // if (finger->contact_type == CONTACT_TYPE_SOFT_FINGER)
    // {
    //     // Select the 1, 2, 3, 6 row, coresponding to Fx, Fy, Fz, Tauz
    //     // hand_jacobian is a 4*joint_num matrix in this case
    //     for (int j = 0; j < finger->joint_num; j++)
    //     {
    //         for (int i = 0; i < 3; i++)
    //         {
    //         }
    //         finger->hand_jacobian[3][j] = temp_result[5][j]; // rotation part, dimensionless quantity
    //     }
    // }

    // Assign the specified row of temp_result to hand_jacobian based on wrench basis.
    int nz_ctr = 0; // non-zero index counter
    for (int i = 0; i < 6; i++)
    {
        if (finger->sensors.wrench_basis[i] != 0)
        {
            for (int j = 0; j < finger->joint_num; j++)
            {
                finger->hand_jacobian[nz_ctr][j] = temp_result[i][j]; // linear part, in mm
            }
            nz_ctr++;
        }
    }
}

/**
 * @brief  Compute the gravity compensation term
 * @param  finger Pointer to the Finger structure
 * @retval  None
 */
void getFingerGravComp(Finger *finger)
{
    for (int i = 0; i < finger->joint_num; i++)
    {
        float temp_sum = 0.0f;
        for (int j = 0; j < finger->joint_num; j++)
        {
            float temp_vector[3];

            temp_vector[0] = finger->jacob_link[j].JP[0][i];
            temp_vector[1] = finger->jacob_link[j].JP[1][i];
            temp_vector[2] = finger->jacob_link[j].JP[2][i];

            temp_sum += finger->mass_link[j] * innerProduct(finger->g_base, temp_vector, 3);
        }
        finger->tau_grav[i] = -temp_sum; // in N-mm
    }
}

/**
 * @brief  Compute the link states of the finger, including: tf_joint, P_frame, z_axis, P_link, jacob_link, tf_SF, jacob_fingertip.
 * @param  finger Pointer to the Finger structure
 * @retval  None
 */
void getFingerLinkState(Finger *finger)
{

    // Compute homogeneous transformation matrices of each joint
    for (int i = 0; i < finger->joint_num; i++)
    {
        getTransformMatrix(finger->DH_alpha[i], finger->DH_d[i], finger->DH_a[i], finger->cur_q[i], &finger->tf_joint[i]); // alpha, d, a, theta
    }

    // (1). Compute P_frame and z_axis of each frame and P_link of each link and get tf_SF
    /* ======================================================================================================= */

    // An temporary identity transformation matrix
    Transformation temp_tf = {.R = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
                              .p = {0, 0, 0}};
    //   .param = {0, 0, 0, 0, 0, 0}};

    // z_axis and P_frame are constant for frame 0
    finger->z_axis[0][0] = 0.0f;
    finger->z_axis[0][1] = 0.0f;
    finger->z_axis[0][2] = 1.0f;

    finger->P_frame[0][0] = 0.0f;
    finger->P_frame[0][1] = 0.0f;
    finger->P_frame[0][2] = 0.0f;
    for (int i = 0; i < finger->joint_num; i++)
    {
        // get transformaiton from link frame i+1 to frame i of the finger
        multiplyTransform(&temp_tf, &finger->tf_joint[i], &temp_tf);

        // compute P_link of link i+1
        applyTransformation(&temp_tf, &finger->P_link_bdf[i], finger->P_link[i]);

        // compute P_frame of frame i+1
        finger->P_frame[i + 1][0] = temp_tf.p[0];
        finger->P_frame[i + 1][1] = temp_tf.p[1];
        finger->P_frame[i + 1][2] = temp_tf.p[2];

        // compute z_axis of frame i+1
        finger->z_axis[i + 1][0] = temp_tf.R[0][2];
        finger->z_axis[i + 1][1] = temp_tf.R[1][2];
        finger->z_axis[i + 1][2] = temp_tf.R[2][2];

        // Get tf_SF at the last iteration,
        // at which temp_tf is the transformation from Fingertip frame to base frame of the finger
        if (i == (finger->joint_num - 1))
        {
            // finger->tf_SF = temp_tf;
            memcpy(&finger->tf_SF.R, &temp_tf.R, sizeof(temp_tf.R));
            memcpy(&finger->tf_SF.p, &temp_tf.p, sizeof(temp_tf.p));

            // get tf_PSens
            Transformation tf_PF = {.R = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
                                    .p = {0, 0, 0}};
            multiplyTransform(&finger->tf_PS, &finger->tf_SF, &tf_PF);
            multiplyTransform(&tf_PF, &finger->tf_FSens, &finger->tf_PSens);
        }
    }

    // (2). Compute linear and rotation part of jacob_link
    /* ======================================================================================================= */
    for (int i = 0; i < finger->joint_num; i++) // i for each link
    {
        for (int j = 0; j < finger->joint_num; j++) // j for each frame
        {
            float temp_vector[3];

            if (i >= j)
            {
                // (P_link j+1) - (P_frame of frame i)
                temp_vector[0] = finger->P_link[i][0] - finger->P_frame[j][0];
                temp_vector[1] = finger->P_link[i][1] - finger->P_frame[j][1];
                temp_vector[2] = finger->P_link[i][2] - finger->P_frame[j][2];

                float cp_result[3];
                crossProduct(finger->z_axis[j], temp_vector, cp_result);

                finger->jacob_link[i].JP[0][j] = cp_result[0];
                finger->jacob_link[i].JP[1][j] = cp_result[1];
                finger->jacob_link[i].JP[2][j] = cp_result[2];

                finger->jacob_link[i].JO[0][j] = finger->z_axis[j][0];
                finger->jacob_link[i].JO[1][j] = finger->z_axis[j][1];
                finger->jacob_link[i].JO[2][j] = finger->z_axis[j][2];
            }
            else
            {
                finger->jacob_link[i].JP[0][j] = 0.0f;
                finger->jacob_link[i].JP[1][j] = 0.0f;
                finger->jacob_link[i].JP[2][j] = 0.0f;
                finger->jacob_link[i].JO[0][j] = 0.0f;
                finger->jacob_link[i].JO[1][j] = 0.0f;
                finger->jacob_link[i].JO[2][j] = 0.0f;
            }
        }
    }

    // (3). Compute jacob_fingertip
    /* ======================================================================================================= */
    for (int i = 0; i < finger->joint_num; i++)
    {
        float temp_vec[3];
        temp_vec[0] = finger->P_frame[3][0] - finger->P_frame[i][0];
        temp_vec[1] = finger->P_frame[3][1] - finger->P_frame[i][1];
        temp_vec[2] = finger->P_frame[3][2] - finger->P_frame[i][2];

        float cp_result[3];
        crossProduct(finger->z_axis[i], temp_vec, cp_result);
        finger->jacob_fingertip.JP[0][i] = cp_result[0];
        finger->jacob_fingertip.JP[1][i] = cp_result[1];
        finger->jacob_fingertip.JP[2][i] = cp_result[2];
        finger->jacob_fingertip.JO[0][i] = finger->z_axis[i][0];
        finger->jacob_fingertip.JO[1][i] = finger->z_axis[i][1];
        finger->jacob_fingertip.JO[2][i] = finger->z_axis[i][2];
    }

    // (4). Compute inertia matrix in dynamic
    /* ======================================================================================================= */
    float sum[MAX_JOINT_NUM][MAX_JOINT_NUM] = {};
    for (int i = 0; i < finger->joint_num; i++)
    {
        // get tf_base_frame
        Transformation tf_base_frame = {};
        memcpy(&tf_base_frame, &finger->tf_joint[i], sizeof(Transformation)); // get tf_i-1_i first
        for (int j = i; j > 0; j--)
        {
            multiplyTransform(&finger->tf_joint[j - 1], &tf_base_frame, &tf_base_frame);
        }

        float rot_base_frame[3][3] = {};
        float rot_frame_base[3][3] = {};
        float link_inertia_in_frame[3][3] = {};
        float link_inertia_in_base[3][3] = {};
        memcpy(&rot_base_frame, tf_base_frame.R, sizeof(tf_base_frame.R));
        matTrsp3x3(&rot_base_frame, &rot_frame_base);
        getInertia(finger->inertia_param[i], &link_inertia_in_frame);
        matMul3x3(link_inertia_in_frame, rot_frame_base, link_inertia_in_base);
        matMul3x3(rot_base_frame, link_inertia_in_base, link_inertia_in_base);

        float temp1[MAX_JOINT_NUM][MAX_JOINT_NUM] = {};
        float temp2[3][MAX_JOINT_NUM] = {};
        float temp3[MAX_JOINT_NUM][MAX_JOINT_NUM] = {};
        for (int j = 0; j < finger->joint_num; j++)
        {
            for (int k = 0; k < finger->joint_num; k++)
            {
                temp1[j][k] = finger->mass_link[i] * (finger->jacob_link[i].JP[0][j] * finger->jacob_link[i].JP[0][k] +
                                                      finger->jacob_link[i].JP[1][j] * finger->jacob_link[i].JP[1][k] +
                                                      finger->jacob_link[i].JP[2][j] * finger->jacob_link[i].JP[2][k]);
            }
        }

        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < finger->joint_num; k++)
            {
                temp2[j][k] = (link_inertia_in_base[j][0] * finger->jacob_link[i].JO[0][k] +
                               link_inertia_in_base[j][1] * finger->jacob_link[i].JO[1][k] +
                               link_inertia_in_base[j][2] * finger->jacob_link[i].JO[2][k]);
            }
        }

        for (int j = 0; j < finger->joint_num; j++)
        {
            for (int k = 0; k < finger->joint_num; k++)
            {
                temp3[j][k] = (finger->jacob_link[i].JO[0][j] * temp2[0][k] +
                               finger->jacob_link[i].JO[1][j] * temp2[1][k] +
                               finger->jacob_link[i].JO[2][j] * temp2[2][k]);
            }
        }

        for (int j = 0; j < finger->joint_num; j++)
        {
            for (int k = 0; k < finger->joint_num; k++)
            {
                sum[j][k] += temp1[j][k] + temp3[j][k];
            }
        }
    }

    memcpy(finger->dynamic_inertia_mat, sum, sizeof(sum));
}

/**
 * @brief  Compute torque of inverse dynamic
 * @param  finger Pointer to Finger structure
 * @param  q_dot Pointer of angular velocity of joint
 * @param  q_dot2 Pointer of angular acceleration of joint
 * @param  torque Pointer of torque result
 * @retval  None
 * @note This function computes < torque = M(q)*q_dot2 + C(q,q_dot)*q_dot
 */
// This inverse dynamics function
void getInverseDynamics(const Finger *finger, const float *q_cmd_dot2, float *torque)
{
    // i for frame 0 ~ N
    float angular_vel[MAX_JOINT_NUM + 1][3]; // Angular velocity of link-i, also the angular velocity of frame i, expressed in frame i, in rad/s
    float angular_acc[MAX_JOINT_NUM + 1][3]; // Angular acceleration of link-i, also the angular acceleration of frame i, expressed in frame i, in rad/(s^2)
    float acc_frame[MAX_JOINT_NUM + 1][3];   // Linear acceration of the origin of frame i, expressed in frame i, in mm/(s^2)

    // i for link 1 ~ N
    float R_joint_trsp[MAX_JOINT_NUM][3][3];
    float acc_COM[MAX_JOINT_NUM][3];    // Linear acceration of the center of mass, expressed in frame i, in mm/(s^2)
    float force_COM[MAX_JOINT_NUM][3];  // force applied at the center of mass, expressed in frame i, in mN
    float torque_COM[MAX_JOINT_NUM][3]; // torque applied at the center of mass, expressed in frame i, in mN-mm

    // i for joint 1 ~ N
    float force_joint[MAX_JOINT_NUM][3];  // force applied at the joint, expressed in frame i, in mN
    float torque_joint[MAX_JOINT_NUM][3]; // torque applied at the joint, expressed in frame i, in mN-mm

    // Step I. Forward kenematic part, i means the i-th link and frame i
    /* ======================================================================================================= */
    // Frame 0 has zero angular velocity, angular acceleration and linear acceleration
    angular_vel[0][0] = 0.0f;
    angular_vel[0][1] = 0.0f;
    angular_vel[0][2] = 0.0f;
    angular_acc[0][0] = 0.0f;
    angular_acc[0][1] = 0.0f;
    angular_acc[0][2] = 0.0f;
    acc_frame[0][0] = 0.0f;
    acc_frame[0][1] = 0.0f;
    acc_frame[0][2] = 0.0f;
    for (int i = 1; i <= finger->joint_num; i++)
    {
        int joint_idx = i - 1;
        int frame_idx = i;

        // Get rotation matrix frome frame i-1 to frame i, R_i_i-1 = trsp(R_i-1_i)
        matTrsp3x3(finger->tf_joint[joint_idx].R, R_joint_trsp[joint_idx]);

        float joint_ang_vel[3]; // anglar velocity of the i-th joint, expressed in frame i-1
        float joint_ang_acc[3]; // anglar acceleration of the i-th joint, expressed in frame i-1
        joint_ang_vel[0] = 0.0f;
        joint_ang_vel[1] = 0.0f;
        joint_ang_vel[2] = finger->cur_q_dot[joint_idx];
        joint_ang_acc[0] = 0.0f;
        joint_ang_acc[1] = 0.0f;
        joint_ang_acc[2] = q_cmd_dot2[joint_idx];

        // (1). Compute the angular velocity of the i-th frame, namely the i-th link
        /* --------------------------------------------------------------------------- */
        float temp_a1[3];
        for (int j = 0; j < 3; j++)
        {
            temp_a1[j] = angular_vel[frame_idx - 1][j] + joint_ang_vel[j];
        }
        applyRotation(R_joint_trsp[joint_idx], temp_a1, angular_vel[frame_idx]);

        // (2). Compute the angular acceleration of the i-th frame, namely the i-th link
        /* --------------------------------------------------------------------------- */
        float temp_b1[3];
        float temp_b2[3];
        crossProduct(angular_vel[frame_idx - 1], joint_ang_vel, temp_b1);
        for (int j = 0; j < 3; j++)
        {
            temp_b2[j] = angular_acc[frame_idx - 1][j] + joint_ang_acc[j] + temp_b1[j];
        }
        applyRotation(R_joint_trsp[joint_idx], temp_b2, angular_acc[frame_idx]);

        // (3). Compute the linear acceleration of the i-th frame, namely the i-th link
        /* --------------------------------------------------------------------------- */
        float temp_c1[3];
        float temp_c2[3];
        float temp_c3[3];
        float temp_c4[3];
        float temp_c5[3];
        crossProduct(angular_acc[frame_idx - 1], finger->tf_joint[joint_idx].p, temp_c1);
        crossProduct(angular_vel[frame_idx - 1], finger->tf_joint[joint_idx].p, temp_c2);
        crossProduct(angular_vel[frame_idx - 1], temp_c2, temp_c2);
        crossProduct(joint_ang_acc, finger->tf_joint[joint_idx].p, temp_c3);
        crossProduct(joint_ang_vel, finger->tf_joint[joint_idx].p, temp_c4);
        crossProduct(joint_ang_vel, temp_c4, temp_c4);
        for (int j = 0; j < 3; j++)
        {
            temp_c5[j] = acc_frame[frame_idx - 1][j] + temp_c1[j] + temp_c2[j] + temp_c3[j] + temp_c4[j];
        }
        applyRotation(R_joint_trsp[joint_idx], temp_c5, acc_frame[frame_idx]);

        // (4). Compute the linear acceleration of the center of mass of link i
        /* --------------------------------------------------------------------------- */
        float temp_d1[3];
        float temp_d2[3];
        crossProduct(angular_acc[frame_idx], finger->P_link_bdf[joint_idx], temp_d1);
        crossProduct(angular_vel[frame_idx], finger->P_link_bdf[joint_idx], temp_d2);
        crossProduct(angular_vel[frame_idx], temp_d2, temp_d2);
        for (int j = 0; j < 3; j++)
        {
            acc_COM[joint_idx][j] = acc_frame[frame_idx][j] + temp_d1[j] + temp_d2[j];
        }
    }

    // Step II. Backward kenetic part, i means the i-th link and frame i
    /* ======================================================================================================= */
    force_joint[finger->joint_num][0] = 0.0f;
    force_joint[finger->joint_num][1] = 0.0f;
    force_joint[finger->joint_num][2] = 0.0f;
    torque_joint[finger->joint_num][0] = 0.0f;
    torque_joint[finger->joint_num][1] = 0.0f;
    torque_joint[finger->joint_num][2] = 0.0f;
    for (int i = finger->joint_num; i > 0; i--)
    {
        int joint_idx = i - 1;
        int frame_idx = i;

        float inertia_matrix[3][3];
        getInertia(finger->inertia_param[joint_idx], inertia_matrix);

        // (1). Compute the force applied at the COM of the i-th link
        /* --------------------------------------------------------------------------- */
        for (int j = 0; j < 3; j++)
        {
            force_COM[joint_idx][j] = finger->mass_link[joint_idx] * acc_COM[joint_idx][j];
        }

        // (2). Compute the torque applied at the COM of the i-th link
        /* --------------------------------------------------------------------------- */
        float temp_f1[3];
        float temp_f2[3];
        for (int j = 0; j < 3; j++)
        {
            temp_f1[j] = inertia_matrix[j][0] * angular_acc[frame_idx][0] +
                         inertia_matrix[j][1] * angular_acc[frame_idx][1] +
                         inertia_matrix[j][2] * angular_acc[frame_idx][2];
            temp_f2[j] = inertia_matrix[j][0] * angular_vel[frame_idx][0] +
                         inertia_matrix[j][1] * angular_vel[frame_idx][1] +
                         inertia_matrix[j][2] * angular_vel[frame_idx][2];
        }
        crossProduct(angular_vel[frame_idx], temp_f2, temp_f2);
        for (int j = 0; j < 3; j++)
        {
            torque_COM[joint_idx][j] = temp_f1[j] + temp_f2[j];
        }
        // (3). Compute the force applied at the i-th joint
        /* --------------------------------------------------------------------------- */
        float temp_g1[3];
        if (joint_idx < finger->joint_num - 1)
        {
            applyRotation(finger->tf_joint[joint_idx + 1].R, force_joint[joint_idx + 1], temp_g1);
        }
        else
        {
            temp_g1[0] = 0.0f;
            temp_g1[1] = 0.0f;
            temp_g1[2] = 0.0f;
        }
        for (int j = 0; j < 3; j++)
        {
            force_joint[joint_idx][j] = temp_g1[j] + force_COM[joint_idx][j];
        }

        // (4). Compute the torque applied at the i-th joint
        /* --------------------------------------------------------------------------- */
        float temp_h1[3];
        float temp_h2[3];
        float temp_h3[3];
        float temp_h4[3];
        float temp_h5[3];
        float temp_h6[3];

        // temp12 = R_i_i+1 * torque_joint[i+1]
        if (joint_idx < finger->joint_num - 1)
        {
            applyRotation(finger->tf_joint[joint_idx + 1].R, torque_joint[joint_idx + 1], temp_h1);
        }
        else
        {
            temp_h1[0] = 0.0f;
            temp_h1[1] = 0.0f;
            temp_h1[2] = 0.0f;
        }

        // temp13 = R_i_i-1 * p_i-1_i
        applyRotation(R_joint_trsp[joint_idx], finger->tf_joint[joint_idx].p, temp_h2);

        // temp13_2 = temp13_1 + P_link_bdf[i]
        for (int j = 0; j < 3; j++)
            temp_h3[j] = temp_h2[j] + finger->P_link_bdf[joint_idx][j];

        // temp13_3 = temp13_2 cross force_COM[i]
        crossProduct(temp_h3, force_COM[joint_idx], temp_h3);

        // temp14_1 = R_i_i+1 * force_joint[i+1]
        if (joint_idx < finger->joint_num - 1)
        {
            applyRotation(finger->tf_joint[joint_idx + 1].R, force_joint[joint_idx + 1], temp_h4);
        }
        else
        {
            temp_h4[0] = 0.0f;
            temp_h4[1] = 0.0f;
            temp_h4[2] = 0.0f;
        }

        // temp14_2 = temp13_1 cross temp14_1
        crossProduct(temp_h2, temp_h4, temp_h5);

        for (int j = 0; j < 3; j++)
        {
            torque_joint[joint_idx][j] = torque_COM[joint_idx][j] + temp_h1[j] + temp_h3[j] + temp_h5[j];
        }
    }

    // Step III. Extract the joint torque in joint direction
    /* ======================================================================================================= */
    for (int i = 1; i <= finger->joint_num; i++)
    {
        int joint_idx = i - 1;
        int frame_idx = i;

        float temp[3];
        applyRotation(finger->tf_joint[joint_idx].R, torque_joint[joint_idx], temp); // temp15 = R_i-1_i * torque_joint[i]
        torque[joint_idx] = temp[2];                                                 // the z-axis is the joint direction
    }
}

void getInertia(const float inertia_param[6], float inertia_matrix[3][3])
{
    inertia_matrix[0][0] = inertia_param[0];
    inertia_matrix[0][1] = inertia_param[3];
    inertia_matrix[0][2] = inertia_param[5];
    inertia_matrix[1][0] = inertia_param[3];
    inertia_matrix[1][1] = inertia_param[1];
    inertia_matrix[1][2] = inertia_param[4];
    inertia_matrix[2][0] = inertia_param[5];
    inertia_matrix[2][1] = inertia_param[4];
    inertia_matrix[2][2] = inertia_param[2];
}

bool jointLimitCheck(const Finger *finger)
{
    for (int i = 0; i < finger->joint_num; i++)
    {
        if (finger->cur_q[i] > finger->joint_max_ang[i] || finger->cur_q[i] < finger->joint_min_ang[i])
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief  Apply a homogeneous transformation matrix to a point.
 * @param  tf_AB Transformation from frame B to frame A
 * @param  p_B Point in frame B (3x1 vector)
 * @param  p_A Point in frame A (3x1 vector)
 * @retval  None
 * @note This function computes < p_A = tf_AB * p_B >
 */
void applyTransformation(const Transformation *tf_AB, const float p_B[3], float p_A[3])
{
    float temp[3];
    applyRotation(tf_AB->R, p_B, temp);
    p_A[0] = temp[0] + tf_AB->p[0];
    p_A[1] = temp[1] + tf_AB->p[1];
    p_A[2] = temp[2] + tf_AB->p[2];
}

/**
 * @brief  Apply a rotation matrix to a vector.
 * @param  rot_AB Rotation matrix from frame B to frame A (3x3 matrix)
 * @param  v_B Vector in frame B (3x1 vector)
 * @param  v_A Vector in frame A (3x1 vector)
 * @retval  None
 * @note This function computes < v_A = rot_AB * v_B >
 */
void applyRotation(const float rot_AB[3][3], const float v_B[3], float v_A[3])
{
    float temp[3];
    for (int i = 0; i < 3; i++)
    {
        temp[i] = rot_AB[i][0] * v_B[0] + rot_AB[i][1] * v_B[1] + rot_AB[i][2] * v_B[2];
    }
    memcpy(v_A, temp, sizeof(temp));
}

/**
 * @brief  Computes the inverse of homogeneous transformaiton matrix.
 * @param  tf The original transformation (from frame B to frame A)
 * @param  result Resultant transformation (from frame A to frame B)
 * @retval  None
 * @note This function computes < result = inverse( tf ) >
 */
void inverseTransform(const Transformation *tf, Transformation *result)
{
    // inv(tf) = [ [trsp(R), -trsp(R) * p];
    //             [   0   ,      1      ] ]
    float R_trsp[3][3];
    float p_temp[3];
    matTrsp3x3(tf->R, R_trsp);
    applyRotation(R_trsp, tf->p, p_temp); // p_temp = R_trsp * p

    result->p[0] = -p_temp[0];
    result->p[1] = -p_temp[1];
    result->p[2] = -p_temp[2];
    memcpy(result->R, R_trsp, sizeof(R_trsp));
}

/**
 * @brief  Computes the combined transformation from two given homogeneous transformation matrices.
 * @param  tf1 First transformation (from frame B to frame A)
 * @param  tf2 Second transformation (from frame C to frame B)
 * @param  result Resultant transformation (from frame C to frame A)
 * @retval  None
 * @note This function computes < result = tf1 * tf2 >
 */
void multiplyTransform(const Transformation *tf1, const Transformation *tf2, Transformation *result)
{
    float temp_p[3];
    float temp_R[3][3];
    matMul3x3(tf1->R, tf2->R, temp_R);
    applyRotation(tf1->R, tf2->p, temp_p);

    temp_p[0] = temp_p[0] + tf1->p[0];
    temp_p[1] = temp_p[1] + tf1->p[1];
    temp_p[2] = temp_p[2] + tf1->p[2];

    memcpy(result->p, temp_p, sizeof(temp_p));
    memcpy(result->R, temp_R, sizeof(temp_R));
}

/**
 * @brief  Compute matrix multiplication of two 3x3 matrices
 * @param  mat1 First 3x3 matrix
 * @param  mat2 Second 3x3 matrix
 * @param  result Resultant 3x3 matrix
 * @retval  None
 * @note This function computes < result = mat1 * mat2 >
 */
void matMul3x3(const float mat1[3][3], const float mat2[3][3], float result[3][3])
{
    float temp[3][3] = {0};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                temp[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
    memcpy(result, temp, sizeof(temp));
}

/**
 * @brief  Apply the skew operation to a 3D vector
 * @param  vec 3D vector
 * @param  skew_mat Skew matrix result
 * @retval  None
 */
void vec2matSkew(const float vec[3], float skew_mat[3][3])
{
    skew_mat[0][0] = 0.0f;
    skew_mat[0][1] = -vec[2];
    skew_mat[0][2] = vec[1];
    skew_mat[1][0] = vec[2];
    skew_mat[1][1] = 0.0f;
    skew_mat[1][2] = -vec[0];
    skew_mat[2][0] = -vec[1];
    skew_mat[2][1] = vec[0];
    skew_mat[2][2] = 0.0f;
}

// void transform2Param(Transformation *tf)
// {
//     // Clamp r31 to avoid numerical issues
//     float r31 = tf->R[2][0];
//     if (r31 > 1.0)
//         r31 = 1.0;
//     if (r31 < -1.0)
//         r31 = -1.0;

//     // Pitch (θ)
//     float pitch = asin(-r31);
//     float cos_pitch = cos(pitch);
//     float roll;
//     float yaw;

//     // ---------- Non-singular case ----------
//     if (fabs(cos_pitch) > 1e-6)
//     {
//         roll = atan2(tf->R[2][1], tf->R[2][2]);
//         yaw = atan2(tf->R[1][0], tf->R[0][0]);
//     }
//     // ---------- Singular case (gimbal lock) ----------
//     else
//     {
//         // Fix roll = 0
//         roll = 0.0f;

//         if (r31 < 0) // pitch = +90 deg
//         {
//             // sin(pitch) = +1 → r31 = -1
//             yaw = atan2(-tf->R[0][1], tf->R[1][1]);
//         }
//         else // pitch = -90 deg
//         {
//             // sin(pitch) = -1 → r31 = +1
//             yaw = atan2(tf->R[0][1], tf->R[1][1]);
//         }
//     }
// }

void transform2Param(const Transformation *tf, float param[6])
{
    // Position
    param[0] = tf->p[0]; // x
    param[1] = tf->p[1]; // y
    param[2] = tf->p[2]; // z

    // ZYX Euler angles extracted from rotation matrix R
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    //
    // R = | R[0][0]  R[0][1]  R[0][2] |   | cYcP   cYsPsR-sYcR   cYsPcR+sYsR |
    //     | R[1][0]  R[1][1]  R[1][2] | = | sYcP   sYsPsR+cYcR   sYsPcR-cYsR |
    //     | R[2][0]  R[2][1]  R[2][2] |   | -sP         cPsR           cPcR   |

    float r00 = tf->R[0][0], r10 = tf->R[1][0], r20 = tf->R[2][0];
    float r11 = tf->R[1][1], r12 = tf->R[1][2];
    float r21 = tf->R[2][1], r22 = tf->R[2][2];

    float cos_pitch = sqrtf(r00 * r00 + r10 * r10);

    float roll, pitch, yaw;

    if (cos_pitch < 1e-6f)
    {
        // Gimbal lock: pitch ≈ ±90°
        // yaw 與 roll 退化，令 yaw = 0，由 R[1][1], R[1][2] 解 roll
        yaw = 0.0f;
        pitch = atan2f(-r20, cos_pitch);
        roll = atan2f(-r12, r11);
    }
    else
    {
        yaw = atan2f(r10, r00);
        pitch = atan2f(-r20, cos_pitch);
        roll = atan2f(r21, r22);
    }

    param[3] = roll;  // rotation about X
    param[4] = pitch; // rotation about Y
    param[5] = yaw;   // rotation about Z
}

/**
 * @brief  Compute transformation matrix from the pos-attitude parameter
 * @param  param the pos-attitude parameter, [x, y, z, roll, pitch, yaw], in ZYX order
 * @param  tf the output Transformation ptr
 * @retval  None
 */
void param2Transform(const float param[6], Transformation *tf)
{
    tf->p[0] = param[0];
    tf->p[1] = param[1];
    tf->p[2] = param[2];

    float roll = param[3];
    float pitch = param[4];
    float yaw = param[5];

    // Compute rotation matrix from roll, pitch, yaw
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    tf->R[0][0] = cy * cp;
    tf->R[0][1] = cy * sp * sr - sy * cr;
    tf->R[0][2] = cy * sp * cr + sy * sr;
    tf->R[1][0] = sy * cp;
    tf->R[1][1] = sy * sp * sr + cy * cr;
    tf->R[1][2] = sy * sp * cr - cy * sr;
    tf->R[2][0] = -sp;
    tf->R[2][1] = cp * sr;
    tf->R[2][2] = cp * cr;
}

/**
 * @brief  Compute the transpose of a 3x3 matrix
 * @param  mat Original 3x3 matrix
 * @param  mat_trsp Transposed 3x3 matrix
 * @retval  None
 */
void matTrsp3x3(const float mat[3][3], float mat_trsp[3][3])
{
    float temp[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            temp[i][j] = mat[j][i];
        }
    }
    memcpy(mat_trsp, temp, sizeof(temp));
}

/**
 * @brief  Computes the cross product of two 3D vectors.
 * @param  vec1 First vector (3x1)
 * @param  vec2 Second vector (3x1)
 * @param  result Resultant vector (3x1)
 * @retval  None
 * @note This function computes < result = (vec1) X (vec2) >
 */
void crossProduct(const float vec1[3], const float vec2[3], float result[3])
{
    float temp[3];
    temp[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
    temp[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
    temp[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];

    memcpy(result, temp, sizeof(temp));
}

/**
 * @brief  Computes the inner product of two vectors.
 * @param  vec1 First vector
 * @param  vec2 Second vector
 * @param  size Size of the vectors
 * @param  result Resultant scalar
 */
float innerProduct(const float *vec1, const float *vec2, int size)
{
    float sum = 0.0f;
    for (int i = 0; i < size; i++)
    {
        sum += vec1[i] * vec2[i];
    }
    return sum;
}

/**
 * @brief  Compute transformation matrix from DH parameters.
 * @param  alpha DH parameter alpha, in rad
 * @param  d     DH parameter d, in mm
 * @param  a     DH parameter a, in mm
 * @param  theta DH parameter theta, in rad
 * @param  tf_matrix Pointer to the Transformation structure to store the result
 * @retval  None
 */
void getTransformMatrix(const float alpha, const float d, const float a, const float theta, Transformation *tf_matrix)
{
    tf_matrix->R[0][0] = cosf(theta);
    tf_matrix->R[0][1] = -sinf(theta) * cosf(alpha);
    tf_matrix->R[0][2] = sinf(theta) * sinf(alpha);
    tf_matrix->p[0] = a * cosf(theta);

    tf_matrix->R[1][0] = sinf(theta);
    tf_matrix->R[1][1] = cosf(theta) * cosf(alpha);
    tf_matrix->R[1][2] = -cosf(theta) * sinf(alpha);
    tf_matrix->p[1] = a * sinf(theta);

    tf_matrix->R[2][0] = 0.0f;
    tf_matrix->R[2][1] = sinf(alpha);
    tf_matrix->R[2][2] = cosf(alpha);
    tf_matrix->p[2] = d;
}