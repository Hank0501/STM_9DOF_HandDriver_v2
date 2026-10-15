#ifndef FINGER_CONFIG_H
#define FINGER_CONFIG_H

// Configuration of thumb
// ==================================================================================================================== //
#define THUMB_JOINT_NUM 3

/*
 */
// Link mass
#define THUMB_MASS_LINK1 0.0195f // in kg
#define THUMB_MASS_LINK2 0.1951f // in kg
#define THUMB_MASS_LINK3 0.0853f // in kg 0.0853

#define THUMB_MASS_POS_LINK1_X -25.66f // in mm
#define THUMB_MASS_POS_LINK1_Y 0.0205f // in mm
#define THUMB_MASS_POS_LINK1_Z 0.8554f // in mm
#define THUMB_MASS_POS_LINK2_X -47.45f // in mm
#define THUMB_MASS_POS_LINK2_Y -6.98f  // in mm
#define THUMB_MASS_POS_LINK2_Z 0.0f    // in mm
#define THUMB_MASS_POS_LINK3_X -35.76f // in mm
#define THUMB_MASS_POS_LINK3_Y -2.75f  // in mm
#define THUMB_MASS_POS_LINK3_Z -0.17f  // in mm

// Inertial of each link, [Ixx, Iyy, Izz, Ixy, Iyz, Ixz], in kg*(mm^2)
// Note that here
// inertia = [ [Ixx, Ixy, Ixz];
//             [Iyx, Iyy, Iyz];
//             [Izx, Izy, Izz] ]
#define THUMB_LNK1_IXX 3902.7909f * 0.001f
#define THUMB_LNK1_IYY 7676.8556f * 0.001f
#define THUMB_LNK1_IZZ 5210.8404f * 0.001f
#define THUMB_LNK1_IXY -10.5175f * 0.001f
#define THUMB_LNK1_IYZ 23.4641f * 0.001f
#define THUMB_LNK1_IXZ 517.3623f * 0.001f

#define THUMB_LNK2_IXX 55375.67f * 0.001f
#define THUMB_LNK2_IYY 216394.29f * 0.001f
#define THUMB_LNK2_IZZ 234628.9f * 0.001f
#define THUMB_LNK2_IXY 40292.22f * 0.001f
#define THUMB_LNK2_IYZ 4844.94f * 0.001f
#define THUMB_LNK2_IXZ 10122.65f * 0.001f

#define THUMB_LNK3_IXX 22936.18f * 0.001f
#define THUMB_LNK3_IYY 39156.39f * 0.001f
#define THUMB_LNK3_IZZ 38031.34f * 0.001f
#define THUMB_LNK3_IXY 4880.33f * 0.001f
#define THUMB_LNK3_IYZ -39.39f * 0.001f
#define THUMB_LNK3_IXZ 726.66f * 0.001f

// #define THUMB_LNK1_IXX 1000.0f * 0.001f
// #define THUMB_LNK1_IYY 1000.0f * 0.001f
// #define THUMB_LNK1_IZZ 1000.0f * 0.001f
// #define THUMB_LNK1_IXY 0.0f
// #define THUMB_LNK1_IYZ 0.0f
// #define THUMB_LNK1_IXZ 0.0f

// #define THUMB_LNK2_IXX 1000.0f * 0.001f
// #define THUMB_LNK2_IYY 1000.0f * 0.001f
// #define THUMB_LNK2_IZZ 1000.0f * 0.001f
// #define THUMB_LNK2_IXY 0.0f
// #define THUMB_LNK2_IYZ 0.0f
// #define THUMB_LNK2_IXZ 0.0f

// #define THUMB_LNK3_IXX 1000.0f * 0.001f
// #define THUMB_LNK3_IYY 1000.0f * 0.001f
// #define THUMB_LNK3_IZZ 1000.0f * 0.001f
// #define THUMB_LNK3_IXY 0.0f
// #define THUMB_LNK3_IYZ 0.0f
// #define THUMB_LNK3_IXZ 0.0f

/*
 */
// DH parameter
#define THUMB_DH_D1 1.050705f  // in mm, after calibration, nominal : 0
#define THUMB_DH_D2 -0.931316f // in mm, after calibration, nominal : 0
#define THUMB_DH_D3 0.0f       // in mm

#define THUMB_DH_A1 51.957f    // in mm, after calibration, nominal : 52
#define THUMB_DH_A2 80.831597f // in mm, after calibration, nominal : 80
#define THUMB_DH_A3 80.0f      // in mm

#define THUMB_DH_ALPHA1 -1.04635f   // in rad, after calibration, nominal : -60 deg/ -1.0472 rad
#define THUMB_DH_ALPHA2 0.00736498f // in rad, after calibration, nominal : 0 deg / 0 rad
#define THUMB_DH_ALPHA3 0.0f        // in rad

#define THUMB_INV_B -10096.0f   // a1^2 - a2^2 - a3^2
#define THUMB_INV_C 7.8125e-05f // 1/(2*a2*a3)
// #define THUMB_INV_F 7420.5f       // a2^2 + a3^2

/*
 */
// Parameter of transformation from Base to palm
// #define THUMB_PALM_BASE_ROLL 1.5708f // in rad, 90 deg
// #define THUMB_PALM_BASE_PITCH 0.0f   // in rad
// #define THUMB_PALM_BASE_YAW 0.0f     // in rad
#define THUMB_PALM_BASE_X 34.5f   // in mm
#define THUMB_PALM_BASE_Y -17.5f  // in mm
#define THUMB_PALM_BASE_Z -23.25f // in mm

// Rotation matrix of tf_PS
// R_PS = [ [R11, R12, R13];
//          [R21, R22, R23];
//          [R31, R32, R33] ]
#define THUMB_PALM_BASE_R11 1.0f
#define THUMB_PALM_BASE_R12 0.0f
#define THUMB_PALM_BASE_R13 0.0f

#define THUMB_PALM_BASE_R21 0.0f
#define THUMB_PALM_BASE_R22 0.0f
#define THUMB_PALM_BASE_R23 -1.0f

#define THUMB_PALM_BASE_R31 0.0f
#define THUMB_PALM_BASE_R32 1.0f
#define THUMB_PALM_BASE_R33 0.0f

/*
 */
// Offset of joint angle in deg, used to allign the servo angle and the model one
// ang_model = ang_servo - homing_offset
#define THUMB_HOMING_OFFSET_TMC 90.0f - 1.193693f // in deg, , after calibration, nominal : 90
#define THUMB_HOMING_OFFSET_PIP 90.0f + 1.191875f // in deg, , after calibration, nominal : 90
#define THUMB_HOMING_OFFSET_IP 90.0f              // in deg, , after calibration, nominal : 90

/*
 */
// Translation part of tf_FSens
#define THUMB_TF_FSENS_X -58.0f // in mm
#define THUMB_TF_FSENS_Y 12.0f  // in mm
#define THUMB_TF_FSENS_Z 0.0f   // in mm

// Rotation matrix of tf_FSens
// R_PS = [ [R11, R12, R13];
//          [R21, R22, R23];
//          [R31, R32, R33] ]
#define THUMB_TF_FSENS_R11 0.0f
#define THUMB_TF_FSENS_R12 1.0f
#define THUMB_TF_FSENS_R13 0.0f

#define THUMB_TF_FSENS_R21 0.0f
#define THUMB_TF_FSENS_R22 0.0f
#define THUMB_TF_FSENS_R23 1.0f

#define THUMB_TF_FSENS_R31 1.0f
#define THUMB_TF_FSENS_R32 0.0f
#define THUMB_TF_FSENS_R33 0.0f

/*
 */
// Limitation of joint angle represented in model perspective
#define THUMB_TMC_MIN -0.10472f  // -6 deg
#define THUMB_TMC_MAX 1.74533f   // 100 deg
#define THUMB_PIP_MIN -0.174533f // -10 deg
#define THUMB_PIP_MAX 1.5708f    // 90 deg
#define THUMB_IP_MIN -0.872665f  // -50 deg
#define THUMB_IP_MAX 1.74533f    // 100 deg

// Configuration of index finger
// ==================================================================================================================== //
#define INDEX_JOINT_NUM 3

/*
 */
// Link mass and inertia
#define INDEX_MASS_LINK1 0.09f   // in kg
#define INDEX_MASS_LINK2 0.101f  // in kg
#define INDEX_MASS_LINK3 0.0341f // in kg

#define INDEX_MASS_POS_LINK1_X -2.41f  // in mm
#define INDEX_MASS_POS_LINK1_Y -15.37f // in mm
#define INDEX_MASS_POS_LINK1_Z 2.0789f // in mm
#define INDEX_MASS_POS_LINK2_X -21.34f // in mm
#define INDEX_MASS_POS_LINK2_Y 0.33f   // in mm
#define INDEX_MASS_POS_LINK2_Z 1.69f   // in mm
#define INDEX_MASS_POS_LINK3_X -25.14f // in mm
#define INDEX_MASS_POS_LINK3_Y 5.47f   // in mm
#define INDEX_MASS_POS_LINK3_Z -0.44f  // in mm

// Inertial of each link, [Ixx, Iyy, Izz, Ixy, Iyz, Ixz], in kg*(mm^2)
// Note that here
// inertia = [ [Ixx, Ixy, Ixz];
//             [Iyx, Iyy, Iyz];
//             [Izx, Izy, Izz] ]
#define INDEX_LNK1_IXX 26581.1822f * 0.001f
#define INDEX_LNK1_IYY 17743.3794f * 0.001f
#define INDEX_LNK1_IZZ 27919.5662f * 0.001f
#define INDEX_LNK1_IXY 264.6757f * 0.001f
#define INDEX_LNK1_IYZ -1817.6646f * 0.001f
#define INDEX_LNK1_IXZ 304.3351f * 0.001f

#define INDEX_LNK2_IXX 18624.30f * 0.001f
#define INDEX_LNK2_IYY 46119.26f * 0.001f
#define INDEX_LNK2_IZZ 40205.28f * 0.001f
#define INDEX_LNK2_IXY 331.03f * 0.001f
#define INDEX_LNK2_IYZ 83.3745f * 0.001f
#define INDEX_LNK2_IXZ -645.7055f * 0.001f

#define INDEX_LNK3_IXX 4492.86f * 0.001f
#define INDEX_LNK3_IYY 13311.91f * 0.001f
#define INDEX_LNK3_IZZ 12170.25f * 0.001f
#define INDEX_LNK3_IXY 2184.30f * 0.001f
#define INDEX_LNK3_IYZ 90.27f * 0.001f
#define INDEX_LNK3_IXZ 155.30f * 0.001f

/*
 */
// DH parameter
#define INDEX_DH_D1 0.0f // in mm
#define INDEX_DH_D2 0.0f // in mm
#define INDEX_DH_D3 0.0f // in mm

#define INDEX_DH_A1 36.25f // in mm
#define INDEX_DH_A2 70.0f  // in mm
#define INDEX_DH_A3 60.0f  // in mm

#define INDEX_DH_ALPHA1 1.570796f // in rad
#define INDEX_DH_ALPHA2 0.0f      // in rad
#define INDEX_DH_ALPHA3 0.0f      // in rad

#define INDEX_INV_B -7185.9375f   // a1^2 - a2^2 - a3^2
#define INDEX_INV_C 1.190476e-04f // 1/(2*a2*a3)
// #define INDEX_INV_F 8983.742f             // a2^2 + a3^2

/*
 */
// Parameter of transformation from Base to palm
// #define INDEX_PALM_BASE_ROLL 0.0f   // in rad
// #define INDEX_PALM_BASE_PITCH 0.0f  // in rad
// #define INDEX_PALM_BASE_YAW 1.5708f // in rad, 90 deg
#define INDEX_PALM_BASE_X 56.5f  // in mm
#define INDEX_PALM_BASE_Y 42.25f // in mm
#define INDEX_PALM_BASE_Z -15.0f // in mm

// Rotation matrix of tf_PS
// R_PS = [ [R11, R12, R13];
//          [R21, R22, R23];
//          [R31, R32, R33] ]
#define INDEX_PALM_BASE_R11 0.0f
#define INDEX_PALM_BASE_R12 -1.0f
#define INDEX_PALM_BASE_R13 0.0f

#define INDEX_PALM_BASE_R21 1.0f
#define INDEX_PALM_BASE_R22 0.0f
#define INDEX_PALM_BASE_R23 0.0f

#define INDEX_PALM_BASE_R31 0.0f
#define INDEX_PALM_BASE_R32 0.0f
#define INDEX_PALM_BASE_R33 1.0f

/*
 */
// Offset of joint angle, used to allign the servo angle and the model one
#define INDEX_HOMING_OFFSET_MCP 270.0f // in deg
#define INDEX_HOMING_OFFSET_PIP 90.0f  // in deg
#define INDEX_HOMING_OFFSET_DIP 180.0f // in deg

/*
 */
// Translation part of tf_FSens
#define INDEX_TF_FSENS_X 0.0f // in mm
#define INDEX_TF_FSENS_Y 0.0f // in mm
#define INDEX_TF_FSENS_Z 0.0f // in mm

// Rotation matrix of tf_FSens
// R_PS = [ [R11, R12, R13];
//          [R21, R22, R23];
//          [R31, R32, R33] ]
#define INDEX_TF_FSENS_R11 1.0f
#define INDEX_TF_FSENS_R12 0.0f
#define INDEX_TF_FSENS_R13 0.0f

#define INDEX_TF_FSENS_R21 0.0f
#define INDEX_TF_FSENS_R22 1.0f
#define INDEX_TF_FSENS_R23 0.0f

#define INDEX_TF_FSENS_R31 0.0f
#define INDEX_TF_FSENS_R32 0.0f
#define INDEX_TF_FSENS_R33 1.0f

/*
 */
// Limitation of joint angle represented in model perspective
#define INDEX_MCP_MIN -0.785398f // in rad, -45 deg
#define INDEX_MCP_MAX 0.785398f  // in rad, 45 deg
#define INDEX_PIP_MIN -0.174533f // in rad, -10 deg
#define INDEX_PIP_MAX 1.91986f   // in rad, 110 deg
#define INDEX_DIP_MIN -1.5708f   // in rad, -90 deg
#define INDEX_DIP_MAX 1.74533f   // in rad, 100 deg

// Configuration of middle finger
// ==================================================================================================================== //
#define MIDDLE_JOINT_NUM 3

/*
 */
// Link mass and inertia
#define MIDDLE_MASS_LINK1 0.09f   // in kg
#define MIDDLE_MASS_LINK2 0.101f  // in kg
#define MIDDLE_MASS_LINK3 0.0341f // in kg

#define MIDDLE_MASS_POS_LINK1_X -2.41f  // in mm
#define MIDDLE_MASS_POS_LINK1_Y -15.37f // in mm
#define MIDDLE_MASS_POS_LINK1_Z 2.0789f // in mm
#define MIDDLE_MASS_POS_LINK2_X -21.34f // in mm
#define MIDDLE_MASS_POS_LINK2_Y 0.33f   // in mm
#define MIDDLE_MASS_POS_LINK2_Z 1.69f   // in mm
#define MIDDLE_MASS_POS_LINK3_X -25.14f // in mm
#define MIDDLE_MASS_POS_LINK3_Y 5.47f   // in mm
#define MIDDLE_MASS_POS_LINK3_Z -0.44f  // in mm

// Inertial of each link, [Ixx, Iyy, Izz, Ixy, Iyz, Ixz], in kg*(mm^2)
// Note that here
// inertia = [ [Ixx, Ixy, Ixz];
//             [Iyx, Iyy, Iyz];
//             [Izx, Izy, Izz] ]
#define MIDDLE_LNK1_IXX 26581.1822f * 0.001f
#define MIDDLE_LNK1_IYY 17743.3794f * 0.001f
#define MIDDLE_LNK1_IZZ 27919.5662f * 0.001f
#define MIDDLE_LNK1_IXY 264.6757f * 0.001f
#define MIDDLE_LNK1_IYZ -1817.6646f * 0.001f
#define MIDDLE_LNK1_IXZ 304.3351f * 0.001f

#define MIDDLE_LNK2_IXX 18624.30f * 0.001f
#define MIDDLE_LNK2_IYY 46119.26f * 0.001f
#define MIDDLE_LNK2_IZZ 40205.28f * 0.001f
#define MIDDLE_LNK2_IXY 331.03f * 0.001f
#define MIDDLE_LNK2_IYZ 83.3745f * 0.001f
#define MIDDLE_LNK2_IXZ -645.7055f * 0.001f

#define MIDDLE_LNK3_IXX 4492.86f * 0.001f
#define MIDDLE_LNK3_IYY 13311.91f * 0.001f
#define MIDDLE_LNK3_IZZ 12170.25f * 0.001f
#define MIDDLE_LNK3_IXY 2184.30f * 0.001f
#define MIDDLE_LNK3_IYZ 90.27f * 0.001f
#define MIDDLE_LNK3_IXZ 155.30f * 0.001f

/*
 */
// DH parameter
#define MIDDLE_DH_D1 0.0f // in mm
#define MIDDLE_DH_D2 0.0f // in mm
#define MIDDLE_DH_D3 0.0f // in mm

#define MIDDLE_DH_A1 36.25f // in mm
#define MIDDLE_DH_A2 70.0f  // in mm
#define MIDDLE_DH_A3 60.0f  // in mm

#define MIDDLE_DH_ALPHA1 1.570796f // in rad
#define MIDDLE_DH_ALPHA2 0.0f      // in rad
#define MIDDLE_DH_ALPHA3 0.0f      // in rad

#define MIDDLE_INV_B -7185.9375f   // a1^2 - a2^2 - a3^2
#define MIDDLE_INV_C 1.190476e-04f // 1/(2*a2*a3)
// #define MIDDLE_INV_F 8983.742f             // a2^2 + a3^2

/*
 */
// Parameter of transformation from Base to palm
// #define MIDDLE_PALM_BASE_ROLL 0.0f   // in rad
// #define MIDDLE_PALM_BASE_PITCH 0.0f  // in rad
// #define MIDDLE_PALM_BASE_YAW 1.5708f // in rad, 90 deg
#define MIDDLE_PALM_BASE_X 0.0f   // in mm
#define MIDDLE_PALM_BASE_Y 42.25f // in mm
#define MIDDLE_PALM_BASE_Z -15.0f // in mm

// Rotation matrix of tf_PS
// R_PS = [ [R11, R12, R13];
//          [R21, R22, R23];
//          [R31, R32, R33] ]
#define MIDDLE_PALM_BASE_R11 0.0f
#define MIDDLE_PALM_BASE_R12 -1.0f
#define MIDDLE_PALM_BASE_R13 0.0f

#define MIDDLE_PALM_BASE_R21 1.0f
#define MIDDLE_PALM_BASE_R22 0.0f
#define MIDDLE_PALM_BASE_R23 0.0f

#define MIDDLE_PALM_BASE_R31 0.0f
#define MIDDLE_PALM_BASE_R32 0.0f
#define MIDDLE_PALM_BASE_R33 1.0f

/*
 */
// Offset of joint angle, used to allign the servo angle and the model one
#define MIDDLE_HOMING_OFFSET_MCP 180.0f // in deg
#define MIDDLE_HOMING_OFFSET_PIP 90.0f  // in deg
#define MIDDLE_HOMING_OFFSET_DIP 180.0f // in deg

/*
 */
// Translation part of tf_FSens
#define MIDDLE_TF_FSENS_X 0.0f // in mm
#define MIDDLE_TF_FSENS_Y 0.0f // in mm
#define MIDDLE_TF_FSENS_Z 0.0f // in mm

// Rotation matrix of tf_FSens
// R_PS = [ [R11, R12, R13];
//          [R21, R22, R23];
//          [R31, R32, R33] ]
#define MIDDLE_TF_FSENS_R11 1.0f
#define MIDDLE_TF_FSENS_R12 0.0f
#define MIDDLE_TF_FSENS_R13 0.0f

#define MIDDLE_TF_FSENS_R21 0.0f
#define MIDDLE_TF_FSENS_R22 1.0f
#define MIDDLE_TF_FSENS_R23 0.0f

#define MIDDLE_TF_FSENS_R31 0.0f
#define MIDDLE_TF_FSENS_R32 0.0f
#define MIDDLE_TF_FSENS_R33 1.0f

/*
 */
// Limitation of joint angle represented in model perspective
#define MIDDLE_MCP_MIN -0.785398f // in rad, -45 deg
#define MIDDLE_MCP_MAX 0.785398f  // in rad, 45 deg
#define MIDDLE_PIP_MIN -0.174533f // in rad, -10 deg
#define MIDDLE_PIP_MAX 1.91986f   // in rad, 110 deg
#define MIDDLE_DIP_MIN -1.5708f   // in rad, -90 deg
#define MIDDLE_DIP_MAX 1.74533f   // in rad, 100 deg

#endif
