#ifndef MY_CAN_TABLE_H
#define MY_CAN_TABLE_H

// Define CAN IDs for different messages
#define CANID_FINGER_MODE 1
#define CANID_FINGER_MODE_TRIG 2
#define CANID_JOINT_POS_CMD 3
#define CANID_JOINT_POS_CMD_ENABLE 4
#define CANID_FINGER_CONT_INFO 5
#define CANID_FINGER_FORCE_CMD 6
#define CANID_FINGER_TORQUE_CMD 7
#define CANID_FINGER_WRENCH_CMD_TRIG 8
#define CANID_GRAV_IN_PALM 9
#define CANID_SETTING_STALLED_PARAM 10
#define CANID_SETTING_RESP_FINGER_STATE 11
#define CANID_SETTING_RESP_JOINT_STATE 12
#define CANID_SETTING_RESP_TFPSENS 13
#define CANID_CLEAR_WARNING 14

// #define CANID_RESP_ERROR_CHECK 100

#define CANID_FINGER_STATE_RESP 101
#define CANID_JOINT_STATE_RESP 102
#define CANID_TF_PSENS_RESP 103

#define CAN_FORCE_RESOLUTION 0.01f         // in N, the resolution of force command sent from upper level
#define CAN_TORQUE_RESOLUTION 0.01f        // in N-m, the resolution of torque command sent from upper level
#define CAN_TIME_RESOLUTION 0.05f          // in s, the resolution of timestamp sent from upper level
#define CAN_POS_RESOLUTION 0.001534f       // in rad, the resolution of position command sent from upper level
#define CAN_VEL_RESOLUTION 0.024f          // in rad/s, the resolution of velocity command sent from upper level
#define CAN_TAUM_RESOLUTION 4.0f           // in N-mm
#define CAN_TF_TRANS_RESOLUTION 2.0f       // in mm, the resolution of translation part of homogeneous transformation matrix sent from upper level
#define CAN_TF_ROT_RESOLUTION 0.02464f     // in rad (2pi/255), the resolution of rotation part of homogeneous transformation matrix sent from upper level
#define CAN_GRAV_RESOLUTION 0.001f         // in m/s^2, the resolution of gravity vector sent from upper level
#define CAN_UNIT_BYTE_RESOLUTION 0.003922f // 1/255
#define CAN_RESP_FREQ_RESOLUTION 1.0f      // in Hz

// POS_CMD_ENABLE
// 1. 每個關節都有，但只能對一手指的所有關節一次寫入
// 2. 只有在該手指為位置模式時才能有效寫入，否則不理會寫入請求
// 1. 軌跡點都順利pop完後會自動將POS_CMD_ENABLE寫成disable狀態
// 3. 如果是從disable寫入變成enable，則會從queue中抓第一個軌跡點作為endpoint，從當前pos_cmd開始生成軌跡命令並控制
//    但如果queue中的第一個軌跡點無效，或是不存在第一個軌跡點，則POS_CMD_ENABLE維持disable狀態並清空queue
// 4. 如果是從enable寫入變成disable，則會將queue清空，並且將pos_cmd保持在當前位置不變

// CANID_FINGER_MODE_TRIG
// 0. 讀取完後會自動清0
// 1. 當從非位置模式切換成位置模式時就以當下關節角度為命令做控制，並且將POS_CMD_ENABLE改為disable狀態

// CANID_FINGER_WRENCH_CMD_TRIG
// 1. 讀取完後會自動清0
// 2. force_buffer會保留上次的值，直到下一次寫入

#endif // MY_CAN_TABLE_H

// 位置模式和queue_enable
//
// 本來就在位置(已有endpoint)
// enable變disable，清空queue
// dis變en，處理第一軌跡問題
//
// 不在位置
// enable、disable隨便變
//

// 從位置模式離開
// 清空queue，變成disable

// 從非位置變成位置
// 如果enable
// 有軌跡，第一點處理
// 無軌跡或失敗，當前位置作為命令，設為disable，清空queue
// 如果disable
// 當前位置作為命令