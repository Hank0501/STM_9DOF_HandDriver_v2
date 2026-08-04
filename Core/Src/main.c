/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "stdbool.h"
#include "my_dynamixel.h"
#include "my_finger.h"
#include "finger_control.h"
#include "finger_config_08hand.h"
#include "my_canbus.h"
#include "my_can_table.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct StateResp
{
  bool resp_switch;
  float freq;
  int counter;
} StateResp;

typedef struct FingerSystem
{
  // float param[6];
  // int temp[6];
  StateResp finger_SR;
  StateResp joint_SR;
  StateResp tf_PSens_SR;

  FingerController controller;
  Finger finger;
} FingerSystem;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_2_DEG 57.2958f
#define DEG_2_RAD 0.01745329f
#define RPM_2_RADS 0.10471976f

#define FINGER_NUM 3

#define THUMB_JOINT_NUM 3
#define INDEX_JOINT_NUM 3
#define MIDDLE_JOINT_NUM 3
#define TOTAL_SERVO_NUM THUMB_JOINT_NUM + INDEX_JOINT_NUM + MIDDLE_JOINT_NUM

#define SERVO_ID_THUMB_TMC 1
#define SERVO_ID_THUMB_PIP 2
#define SERVO_ID_THUMB_IP 3
#define SERVO_ID_INDEX_MCP 4
#define SERVO_ID_INDEX_PIP 5
#define SERVO_ID_INDEX_DIP 6
#define SERVO_ID_MIDDLE_MCP 7
#define SERVO_ID_MIDDLE_PIP 8
#define SERVO_ID_MIDDLE_DIP 9

// Servo current to tau conversion parameters
//  (0.63,0.48), (1.05,0.72) in performance graph of XM430W350
//  I = a*Tau + b, a=4/7
//  I in A, Tau in N-m
#define SERVO_KT_C1 0.571428f   // Torque constant, N-m to A
#define SERVO_KT_C2_POS 4.086f  // Positive current offset, in mA
#define SERVO_KT_C2_NEG -2.035f // Negative current offset, in mA

#define CONTROL_PERIOD 1.7f       // in ms
#define STATE_COUNTER_PERIOD 0.5f // in ms

#define ESO_OMEGA0 20.0f

#define VEL_LPF_ALPHA 0.95f

#define ERR_EXCEPTION_BUTTON_PRESSED 0x01 // 0000 0000 0000 0001
#define ERR_CAN_TX_QUEUE_FULL 0x02        // 0000 0000 0000 0010
#define ERR_CAN_RX_QUEUE_FULL 0x04        // 0000 0000 0000 0100
#define ERR_SERVO_RX_FAIL 0x10            // 0000 0000 0001 0000
#define ERR_SERVO_SHUTDOWN 0x20           // 0000 0000 0010 0000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile FingerSystem thumb_sys = {
    .finger.joint_num = THUMB_JOINT_NUM,
    .finger.homing_offset = {THUMB_HOMING_OFFSET_TMC, THUMB_HOMING_OFFSET_PIP, THUMB_HOMING_OFFSET_IP, 0.0f, 0.0f},
    .finger.joint_min_ang = {THUMB_TMC_MIN, THUMB_PIP_MIN, THUMB_IP_MIN},
    .finger.joint_max_ang = {THUMB_TMC_MAX, THUMB_PIP_MAX, THUMB_IP_MAX},
    .finger.DH_alpha = {THUMB_DH_ALPHA1, THUMB_DH_ALPHA2, THUMB_DH_ALPHA3, 0.0f, 0.0f},
    .finger.DH_d = {THUMB_DH_D1, THUMB_DH_D2, THUMB_DH_D3, 0.0f, 0.0f},
    .finger.DH_a = {THUMB_DH_A1, THUMB_DH_A2, THUMB_DH_A3, 0.0f, 0.0f},
    .finger.mass_link = {THUMB_MASS_LINK1, THUMB_MASS_LINK2, THUMB_MASS_LINK3, 0, 0},
    .finger.P_link_bdf = {{THUMB_MASS_POS_LINK1_X, THUMB_MASS_POS_LINK1_Y, THUMB_MASS_POS_LINK1_Z},
                          {THUMB_MASS_POS_LINK2_X, THUMB_MASS_POS_LINK2_Y, THUMB_MASS_POS_LINK2_Z},
                          {THUMB_MASS_POS_LINK3_X, THUMB_MASS_POS_LINK3_Y, THUMB_MASS_POS_LINK3_Z},
                          {0.0f, 0.0f, 0.0f},
                          {0.0f, 0.0f, 0.0f}},
    .finger.tf_PS.p = {THUMB_PALM_BASE_X, THUMB_PALM_BASE_Y, THUMB_PALM_BASE_Z},
    .finger.tf_PS.R = {{THUMB_PALM_BASE_R11, THUMB_PALM_BASE_R12, THUMB_PALM_BASE_R13},
                       {THUMB_PALM_BASE_R21, THUMB_PALM_BASE_R22, THUMB_PALM_BASE_R23},
                       {THUMB_PALM_BASE_R31, THUMB_PALM_BASE_R32, THUMB_PALM_BASE_R33}},
    // .finger.tf_PS.param = {THUMB_PALM_BASE_X,
    //                        THUMB_PALM_BASE_Y,
    //                        THUMB_PALM_BASE_Z,
    //                        THUMB_PALM_BASE_ROLL,
    //                        THUMB_PALM_BASE_PITCH,
    //                        THUMB_PALM_BASE_YAW},
    .finger.tf_FSens.p = {THUMB_TF_FSENS_X, THUMB_TF_FSENS_Y, THUMB_TF_FSENS_Z},
    .finger.tf_FSens.R = {{THUMB_TF_FSENS_R11, THUMB_TF_FSENS_R12, THUMB_TF_FSENS_R13},
                          {THUMB_TF_FSENS_R21, THUMB_TF_FSENS_R22, THUMB_TF_FSENS_R23},
                          {THUMB_TF_FSENS_R31, THUMB_TF_FSENS_R32, THUMB_TF_FSENS_R33}},
    // .finger.tf_FSens.param = {-47.5f, 13.0792f, 0.0f, 0.0f, -1.5708f, -1.5708f},
    .finger.inertia_param = {{THUMB_LNK1_IXX, THUMB_LNK1_IYY, THUMB_LNK1_IZZ, THUMB_LNK1_IXY, THUMB_LNK1_IYZ, THUMB_LNK1_IXZ},
                             {THUMB_LNK2_IXX, THUMB_LNK2_IYY, THUMB_LNK2_IZZ, THUMB_LNK2_IXY, THUMB_LNK2_IYZ, THUMB_LNK2_IXZ},
                             {THUMB_LNK3_IXX, THUMB_LNK3_IYY, THUMB_LNK3_IZZ, THUMB_LNK3_IXY, THUMB_LNK3_IYZ, THUMB_LNK3_IXZ},
                             {0, 0, 0, 0, 0, 0},
                             {0, 0, 0, 0, 0, 0}},
    .finger.vel_filter_alpha = {VEL_LPF_ALPHA, VEL_LPF_ALPHA, VEL_LPF_ALPHA, 0, 0},
};

volatile FingerSystem index_sys = {
    .finger.joint_num = INDEX_JOINT_NUM,
    .finger.homing_offset = {INDEX_HOMING_OFFSET_MCP, INDEX_HOMING_OFFSET_PIP, INDEX_HOMING_OFFSET_DIP, 0.0f, 0.0f},
    .finger.joint_min_ang = {INDEX_MCP_MIN, INDEX_PIP_MIN, INDEX_DIP_MIN},
    .finger.joint_max_ang = {INDEX_MCP_MAX, INDEX_PIP_MAX, INDEX_DIP_MAX},
    .finger.DH_alpha = {INDEX_DH_ALPHA1, INDEX_DH_ALPHA2, INDEX_DH_ALPHA3, 0.0f, 0.0f},
    .finger.DH_d = {INDEX_DH_D1, INDEX_DH_D2, INDEX_DH_D3, 0.0f, 0.0f},
    .finger.DH_a = {INDEX_DH_A1, INDEX_DH_A2, INDEX_DH_A3, 0.0f, 0.0f},
    .finger.mass_link = {INDEX_MASS_LINK1, INDEX_MASS_LINK2, INDEX_MASS_LINK3, 0, 0},
    .finger.P_link_bdf = {{INDEX_MASS_POS_LINK1_X, INDEX_MASS_POS_LINK1_Y, INDEX_MASS_POS_LINK1_Z},
                          {INDEX_MASS_POS_LINK2_X, INDEX_MASS_POS_LINK2_Y, INDEX_MASS_POS_LINK2_Z},
                          {INDEX_MASS_POS_LINK3_X, INDEX_MASS_POS_LINK3_Y, INDEX_MASS_POS_LINK3_Z},
                          {0.0f, 0.0f, 0.0f},
                          {0.0f, 0.0f, 0.0f}},
    .finger.tf_PS.p = {INDEX_PALM_BASE_X, INDEX_PALM_BASE_Y, INDEX_PALM_BASE_Z},
    .finger.tf_PS.R = {{INDEX_PALM_BASE_R11, INDEX_PALM_BASE_R12, INDEX_PALM_BASE_R13},
                       {INDEX_PALM_BASE_R21, INDEX_PALM_BASE_R22, INDEX_PALM_BASE_R23},
                       {INDEX_PALM_BASE_R31, INDEX_PALM_BASE_R32, INDEX_PALM_BASE_R33}},
    // .finger.tf_PS.param = {INDEX_PALM_BASE_X,
    //                        INDEX_PALM_BASE_Y,
    //                        INDEX_PALM_BASE_Z,
    //                        INDEX_PALM_BASE_ROLL,
    //                        INDEX_PALM_BASE_PITCH,
    //                        INDEX_PALM_BASE_YAW},
    .finger.tf_FSens.p = {INDEX_TF_FSENS_X, INDEX_TF_FSENS_Y, INDEX_TF_FSENS_Z},
    .finger.tf_FSens.R = {{INDEX_TF_FSENS_R11, INDEX_TF_FSENS_R12, INDEX_TF_FSENS_R13},
                          {INDEX_TF_FSENS_R21, INDEX_TF_FSENS_R22, INDEX_TF_FSENS_R23},
                          {INDEX_TF_FSENS_R31, INDEX_TF_FSENS_R32, INDEX_TF_FSENS_R33}},
    // .finger.tf_FSens.param = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .finger.inertia_param = {{INDEX_LNK1_IXX, INDEX_LNK1_IYY, INDEX_LNK1_IZZ, INDEX_LNK1_IXY, INDEX_LNK1_IYZ, INDEX_LNK1_IXZ},
                             {INDEX_LNK2_IXX, INDEX_LNK2_IYY, INDEX_LNK2_IZZ, INDEX_LNK2_IXY, INDEX_LNK2_IYZ, INDEX_LNK2_IXZ},
                             {INDEX_LNK3_IXX, INDEX_LNK3_IYY, INDEX_LNK3_IZZ, INDEX_LNK3_IXY, INDEX_LNK3_IYZ, INDEX_LNK3_IXZ},
                             {0, 0, 0, 0, 0, 0},
                             {0, 0, 0, 0, 0, 0}},
    .finger.vel_filter_alpha = {VEL_LPF_ALPHA, VEL_LPF_ALPHA, VEL_LPF_ALPHA, 0, 0},
};

volatile FingerSystem middle_sys = {
    .finger.joint_num = MIDDLE_JOINT_NUM,
    .finger.homing_offset = {MIDDLE_HOMING_OFFSET_MCP, MIDDLE_HOMING_OFFSET_PIP, MIDDLE_HOMING_OFFSET_DIP, 0.0f, 0.0f},
    .finger.joint_min_ang = {MIDDLE_MCP_MIN, MIDDLE_PIP_MIN, MIDDLE_DIP_MIN},
    .finger.joint_max_ang = {MIDDLE_MCP_MAX, MIDDLE_PIP_MAX, MIDDLE_DIP_MAX},
    .finger.DH_alpha = {MIDDLE_DH_ALPHA1, MIDDLE_DH_ALPHA2, MIDDLE_DH_ALPHA3, 0.0f, 0.0f},
    .finger.DH_d = {MIDDLE_DH_D1, MIDDLE_DH_D2, MIDDLE_DH_D3, 0.0f, 0.0f},
    .finger.DH_a = {MIDDLE_DH_A1, MIDDLE_DH_A2, MIDDLE_DH_A3, 0.0f, 0.0f},
    .finger.mass_link = {MIDDLE_MASS_LINK1, MIDDLE_MASS_LINK2, MIDDLE_MASS_LINK3, 0, 0},
    .finger.P_link_bdf = {{MIDDLE_MASS_POS_LINK1_X, MIDDLE_MASS_POS_LINK1_Y, MIDDLE_MASS_POS_LINK1_Z},
                          {MIDDLE_MASS_POS_LINK2_X, MIDDLE_MASS_POS_LINK2_Y, MIDDLE_MASS_POS_LINK2_Z},
                          {MIDDLE_MASS_POS_LINK3_X, MIDDLE_MASS_POS_LINK3_Y, MIDDLE_MASS_POS_LINK3_Z},
                          {0.0f, 0.0f, 0.0f},
                          {0.0f, 0.0f, 0.0f}},
    .finger.tf_PS.p = {MIDDLE_PALM_BASE_X, MIDDLE_PALM_BASE_Y, MIDDLE_PALM_BASE_Z},
    .finger.tf_PS.R = {{MIDDLE_PALM_BASE_R11, MIDDLE_PALM_BASE_R12, MIDDLE_PALM_BASE_R13},
                       {MIDDLE_PALM_BASE_R21, MIDDLE_PALM_BASE_R22, MIDDLE_PALM_BASE_R23},
                       {MIDDLE_PALM_BASE_R31, MIDDLE_PALM_BASE_R32, MIDDLE_PALM_BASE_R33}},
    // .finger.tf_PS.param = {MIDDLE_PALM_BASE_X,
    //                        MIDDLE_PALM_BASE_Y,
    //                        MIDDLE_PALM_BASE_Z,
    //                        MIDDLE_PALM_BASE_ROLL,
    //                        MIDDLE_PALM_BASE_PITCH,
    //                        MIDDLE_PALM_BASE_YAW},
    .finger.tf_FSens.p = {MIDDLE_TF_FSENS_X, MIDDLE_TF_FSENS_Y, MIDDLE_TF_FSENS_Z},
    .finger.tf_FSens.R = {{MIDDLE_TF_FSENS_R11, MIDDLE_TF_FSENS_R12, MIDDLE_TF_FSENS_R13},
                          {MIDDLE_TF_FSENS_R21, MIDDLE_TF_FSENS_R22, MIDDLE_TF_FSENS_R23},
                          {MIDDLE_TF_FSENS_R31, MIDDLE_TF_FSENS_R32, MIDDLE_TF_FSENS_R33}},
    // .finger.tf_FSens.param = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    .finger.inertia_param = {{MIDDLE_LNK1_IXX, MIDDLE_LNK1_IYY, MIDDLE_LNK1_IZZ, MIDDLE_LNK1_IXY, MIDDLE_LNK1_IYZ, MIDDLE_LNK1_IXZ},
                             {MIDDLE_LNK2_IXX, MIDDLE_LNK2_IYY, MIDDLE_LNK2_IZZ, MIDDLE_LNK2_IXY, MIDDLE_LNK2_IYZ, MIDDLE_LNK2_IXZ},
                             {MIDDLE_LNK3_IXX, MIDDLE_LNK3_IYY, MIDDLE_LNK3_IZZ, MIDDLE_LNK3_IXY, MIDDLE_LNK3_IYZ, MIDDLE_LNK3_IXZ},
                             {0, 0, 0, 0, 0, 0},
                             {0, 0, 0, 0, 0, 0}},
    .finger.vel_filter_alpha = {VEL_LPF_ALPHA, VEL_LPF_ALPHA, VEL_LPF_ALPHA, 0, 0},
};

int thumb_idx = 0;
int index_idx = 1;
int middle_idx = 2;
volatile FingerSystem *finger_ptr_list[FINGER_NUM] = {&thumb_sys, &index_sys, &middle_sys};

// Variables related to servo communication
volatile ServoXM4340 myServoList[TOTAL_SERVO_NUM];
float goal_current_list[TOTAL_SERVO_NUM];
float cur_ang_list[TOTAL_SERVO_NUM];
float cur_vel_list[TOTAL_SERVO_NUM];
float cur_current_list[TOTAL_SERVO_NUM];
float goal_position_list[TOTAL_SERVO_NUM];

// CAN communication
// --------------------------------------------------------- //
volatile CAN_Queue can_rx_queue; // CAN receive FIFO buffer
volatile CAN_Queue can_tx_queue; // CAN transmit FIFO buffer

// Others
// --------------------------------------------------------- //
bool controller_isEnabled = false;
uint16_t hand_error_code;
float g_palm[3];                         // gravity vector in the palm frame, updated from upper level, in  m/s^2
volatile bool exc_btn_isPressed = false; // exception error button flag, set to true when the button is pressed
volatile bool controlFlag = false;       // control loop flag, set to true when the timer interrupt occurs
volatile int time_pass;

uint8_t rxData[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void fingerEndPointFK(Finger *finger, int finger_idx, float end_point[3]);
void computeGoalCurrent(const Finger *finger, FingerController *controller);
bool getServoFeedback_thumb(void);
bool getServoFeedback(void);
void CAN_Parsing_PingReq(CAN_Frame *frame);
void CAN_Parsing_FgrMode(CAN_Frame *frame);
void CAN_Parsing_FgrModeTrig(CAN_Frame *frame);
void CAN_Parsing_JntPosCmd(CAN_Frame *frame);
void CAN_Parsing_JntPosCmdEnable(CAN_Frame *frame);
void CAN_Parsing_FgrContInfoTran(CAN_Frame *frame);
void CAN_Parsing_FgrContInfoRot(CAN_Frame *frame);
void CAN_Parsing_FgrForceCmd(CAN_Frame *frame);
void CAN_Parsing_FgrTorqueCmd(CAN_Frame *frame);
void CAN_Parsing_FgrWrenchCmdTrig(CAN_Frame *frame);
void CAN_Parsing_GravInPalm(CAN_Frame *frame);
void CAN_Parsing_SettingStalled(CAN_Frame *frame);
void CAN_Parsing_SettingFgrStateResp(CAN_Frame *frame);
void CAN_Parsing_SettingJntStateResp(CAN_Frame *frame);
void CAN_Parsing_SettingTfPSensResp(CAN_Frame *frame);
void CAN_Parsing_ClearWarning(CAN_Frame *frame);
void CAN_AddRespMsg_FgrState(void);
void CAN_AddRespMsg_JntState(void);
void CAN_AddRespMsg_TfPSensState(void);
void addCANMsgToQueue(const uint32_t id, const uint8_t dlc, const uint8_t *data);
void CAN_TxheaderSetup(CAN_TxHeaderTypeDef *Txheader,
                       uint32_t StdId, uint32_t ExtId,
                       uint32_t IDE, uint32_t RTR, uint32_t DLC,
                       FunctionalState TransmitGlobalTime);
void CAN1_FilterInit(uint32_t FilterBank, uint32_t FilterMode, uint32_t FilterScale,
                     uint32_t FilterIdHigh, uint32_t FilterIdLow, uint32_t MaskIdHigh, uint32_t MaskIdLow,
                     uint32_t FilterFIFOAssignment, uint32_t FilterActivation);
uint16_t servoException(ServoXM4340 *servo, int max_rx_err_ctr);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */

  // Servo initialization
  // ------------------------------------------------------ //
  DXL_InitServo(&myServoList[0], SERVO_ID_THUMB_TMC, &huart6, GPIOD, GPIO_PIN_15);
  DXL_InitServo(&myServoList[1], SERVO_ID_THUMB_PIP, &huart6, GPIOD, GPIO_PIN_15);
  DXL_InitServo(&myServoList[2], SERVO_ID_THUMB_IP, &huart6, GPIOD, GPIO_PIN_15);
  DXL_InitServo(&myServoList[3], SERVO_ID_INDEX_MCP, &huart6, GPIOD, GPIO_PIN_15);
  DXL_InitServo(&myServoList[4], SERVO_ID_INDEX_PIP, &huart6, GPIOD, GPIO_PIN_15);
  DXL_InitServo(&myServoList[5], SERVO_ID_INDEX_DIP, &huart6, GPIOD, GPIO_PIN_15);
  DXL_InitServo(&myServoList[6], SERVO_ID_MIDDLE_MCP, &huart6, GPIOD, GPIO_PIN_15);
  DXL_InitServo(&myServoList[7], SERVO_ID_MIDDLE_PIP, &huart6, GPIOD, GPIO_PIN_15);
  DXL_InitServo(&myServoList[8], SERVO_ID_MIDDLE_DIP, &huart6, GPIOD, GPIO_PIN_15);
  for (int i = 0; i < TOTAL_SERVO_NUM; i++)
  {
    getServo_BaudRate(&myServoList[i], 10);

    // Set Operating Mode to Current Control Mode
    setServo_OperatingMode(&myServoList[i], Current_CtrlMode, 10);
    // setServo_CurrentLimit(&myServoList[i], 1193.0f * DXL_CUR_RESOLUTION);
    getServo_CurrentLimit(&myServoList[i], 10);

    setServo_ReturnDelayTime(&myServoList[i], 0, 10);
    getServo_ReturnDelayTime(&myServoList[i], 10);
  }
  // setServo_OperatingMode(&myServoList[0], CurrentBased_POS_CtrlMode, 10);
  // setServo_OperatingMode(&myServoList[1], CurrentBased_POS_CtrlMode, 10);
  // setServo_OperatingMode(&myServoList[2], CurrentBased_POS_CtrlMode, 10);

  // Finger initialization
  // ------------------------------------------------------ //
  getServoFeedback(); // Update cur_q, cur_q_dot, cur_current of servo
  // default gravity vector in the palm frame, will be updated from upper level

  for (int i = 0; i < FINGER_NUM; i++)
  {
    for (int j = 0; j < finger_ptr_list[i]->finger.joint_num; j++)
    {
      finger_ptr_list[i]->finger.cur_q_dot[j] = 0.0f;
      finger_ptr_list[i]->finger.cur_q_dot_raw[j] = 0.0f;
      finger_ptr_list[i]->finger.prev_q_dot_raw[j] = 0.0f;
    }
  }
  g_palm[0] = 0.0f;
  g_palm[1] = 0.0f;
  g_palm[2] = 0.0f;

  // Controller initialization
  // ------------------------------------------------------ //
  for (int i = 0; i < FINGER_NUM; i++)
  {
    memset(&finger_ptr_list[i]->controller, 0, sizeof(FingerController));

    finger_ptr_list[i]->controller.control_mode = FINGER_IDLE;
    finger_ptr_list[i]->controller.control_period = CONTROL_PERIOD / 1000.0f;
    finger_ptr_list[i]->controller.K_d[0] = 1100.0f; // 500.0f;
    finger_ptr_list[i]->controller.K_d[1] = 1100.0f; // 1500.0f;
    finger_ptr_list[i]->controller.K_d[2] = 2000.0f; // 28000.0f;
    finger_ptr_list[i]->controller.D_d[0] = 2.0f;    // 10.0f;
    finger_ptr_list[i]->controller.D_d[1] = 2.0f;    // 15.0f;
    finger_ptr_list[i]->controller.D_d[2] = 2.0f;    // 400.0f;
    finger_ptr_list[i]->controller.G_i[0] = 00.0f;   // 0000.0f;
    finger_ptr_list[i]->controller.G_i[1] = 00.0f;   // 1500.0f;
    finger_ptr_list[i]->controller.G_i[2] = 00.0f;   // 10000.0f;

    finger_ptr_list[i]->controller.stalled_thrld_time = 0.5;
    finger_ptr_list[i]->controller.stalled_thrld_err_q = 0.5;
    finger_ptr_list[i]->controller.stalled_thrld_q_dot = 0.5;
    finger_ptr_list[i]->controller.stalled_thrld_tau_m = 0.5;

    finger_ptr_list[i]->controller.ESO_beta1 = 3.0f * ESO_OMEGA0;
    finger_ptr_list[i]->controller.ESO_beta2 = 3.0f * ESO_OMEGA0 * ESO_OMEGA0;
    finger_ptr_list[i]->controller.ESO_beta3 = ESO_OMEGA0 * ESO_OMEGA0 * ESO_OMEGA0;

    finger_ptr_list[i]->controller.force_ramp_rate = 8.0f;
    finger_ptr_list[i]->controller.torque_ramp_rate = 0.1f;

    for (int j = 0; j < finger_ptr_list[i]->finger.joint_num; j++)
    {
      finger_ptr_list[i]->controller.est_z1[j] = finger_ptr_list[i]->finger.cur_q[j];
      finger_ptr_list[i]->controller.est_z2[j] = finger_ptr_list[i]->finger.cur_q_dot[j];
      finger_ptr_list[i]->controller.est_z3[j] = 0.0f;

      finger_ptr_list[i]->controller.bl_cycle[j] = 5;
      finger_ptr_list[i]->controller.bl_current[j] = 50;
      finger_ptr_list[i]->controller.bl_ang[j] = 0.01f;

      TrajectoryQueue_Init(&finger_ptr_list[i]->controller.traj_intp[j].queue);
    }
  }

  // gravity [0 -9.81 0]
  finger_ptr_list[0]->controller.K_d[0] = 650.0f;  // 400
  finger_ptr_list[0]->controller.K_d[1] = 650.0f;  // 500
  finger_ptr_list[0]->controller.K_d[2] = 4500.0f; // 4500
  finger_ptr_list[0]->controller.D_d[0] = 27.0f;   // 5
  finger_ptr_list[0]->controller.D_d[1] = 45.0f;   // 10
  finger_ptr_list[0]->controller.D_d[2] = 200.0f;  // 200
  finger_ptr_list[0]->controller.G_i[0] = 000.0f;
  finger_ptr_list[0]->controller.G_i[1] = 000.0f;
  finger_ptr_list[0]->controller.G_i[2] = 000.0f;
  finger_ptr_list[0]->controller.ffd_acc[0] = 0.0f;
  finger_ptr_list[0]->controller.ffd_acc[1] = 0.0f;
  finger_ptr_list[0]->controller.ffd_acc[2] = 0.0f;
  finger_ptr_list[0]->controller.ffd_vel[0] = 17.0f;
  finger_ptr_list[0]->controller.ffd_vel[1] = 65.0f;
  finger_ptr_list[0]->controller.ffd_vel[2] = 20.0f;
  finger_ptr_list[0]->controller.Ipos_offset[0] = 15.0f;
  finger_ptr_list[0]->controller.Ipos_offset[1] = 10.0f;
  finger_ptr_list[0]->controller.Ipos_offset[2] = 5.0f;
  finger_ptr_list[0]->controller.Ineg_offset[0] = 5.0f;
  finger_ptr_list[0]->controller.Ineg_offset[1] = 10.0f;
  finger_ptr_list[0]->controller.Ineg_offset[2] = 5.0f;
  finger_ptr_list[0]->controller.bl_current[0] = 50.0f;
  finger_ptr_list[0]->controller.bl_current[1] = 70.0f;
  finger_ptr_list[0]->controller.bl_current[2] = 60.0f;
  finger_ptr_list[0]->controller.bl_cycle[0] = 2;
  finger_ptr_list[0]->controller.bl_cycle[1] = 4;
  finger_ptr_list[0]->controller.bl_cycle[2] = 3;
  finger_ptr_list[0]->controller.bl_ang[0] = 0.02f;
  finger_ptr_list[0]->controller.bl_ang[1] = 0.02f;
  finger_ptr_list[0]->controller.bl_ang[2] = 0.01f;

  finger_ptr_list[1]->controller.K_d[0] = 1000.0f;  // 750.0f;
  finger_ptr_list[1]->controller.K_d[1] = 1500.0f;  // 850.0f;
  finger_ptr_list[1]->controller.K_d[2] = 19000.0f; // 15000.0f;
  finger_ptr_list[1]->controller.D_d[0] = 45.0f;    // 8.0f;
  finger_ptr_list[1]->controller.D_d[1] = 80.0f;    // 30.0f;
  finger_ptr_list[1]->controller.D_d[2] = 1500.0f;  // 500.0f;
  finger_ptr_list[1]->controller.G_i[0] = 000.0f;
  finger_ptr_list[1]->controller.G_i[1] = 000.0f;
  finger_ptr_list[1]->controller.G_i[2] = 000.0f;
  finger_ptr_list[1]->controller.ffd_acc[0] = 0.0f;
  finger_ptr_list[1]->controller.ffd_acc[1] = 0.0f;
  finger_ptr_list[1]->controller.ffd_acc[2] = 0.01f;
  finger_ptr_list[1]->controller.ffd_vel[0] = 35.0f; // 15
  finger_ptr_list[1]->controller.ffd_vel[1] = 20.0f;
  finger_ptr_list[1]->controller.ffd_vel[2] = 15.0f;
  finger_ptr_list[1]->controller.Ipos_offset[0] = 5.0f;
  finger_ptr_list[1]->controller.Ipos_offset[1] = 5.0f;
  finger_ptr_list[1]->controller.Ipos_offset[2] = 10.0f;
  finger_ptr_list[1]->controller.Ineg_offset[0] = 0.0f;
  finger_ptr_list[1]->controller.Ineg_offset[1] = 0.0f;
  finger_ptr_list[1]->controller.Ineg_offset[2] = 10.0f;
  finger_ptr_list[1]->controller.bl_current[0] = 65.0f;
  finger_ptr_list[1]->controller.bl_current[1] = 70.0f;
  finger_ptr_list[1]->controller.bl_current[2] = 60.0f;
  finger_ptr_list[1]->controller.bl_cycle[0] = 3;
  finger_ptr_list[1]->controller.bl_cycle[1] = 4;
  finger_ptr_list[1]->controller.bl_cycle[2] = 2;
  finger_ptr_list[1]->controller.bl_ang[0] = 0.025f;
  finger_ptr_list[1]->controller.bl_ang[1] = 0.01f;
  finger_ptr_list[1]->controller.bl_ang[2] = 0.01f;

  finger_ptr_list[2]->controller.K_d[0] = 1100.0f;  // 1500.0f;
  finger_ptr_list[2]->controller.K_d[1] = 1500.0f;  // 1500.0f;
  finger_ptr_list[2]->controller.K_d[2] = 20000.0f; // 20000.0f;
  finger_ptr_list[2]->controller.D_d[0] = 40.0f;    // 15.0f;
  finger_ptr_list[2]->controller.D_d[1] = 70.0f;    // 80.0f;
  finger_ptr_list[2]->controller.D_d[2] = 1000.0f;  // 480.0f;
  finger_ptr_list[2]->controller.G_i[0] = 000.0f;
  finger_ptr_list[2]->controller.G_i[1] = 000.0f;
  finger_ptr_list[2]->controller.G_i[2] = 000.0f;
  finger_ptr_list[2]->controller.ffd_acc[0] = 0.0f;
  finger_ptr_list[2]->controller.ffd_acc[1] = 0.0f;
  finger_ptr_list[2]->controller.ffd_acc[2] = 0.0f;
  finger_ptr_list[2]->controller.ffd_vel[0] = 10.0f;
  finger_ptr_list[2]->controller.ffd_vel[1] = 2.0f;
  finger_ptr_list[2]->controller.ffd_vel[2] = 2.0f;
  finger_ptr_list[2]->controller.Ipos_offset[0] = 5.0f;
  finger_ptr_list[2]->controller.Ipos_offset[1] = 10.0f;
  finger_ptr_list[2]->controller.Ipos_offset[2] = 10.0f;
  finger_ptr_list[2]->controller.Ineg_offset[0] = 5.0f;
  finger_ptr_list[2]->controller.Ineg_offset[1] = 10.0f;
  finger_ptr_list[2]->controller.Ineg_offset[2] = 2.0f;
  finger_ptr_list[2]->controller.bl_current[0] = 60.0f;
  finger_ptr_list[2]->controller.bl_current[1] = 50.0f;
  finger_ptr_list[2]->controller.bl_current[2] = 70.0f;
  finger_ptr_list[2]->controller.bl_cycle[0] = 4;
  finger_ptr_list[2]->controller.bl_cycle[1] = 4;
  finger_ptr_list[2]->controller.bl_cycle[2] = 2;
  finger_ptr_list[2]->controller.bl_ang[0] = 0.01f;
  finger_ptr_list[2]->controller.bl_ang[1] = 0.02f;
  finger_ptr_list[2]->controller.bl_ang[2] = 0.01f;

  // gravity [-9.81 0 0]
  // finger_ptr_list[0]->controller.K_d[0] = 1500.0f;
  // finger_ptr_list[0]->controller.K_d[1] = 500.0f;
  // finger_ptr_list[0]->controller.K_d[2] = 4500.0f;
  // finger_ptr_list[0]->controller.D_d[0] = 5.0f;
  // finger_ptr_list[0]->controller.D_d[1] = 10.0f;
  // finger_ptr_list[0]->controller.D_d[2] = 200.0f;
  // finger_ptr_list[0]->controller.G_i[0] = 000.0f;
  // finger_ptr_list[0]->controller.G_i[1] = 000.0f;
  // finger_ptr_list[0]->controller.G_i[2] = 000.0f;
  // finger_ptr_list[0]->controller.ffd_acc[0] = 0.0f;
  // finger_ptr_list[0]->controller.ffd_acc[1] = 0.0f;
  // finger_ptr_list[0]->controller.ffd_acc[2] = 0.0f;
  // finger_ptr_list[0]->controller.ffd_vel[0] = 100.0f;
  // finger_ptr_list[0]->controller.ffd_vel[1] = 0.0f;
  // finger_ptr_list[0]->controller.ffd_vel[2] = 2.0f;
  // finger_ptr_list[0]->controller.Ipos_offset[0] = 0.0f;
  // finger_ptr_list[0]->controller.Ipos_offset[1] = 0.0f;
  // finger_ptr_list[0]->controller.Ipos_offset[2] = 0.0f;
  // finger_ptr_list[0]->controller.Ineg_offset[0] = 0.0f;
  // finger_ptr_list[0]->controller.Ineg_offset[1] = 0.0f;
  // finger_ptr_list[0]->controller.Ineg_offset[2] = 0.0f;

  // finger_ptr_list[1]->controller.K_d[0] = 750.0f;   // 800.0f;
  // finger_ptr_list[1]->controller.K_d[1] = 850.0f;   // 800.0f;
  // finger_ptr_list[1]->controller.K_d[2] = 15000.0f; // 2000.0f;
  // finger_ptr_list[1]->controller.D_d[0] = 8.0f;     // 25.0f;
  // finger_ptr_list[1]->controller.D_d[1] = 30.0f;    // 25.0f;
  // finger_ptr_list[1]->controller.D_d[2] = 500.0f;   // 2.0f;
  // finger_ptr_list[1]->controller.G_i[0] = 000.0f;
  // finger_ptr_list[1]->controller.G_i[1] = 000.0f;
  // finger_ptr_list[1]->controller.G_i[2] = 000.0f;
  // finger_ptr_list[1]->controller.ffd_acc[0] = 0.0f;
  // finger_ptr_list[1]->controller.ffd_acc[1] = 0.0f;
  // finger_ptr_list[1]->controller.ffd_acc[2] = 0.0f;
  // finger_ptr_list[1]->controller.ffd_vel[0] = 0.0f;
  // finger_ptr_list[1]->controller.ffd_vel[1] = 1.0f;
  // finger_ptr_list[1]->controller.ffd_vel[2] = 0.0f;
  // finger_ptr_list[1]->controller.Ipos_offset[0] = 3.0f;
  // finger_ptr_list[1]->controller.Ipos_offset[1] = 0.0f;
  // finger_ptr_list[1]->controller.Ipos_offset[2] = 0.0f;
  // finger_ptr_list[1]->controller.Ineg_offset[0] = 0.0f;
  // finger_ptr_list[1]->controller.Ineg_offset[1] = 0.0f;
  // finger_ptr_list[1]->controller.Ineg_offset[2] = 0.0f;

  // finger_ptr_list[2]->controller.K_d[0] = 1500.0f;  // 800.0f;
  // finger_ptr_list[2]->controller.K_d[1] = 1500.0f;  // 800.0f;
  // finger_ptr_list[2]->controller.K_d[2] = 20000.0f; // 2000.0f;
  // finger_ptr_list[2]->controller.D_d[0] = 15.0f;    // 25.0f;
  // finger_ptr_list[2]->controller.D_d[1] = 80.0f;    // 25.0f;
  // finger_ptr_list[2]->controller.D_d[2] = 480.0f;   // 2.0f;
  // finger_ptr_list[2]->controller.G_i[0] = 000.0f;
  // finger_ptr_list[2]->controller.G_i[1] = 000.0f;
  // finger_ptr_list[2]->controller.G_i[2] = 000.0f;
  // finger_ptr_list[2]->controller.ffd_acc[0] = 2.0f;
  // finger_ptr_list[2]->controller.ffd_acc[1] = 0.0f;
  // finger_ptr_list[2]->controller.ffd_acc[2] = 0.0f;
  // finger_ptr_list[2]->controller.ffd_vel[0] = 8.0f;
  // finger_ptr_list[2]->controller.ffd_vel[1] = 5.0f;
  // finger_ptr_list[2]->controller.ffd_vel[2] = 2.0f;
  // finger_ptr_list[2]->controller.Ipos_offset[0] = 0.0f;
  // finger_ptr_list[2]->controller.Ipos_offset[1] = 0.0f;
  // finger_ptr_list[2]->controller.Ipos_offset[2] = 15.0f;
  // finger_ptr_list[2]->controller.Ineg_offset[0] = 0.0f;
  // finger_ptr_list[2]->controller.Ineg_offset[1] = 0.0f;
  // finger_ptr_list[2]->controller.Ineg_offset[2] = 0.0f;

  // StateResp initialization
  // ------------------------------------------------------ //
  for (int i = 0; i < FINGER_NUM; i++)
  {
    memset(&finger_ptr_list[i]->finger_SR, 0, sizeof(StateResp));
    memset(&finger_ptr_list[i]->joint_SR, 0, sizeof(StateResp));
    memset(&finger_ptr_list[i]->tf_PSens_SR, 0, sizeof(StateResp));

    // finger_ptr_list[i]->finger_SR.freq = 1.0f;
    // finger_ptr_list[i]->finger_SR.resp_switch = true;
  }

  // CAN initialization
  // ------------------------------------------------------ //
  CAN_Queue_Init(&can_rx_queue);
  CAN_Queue_Init(&can_tx_queue);
  CAN1_FilterInit(0,                     // Bank
                  CAN_FILTERMODE_IDMASK, // Mode
                  CAN_FILTERSCALE_16BIT, // Scale
                  0x0000,                // FilterIdHigh
                  0x0000,                // FilterIdLow
                  0x0000,                // MaskIdHigh
                  0x0000,                // MaskIdLow
                  CAN_FILTER_FIFO0,      // FIFOx
                  ENABLE);               // Activation

  // For testing
  // /////////////////////////////////////////
  // float omega = 2.0f * 3.14159f / 2.0f;
  // for (int i = 0; i < 50; i++)
  // {
  //   TrajectoryPoint point1;
  //   point1.time = i * 0.3f + 0.5f;
  //   point1.pos = 0.2 * sinf(omega * point1.time);
  //   point1.vel = 0.2 * omega * cosf(omega * point1.time);

  //   TrajectoryPoint point2;
  //   point2.time = i * 0.3f + 0.5f;
  //   point2.pos = 0.2 * sinf(omega * point2.time) + 0.6f;
  //   point2.vel = 0.2 * omega * cosf(omega * point2.time);

  //   TrajectoryPoint point3;
  //   point3.time = i * 0.3f + 0.5f;
  //   point3.pos = 0.2 * sinf(omega * point3.time) - 0.3f;
  //   point3.vel = 0.2 * omega * cosf(omega * point3.time);

  //   TrajectoryQueue_Push(&finger_ptr_list[0]->controller.traj_intp[0], &point2);
  //   TrajectoryQueue_Push(&finger_ptr_list[0]->controller.traj_intp[1], &point2);
  //   TrajectoryQueue_Push(&finger_ptr_list[0]->controller.traj_intp[2], &point1);

  //   TrajectoryQueue_Push(&finger_ptr_list[1]->controller.traj_intp[0], &point3);
  //   TrajectoryQueue_Push(&finger_ptr_list[1]->controller.traj_intp[1], &point2);
  //   TrajectoryQueue_Push(&finger_ptr_list[1]->controller.traj_intp[2], &point1);

  //   TrajectoryQueue_Push(&finger_ptr_list[2]->controller.traj_intp[0], &point2);
  //   TrajectoryQueue_Push(&finger_ptr_list[2]->controller.traj_intp[1], &point2);
  //   TrajectoryQueue_Push(&finger_ptr_list[2]->controller.traj_intp[2], &point1);
  // }

  // // for (int i = 0; i < FINGER_NUM; i++)
  // // {
  // //   finger_ptr_list[i]->controller.control_mode = FINGER_POS_CTRL;
  // //   for (int j = 0; j < MAX_JOINT_NUM; j++)
  // //     finger_ptr_list[i]->controller.traj_intp[j].queue_is_enabled = true;
  // // }

  // for (int i = 0; i < 3; i++)
  // {
  //   finger_ptr_list[i]->controller.control_mode = FINGER_POS_CTRL;
  //   for (int j = 0; j < 3; j++)
  //   {
  //     finger_ptr_list[i]->controller.traj_intp[j].queue_is_enabled = true;
  //     TrajectoryPoint first_point;
  //     TrajectoryPoint second_point;
  //     TrajectoryPoint current_point;
  //     current_point.time = 0.0f;
  //     current_point.pos = finger_ptr_list[i]->finger.cur_q[j];
  //     current_point.vel = 0.0f;

  //     if (TrajectoryQueue_Pop(&finger_ptr_list[i]->controller.traj_intp[j].queue, &first_point))
  //     {
  //       // if (TrajectoryQueue_Pop(&finger_ptr_list[i]->controller.traj_intp[j].queue, &second_point))
  //       // {
  //       // }
  //       if (first_point.time > 0.0f)
  //       {
  //         HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
  //         finger_ptr_list[i]->controller.traj_intp[j].start_point = current_point;
  //         finger_ptr_list[i]->controller.traj_intp[j].end_point = first_point;
  //         finger_ptr_list[i]->controller.traj_intp[j].cur_time = 0.0f;
  //       }
  //     }
  //   }
  // }

  // End of variables initialization
  // ------------------------------------------------------ //
  HAL_Delay(500);

  // Start the timers
  HAL_TIM_Base_Start_IT(&htim6); // control timer, 1 MHz, 250 Hz ISR
  HAL_TIM_Base_Start_IT(&htim7); // repeat count timer, 1 MHz counting, 2 kHz ISR
  HAL_TIM_Base_Start(&htim13);   // time measurement timer, 10 kHz counting

  // Enable Can Interrupt
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan1);

  // Enable Torque
  for (int i = 0; i < TOTAL_SERVO_NUM; i++)
  {
    setServo_TorqueENA(&myServoList[i], TORQUE_ENABLE, 10);
  }

  HAL_Delay(100);

  // while (1)
  // {
  //   // CAN_Frame response_frame = {};
  //   // response_frame.id = CANID_RESP_ERROR_CHECK; // Set the response ID
  //   // response_frame.dlc = 8;                     // Set the data length
  //   // response_frame.data[0] = 10;                // Set a specific byte to indicate successful connection

  //   // if (!CAN_Queue_Push(&can_tx_queue, &response_frame))
  //   // {
  //   // }

  //   // CAN_TxHeaderTypeDef TxHeader;
  //   // uint32_t TxMailbox;
  //   // CAN_Frame tx_frame;
  //   // if (CAN_Queue_Pop(&can_tx_queue, &tx_frame))
  //   // {
  //   //   CAN_TxheaderSetup(&TxHeader, tx_frame.id, 0, CAN_ID_STD, CAN_RTR_DATA, tx_frame.dlc, DISABLE);
  //   //   HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_frame.data, &TxMailbox);
  //   // }

  //   // if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
  //   // {
  //   //   CAN_RxHeaderTypeDef rxHeader;
  //   //   HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData);
  //   // }
  //   // HAL_Delay(100);
  // }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // (A). Exception Error Button Pressed
    // ------------------------------------------------------------------------------- //
    if (exc_btn_isPressed || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
    {
      hand_error_code |= ERR_EXCEPTION_BUTTON_PRESSED;
      Error_Handler();
    }

    // (B). CAN data parse and update variable
    // ------------------------------------------------------------------------------- //
    CAN_Frame rx_frame;
    while (CAN_Queue_Pop(&can_rx_queue, &rx_frame))
    {
      // Parse CAN frame and update variables
      switch (rx_frame.id)
      {
      case CANID_PING_REQ:
        CAN_Parsing_PingReq(&rx_frame);
        break;
      case CANID_FINGER_MODE:
        CAN_Parsing_FgrMode(&rx_frame);
        break;
      case CANID_FINGER_MODE_TRIG:
        CAN_Parsing_FgrModeTrig(&rx_frame);
        break;
      case CANID_JOINT_POS_CMD:
        CAN_Parsing_JntPosCmd(&rx_frame);
        break;
      case CANID_JOINT_POS_CMD_ENABLE:
        CAN_Parsing_JntPosCmdEnable(&rx_frame);
        break;
      case CANID_FINGER_CONT_INFO_TRAN:
        CAN_Parsing_FgrContInfoTran(&rx_frame);
        break;
      case CANID_FINGER_CONT_INFO_ROT:
        CAN_Parsing_FgrContInfoRot(&rx_frame);
        break;
      case CANID_FINGER_FORCE_CMD:
        CAN_Parsing_FgrForceCmd(&rx_frame);
        break;
      case CANID_FINGER_TORQUE_CMD:
        CAN_Parsing_FgrTorqueCmd(&rx_frame);
        break;
      case CANID_FINGER_WRENCH_CMD_TRIG:
        CAN_Parsing_FgrWrenchCmdTrig(&rx_frame);
        break;
      case CANID_GRAV_IN_PALM:
        CAN_Parsing_GravInPalm(&rx_frame);
        break;
      case CANID_SETTING_STALLED_PARAM:
        CAN_Parsing_SettingStalled(&rx_frame);
        break;
      case CANID_SETTING_RESP_FINGER_STATE:
        CAN_Parsing_SettingFgrStateResp(&rx_frame);
        break;
      case CANID_SETTING_RESP_JOINT_STATE:
        CAN_Parsing_SettingJntStateResp(&rx_frame);
        break;
      case CANID_SETTING_RESP_TFPSENS:
        CAN_Parsing_SettingTfPSensResp(&rx_frame);
        break;
      case CANID_CLEAR_WARNING:
        CAN_Parsing_ClearWarning(&rx_frame);
        break;
      default:
        break;
      }
    }

    // check state resp
    CAN_AddRespMsg_FgrState();
    CAN_AddRespMsg_JntState();
    CAN_AddRespMsg_TfPSensState();

    // for (int i = 0; i < FINGER_NUM; i++)
    // {
    //   if (finger_ptr_list[i]->finger_SR.resp_switch &&
    //       STATE_COUNTER_PERIOD * finger_ptr_list[i]->finger_SR.counter >= 1000.0f / finger_ptr_list[i]->finger_SR.freq)
    //   {
    //     uint8_t data[8] = {};
    //     data[0] = i;
    //     data[1] = finger_ptr_list[i]->controller.control_mode;
    //     data[2] = 1;

    //     for (int j = 0; j < finger_ptr_list[i]->finger.joint_num; j++)
    //     {
    //       if (finger_ptr_list[i]->controller.traj_intp->traj_progress < 0.9999f)
    //       {
    //         data[2] = 0;
    //         break;
    //       }
    //     }

    //     float max_stall;
    //     int temp;
    //     for (int j = 1; j < finger_ptr_list[i]->finger.joint_num; j++)
    //     {
    //       max_stall = fmax(finger_ptr_list[i]->controller.stalled_level[j],
    //                        finger_ptr_list[i]->controller.stalled_level[j - 1]);
    //     }
    //     temp = (int)roundf(max_stall / CAN_UNIT_BYTE_RESOLUTION);
    //     data[3] = temp > 127 ? 127 : (temp < -128 ? -128 : (uint8_t)temp);

    //     for (int j = 0; j < finger_ptr_list[i]->finger.joint_num; j++)
    //     {
    //       if (finger_ptr_list[i]->finger.warning_code[j])
    //         data[4] |= (1 << j);
    //     }
    //     addCANMsgToQueue(CANID_FINGER_STATE_RESP, 8, data);
    //     finger_ptr_list[i]->finger_SR.counter = 0;
    //   }

    //   if (finger_ptr_list[i]->joint_SR.resp_switch &&
    //       STATE_COUNTER_PERIOD * finger_ptr_list[i]->joint_SR.counter >= 1000.0f / finger_ptr_list[i]->joint_SR.freq)
    //   {
    //     for (int j = 0; j < finger_ptr_list[i]->finger.joint_num; j++)
    //     {
    //       int temp;
    //       uint8_t data[8] = {};
    //       data[0] = i;
    //       data[1] = j;

    //       temp = (int)roundf(finger_ptr_list[i]->controller.traj_intp->traj_progress / CAN_UNIT_BYTE_RESOLUTION);
    //       data[2] = temp > 127 ? 127 : (temp < -128 ? -128 : (uint8_t)temp);

    //       int16_t pos = roundf(finger_ptr_list[i]->finger.cur_q[j] / CAN_POS_RESOLUTION);
    //       data[3] = pos & 0xff;
    //       data[4] = (pos >> 8) & 0xff;

    //       temp = (int)roundf(finger_ptr_list[i]->controller.stalled_level[j] / CAN_UNIT_BYTE_RESOLUTION);
    //       data[5] = temp > 127 ? 127 : (temp < -128 ? -128 : (uint8_t)temp);

    //       data[6] = finger_ptr_list[i]->finger.warning_code[j] & 0xff;
    //       data[7] = (finger_ptr_list[i]->finger.warning_code[j] >> 8) & 0xff;
    //       addCANMsgToQueue(CANID_JOINT_STATE_RESP, 8, data);
    //     }
    //     finger_ptr_list[i]->joint_SR.counter = 0;
    //   }

    //   if (finger_ptr_list[i]->tf_PSens_SR.resp_switch &&
    //       STATE_COUNTER_PERIOD * finger_ptr_list[i]->tf_PSens_SR.counter >= 1000.0f / finger_ptr_list[i]->tf_PSens_SR.freq)
    //   {
    //     float param[6] = {};
    //     transform2Param(&finger_ptr_list[i]->finger.tf_PSens, param);

    //     uint8_t data[8] = {};
    //     int temp;
    //     data[0] = i;

    //     for (int j = 0; j < 6; j++)
    //     {
    //       if (j < 3)
    //         temp = (int)roundf(param[j] / CAN_TF_TRANS_RESOLUTION);
    //       else
    //         temp = (int)roundf(param[j] / CAN_TF_ROT_RESOLUTION);
    //       data[j + 1] = temp > 127 ? 127 : (temp < -128 ? -128 : (uint8_t)temp);
    //     }
    //     addCANMsgToQueue(CANID_TF_PSENS_TRAN_RESP, 8, data);
    //     finger_ptr_list[i]->tf_PSens_SR.counter = 0;
    //   }
    // }

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0 && !CAN_Queue_IsEmpty(&can_tx_queue))
    {
      CAN_TxHeaderTypeDef TxHeader;
      uint32_t TxMailbox;
      CAN_Frame tx_frame;
      if (CAN_Queue_Pop(&can_tx_queue, &tx_frame))
      {
        CAN_TxheaderSetup(&TxHeader, tx_frame.id, 0, CAN_ID_STD, CAN_RTR_DATA, tx_frame.dlc, DISABLE);
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_frame.data, &TxMailbox);
      }
    }

    // (C). Control Loop
    // ------------------------------------------------------------------------------- //
    if (controlFlag)
    {
      // (c1). Start time measurement
      // ------------------------------------------------ //
      int time_start;
      time_start = __HAL_TIM_GET_COUNTER(&htim13);

      // (c2). Update servo and joint information
      // ------------------------------------------------ //
      // Update cur_q, cur_q_dot, cur_current of servo
      // if (!getServoFeedback_thumb())
      if (!getServoFeedback())
      {
        for (int i = 0; i < TOTAL_SERVO_NUM; i++)
        {
          hand_error_code |= servoException(&myServoList[i], 10);
          if (hand_error_code != 0)
            Error_Handler();
        }
      }
      // (c3). Limitation Check
      // ------------------------------------------------ //

      // (c4). Update g_base of each finger
      // ------------------------------------------------ //
      for (int i = 0; i < FINGER_NUM; i++)
        updateFingerGravity(&finger_ptr_list[i]->finger, g_palm);

      // (c5). Update dynamics of each finger
      // ------------------------------------------------ //
      for (int i = 0; i < FINGER_NUM; i++)
      {
        updateFingerDynamics(&finger_ptr_list[i]->finger);
      }

      // (c6). Finger level controller.
      //       Controller computes the desired torque of joint.
      // ------------------------------------------------ //
      // finger_ptr_list[0]->controller.D_d[0] = param_d1;
      // finger_ptr_list[0]->controller.D_d[1] = param_d2;
      // finger_ptr_list[0]->controller.D_d[2] = param_d3;

      // finger_ptr_list[0]->controller.K_d[0] = param_k1;
      // finger_ptr_list[0]->controller.K_d[1] = param_k2;
      // finger_ptr_list[0]->controller.K_d[2] = param_k3;
      for (int i = 0; i < FINGER_NUM; i++)
      {
        if (finger_ptr_list[i]->controller.control_mode == FINGER_POS_CTRL)
        {
          for (int j = 0; j < finger_ptr_list[i]->finger.joint_num; j++)
          {
            jointCmdGeneration(&finger_ptr_list[i]->controller.traj_intp[j], finger_ptr_list[i]->controller.control_period);
            if (finger_ptr_list[i]->controller.traj_intp[j].traj_progress < 0.9999f &&
                finger_ptr_list[i]->controller.traj_intp[j].traj_progress > 0.01f &&
                finger_ptr_list[i]->controller.traj_intp[j].queue_is_enabled == false)
            {
              finger_ptr_list[i]->controller.warning_code[j] |= WARNING_TRAJ_ABORTED;
            }
          }

          fingerEndPointFK(&finger_ptr_list[i]->finger, i, finger_ptr_list[i]->controller.p_cmd);
          jointAngController_InvDym(&finger_ptr_list[i]->finger, &finger_ptr_list[i]->controller);
        }
        else if (finger_ptr_list[i]->controller.control_mode == FINGER_FORCE_CTRL)
        {
          FingerController *ctrl = &finger_ptr_list[i]->controller;
          // ????? f_cmd_org ???
          ForceRamp_Update(&ctrl->f_ramp_force,
                           &ctrl->f_cmd_org[0],
                           ctrl->force_ramp_rate,
                           ctrl->control_period,
                           ctrl->contact_f_num_force);

          // ?????? f_cmd_org ???
          ForceRamp_Update(&ctrl->f_ramp_torque,
                           &ctrl->f_cmd_org[ctrl->contact_f_num_force],
                           ctrl->torque_ramp_rate,
                           ctrl->control_period,
                           ctrl->contact_f_num_torque);

          float force_cmd[6] = {};
          for (int j = 0; j < ctrl->contact_f_num_force; j++)
            force_cmd[j] = ctrl->f_ramp_force.f_cmd[j];
          for (int j = 0; j < ctrl->contact_f_num_torque; j++)
            force_cmd[ctrl->contact_f_num_force + j] = ctrl->f_ramp_torque.f_cmd[j];

          computeProjF_4x3_FR(&finger_ptr_list[i]->finger, ctrl, force_cmd);
          fingerForceCtrl_Open(&finger_ptr_list[i]->finger, ctrl);
        }
        else if (finger_ptr_list[i]->controller.control_mode == FINGER_IDLE)
        {
          for (int j = 0; j < finger_ptr_list[i]->finger.joint_num; j++)
          {
            finger_ptr_list[i]->controller.tau_d[j] = 0.0f;
          }
          // thumb_sys.controller.traj_intp[0].q_cmd = 0.0f;
          // thumb_sys.controller.traj_intp[1].q_cmd = 0.0f;
          // thumb_sys.controller.traj_intp[2].q_cmd = 0.0f;
        }
      }

      // (c7). Compute goal_current from torque command
      // ------------------------------------------------ //
      // finger_ptr_list[1]->controller.tau_d[0] = 0.0f;
      // finger_ptr_list[1]->controller.tau_d[1] = 0.0f;
      // finger_ptr_list[1]->controller.tau_d[2] = 0.0f;

      // finger_ptr_list[2]->controller.tau_d[0] = 0.0f;
      // finger_ptr_list[2]->controller.tau_d[1] = 0.0f;
      // finger_ptr_list[2]->controller.tau_d[2] = 0.0f;

      // finger_ptr_list[0]->controller.tau_d[0] = 0.0f;
      // finger_ptr_list[0]->controller.tau_d[1] = 0.0f;
      // finger_ptr_list[0]->controller.tau_d[2] = 0.0f;
      for (int i = 0; i < FINGER_NUM; i++)
      {
        computeGoalCurrent(&finger_ptr_list[i]->finger, &finger_ptr_list[i]->controller);
      }
      // finger_ptr_list[0]->controller.goal_current[0] = 100.0f;
      // finger_ptr_list[0]->controller.goal_current[1] = 81.0f;
      // finger_ptr_list[0]->controller.goal_current[2] = 17.0f;
      // thumb_sys.controller.goal_current[0] = fabsf(thumb_sys.controller.goal_current[0]) > 25.0f ? thumb_sys.controller.goal_current[0] : 0.0f;

      // (c8). Get goal_current_list and send to the servo
      // ------------------------------------------------ //
      for (int i = 0; i < THUMB_JOINT_NUM; i++)
        goal_current_list[0 + i] = thumb_sys.controller.goal_current[i];
      for (int i = 0; i < INDEX_JOINT_NUM; i++)
        goal_current_list[THUMB_JOINT_NUM + i] = index_sys.controller.goal_current[i];
      for (int i = 0; i < MIDDLE_JOINT_NUM; i++)
        goal_current_list[THUMB_JOINT_NUM + INDEX_JOINT_NUM + i] = middle_sys.controller.goal_current[i];

      // setServo_Position_PGain(&myServoList[0], p_param, 1);
      // setServo_Position_DGain(&myServoList[0], d_param, 1);
      // getServo_Position_PGain(&myServoList[0], 1);
      // getServo_Position_DGain(&myServoList[0], 1);

      // goal_position_list[0] = thumb_sys.controller.traj_intp[0].q_cmd * RAD_2_DEG + thumb_sys.finger.homing_offset[0];
      // goal_position_list[1] = thumb_sys.controller.traj_intp[1].q_cmd * RAD_2_DEG + thumb_sys.finger.homing_offset[1];
      // goal_position_list[2] = thumb_sys.controller.traj_intp[2].q_cmd * RAD_2_DEG + thumb_sys.finger.homing_offset[2];
      // syncWrite_GoalPosition(myServoList, 1, goal_position_list);
      // syncWrite_GoalCurrent(myServoList, 3, goal_current_list);
      syncWrite_GoalCurrent(myServoList, TOTAL_SERVO_NUM, goal_current_list);

      // (c9). End of control loop
      // ------------------------------------------------ //
      controlFlag = false;
      time_pass = __HAL_TIM_GET_COUNTER(&htim13) - time_start;
      __HAL_TIM_SET_COUNTER(&htim13, 0);
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void fingerEndPointFK(Finger *finger, int finger_idx, float end_point[3])
{
  float end[3] = {};
  if (finger_idx == thumb_idx)
  {
    // thumb forward kinematics
    float s1 = sinf(finger->cur_q[0]);
    float c1 = cosf(finger->cur_q[0]);
    float s2 = sinf(finger->cur_q[1]);
    float c2 = cosf(finger->cur_q[1]);
    float s23 = sinf(finger->cur_q[1] + finger->cur_q[2]);
    float c23 = cosf(finger->cur_q[1] + finger->cur_q[2]);
    float temp1 = THUMB_DH_A2 * s2 + THUMB_DH_A3 * s23;
    float temp2 = THUMB_DH_A1 + THUMB_DH_A2 * c2 + THUMB_DH_A3 * c23;
    end[0] = c1 * temp2 - 0.5f * s1 * temp1;
    end[1] = s1 * temp2 + 0.5f * c1 * temp1;
    end[2] = -0.8660254 * temp1; // sqrt(3)/2 = 0.8660254
  }
  else if (finger_idx == index_idx)
  {
    // index finger forward kinematics
    float s1 = sinf(finger->cur_q[0]);
    float c1 = cosf(finger->cur_q[0]);
    float s2 = sinf(finger->cur_q[1]);
    float c2 = cosf(finger->cur_q[1]);
    float s23 = sinf(finger->cur_q[1] + finger->cur_q[2]);
    float c23 = cosf(finger->cur_q[1] + finger->cur_q[2]);
    float temp1 = INDEX_DH_A1 + INDEX_DH_A2 * c2 + INDEX_DH_A3 * c23;
    end[0] = temp1 * c1;
    end[1] = temp1 * s1;
    end[2] = INDEX_DH_A2 * s2 + INDEX_DH_A3 * s23;
  }
  else if (finger_idx == middle_idx)
  {
    // middle finger forward kinematics
    float s1 = sinf(finger->cur_q[0]);
    float c1 = cosf(finger->cur_q[0]);
    float s2 = sinf(finger->cur_q[1]);
    float c2 = cosf(finger->cur_q[1]);
    float s23 = sinf(finger->cur_q[1] + finger->cur_q[2]);
    float c23 = cosf(finger->cur_q[1] + finger->cur_q[2]);
    float temp1 = MIDDLE_DH_A1 + MIDDLE_DH_A2 * c2 + MIDDLE_DH_A3 * c23;
    end[0] = temp1 * c1;
    end[1] = temp1 * s1;
    end[2] = MIDDLE_DH_A2 * s2 + MIDDLE_DH_A3 * s23;
  }

  applyTransformation(&finger->tf_PS, end, end_point);
}

void computeGoalCurrent(const Finger *finger, FingerController *controller)
{
  for (int i = 0; i < finger->joint_num; i++)
  {
    float cur_q_dot = finger->cur_q_dot[i];
    float tau_d = controller->tau_d[i];
    float tau_g = finger->tau_grav[i];
    float ctrl_goal_current = tau_d * SERVO_KT_C1;
    float grav_goal_current = tau_g * SERVO_KT_C1;
    float total_goal_current = controller->goal_current[i]; // Á∂≠Ê?Å‰?äÊ¨°???

    // if (fabsf(tau_d) < 0.0002f && cur_q_dot == 0.0f)
    //   goal_current = tau_d * SERVO_KT_C1; // in mA
    // else if (cur_q_dot > 0.0f)
    //   goal_current = tau_d * SERVO_KT_C1 + SERVO_KT_C2_POS + 0.5f * cur_q_dot; // in mA
    // else if (cur_q_dot < 0.0f)
    //   goal_current = tau_d * SERVO_KT_C1 + SERVO_KT_C2_NEG + 0.5f * cur_q_dot; // in mA
    // else
    //   goal_current = tau_d * SERVO_KT_C1;

    int sign_tau_d = (tau_d >= 0) - (tau_d <= 0);
    float I_coulomb = 0.0f;
    if (fabsf(controller->err_q[i]) > controller->bl_ang[i])
    {
      controller->bl_trig_cntr[i] = fminf(controller->bl_trig_cntr[i] + 1, DEADZONE_TRIG_CNTR);

      // if (controller->bl_trig_cntr[i] >= DEADZONE_TRIG_CNTR)
      if (1)
      {
        if (sign_tau_d > 0)
          I_coulomb = controller->Ipos_offset[i];
        else if (sign_tau_d < 0)
          I_coulomb = controller->Ineg_offset[i];
        // total_goal_current = ctrl_goal_current +
        //                      I_coulomb * sign_tau_d +
        //                      controller->ffd_vel[i] * controller->traj_intp[i].q_dot_cmd +
        //                      controller->ffd_acc[i] * controller->traj_intp[i].q_dot2_cmd;
        // controller->prev_goal_current[i] = total_goal_current;
      }
    }
    // else
    // {
    //   controller->bl_trig_cntr[i] = fmaxf(controller->bl_trig_cntr[i] - 1, -1);

    //   if (controller->bl_trig_cntr[i] < 0)
    //   // if (1)
    //   {
    //     if (ctrl_goal_current == 0.0f)
    //       total_goal_current = 0.0f;
    //     else
    //       // total_goal_current = controller->prev_goal_current[i];
    //       total_goal_current = grav_goal_current;
    //   }
    // }

    if (controller->control_mode == FINGER_POS_CTRL)
    {
      total_goal_current = ctrl_goal_current +
                           I_coulomb * sign_tau_d +
                           controller->ffd_vel[i] * controller->traj_intp[i].q_dot_cmd +
                           controller->ffd_acc[i] * controller->traj_intp[i].q_dot2_cmd;

      controller->cur_sign[i] = (total_goal_current > 4.0f) ? 1 : ((total_goal_current < -4.0f) ? -1 : 0);
      controller->dir_changed[i] = (controller->cur_sign[i] != 0 &&
                                    controller->prev_sign[i] != 0 &&
                                    controller->cur_sign[i] != controller->prev_sign[i]);
      if (controller->cur_sign[i] != 0)
        controller->prev_sign[i] = controller->cur_sign[i];

      if (controller->dir_changed[i] &&
          controller->bl_state[i] == BL_STATE_NORMAL)
      {
        controller->bl_state[i] = BL_STATE_CROSSING;
        controller->bl_cntr[i] = controller->bl_cycle[i];
      }

      if (controller->bl_state[i] == BL_STATE_CROSSING)
      {
        controller->bl_cntr[i] -= 1;
        if (controller->bl_cntr[i] <= 0)
        {
          controller->bl_state[i] = BL_STATE_NORMAL;
        }
        controller->goal_current[i] = controller->bl_current[i] * controller->prev_sign[i];
      }
      else
        controller->goal_current[i] = total_goal_current;

      // controller->goal_current[i] = total_goal_current;
    }
    else // if (controller->control_mode == FINGER_FORCE_CTRL)
    {
      controller->goal_current[i] = ctrl_goal_current;
    }
  }
}

// Get joint states from DYNAMIXEL motors
// ---------------------------------------------------------- //
bool getServoFeedback_thumb(void)
{
  // Get joint angle
  if (syncRead_PresentPosition(myServoList, 3, cur_ang_list, 5))
  {
    for (int i = 0; i < THUMB_JOINT_NUM; i++)
      thumb_sys.finger.cur_q[i] = (cur_ang_list[i] - thumb_sys.finger.homing_offset[i]) * DEG_2_RAD;
  }
  else
  {
    return false;
  }

  // Get joint angular velocity
  if (syncRead_PresentVelocity(myServoList, 3, cur_vel_list, 5))
  {
    for (int i = 0; i < THUMB_JOINT_NUM; i++)
      thumb_sys.finger.cur_q_dot[i] = cur_vel_list[i] * RPM_2_RADS;
  }
  else
  {
    return false;
  }

  //  Get servo current of the joint
  if (syncRead_PresentCurrent(myServoList, 3, cur_current_list, 5))
  {
    for (int i = 0; i < THUMB_JOINT_NUM; i++)
      thumb_sys.finger.cur_current[i] = cur_current_list[i];
  }
  else
  {
    return false;
  }

  return true;
}

bool getServoFeedback(void)
{
  // Get joint angle
  if (syncRead_PresentPosition(myServoList, TOTAL_SERVO_NUM, cur_ang_list, 5))
  {
    for (int i = 0; i < THUMB_JOINT_NUM; i++)
      thumb_sys.finger.cur_q[i] = (cur_ang_list[i] - thumb_sys.finger.homing_offset[i]) * DEG_2_RAD;
    for (int i = 0; i < INDEX_JOINT_NUM; i++)
      index_sys.finger.cur_q[i] = (cur_ang_list[THUMB_JOINT_NUM + i] - index_sys.finger.homing_offset[i]) * DEG_2_RAD;
    for (int i = 0; i < MIDDLE_JOINT_NUM; i++)
      middle_sys.finger.cur_q[i] = (cur_ang_list[THUMB_JOINT_NUM + INDEX_JOINT_NUM + i] - middle_sys.finger.homing_offset[i]) * DEG_2_RAD;
  }
  else
  {
    return false;
  }

  // // Get joint angular velocity using PresentVelocity of DYNAMIXEL Motor
  // if (syncRead_PresentVelocity(myServoList, TOTAL_SERVO_NUM, cur_vel_list, 5))
  // {
  //   for (int i = 0; i < THUMB_JOINT_NUM; i++)
  //     thumb_sys.finger.cur_q_dot[i] = cur_vel_list[i] * RPM_2_RADS;
  //   for (int i = 0; i < INDEX_JOINT_NUM; i++)
  //     index_sys.finger.cur_q_dot[i] = cur_vel_list[THUMB_JOINT_NUM + i] * RPM_2_RADS;
  //   for (int i = 0; i < MIDDLE_JOINT_NUM; i++)
  //     middle_sys.finger.cur_q_dot[i] = cur_vel_list[THUMB_JOINT_NUM + INDEX_JOINT_NUM + i] * RPM_2_RADS;
  // }
  // else
  // {
  //   return false;
  // }

  ///////////////////////////////////////////////

  // Get joint angular velocity (finite difference + Tustin-type low-pass filter)
  {

    for (int i = 0; i < THUMB_JOINT_NUM; i++)
    {
      float period = thumb_sys.controller.control_period;
      float raw_vel = period != 0.0 ? (thumb_sys.finger.cur_q[i] - thumb_sys.finger.prev_q[i]) / period : 0.0f;
      float alpha = thumb_sys.finger.vel_filter_alpha[i];
      thumb_sys.finger.cur_q_dot[i] = alpha * thumb_sys.finger.cur_q_dot[i] +
                                      (1.0f - alpha) * 0.5f * (raw_vel + thumb_sys.finger.prev_q_dot_raw[i]);
      thumb_sys.finger.cur_q_dot_raw[i] = raw_vel;
      thumb_sys.finger.prev_q_dot_raw[i] = raw_vel;
      thumb_sys.finger.prev_q[i] = thumb_sys.finger.cur_q[i];
    }

    for (int i = 0; i < INDEX_JOINT_NUM; i++)
    {
      float period = index_sys.controller.control_period;
      float raw_vel = period != 0.0 ? (index_sys.finger.cur_q[i] - index_sys.finger.prev_q[i]) / period : 0.0f;
      float alpha = index_sys.finger.vel_filter_alpha[i];
      index_sys.finger.cur_q_dot[i] = alpha * index_sys.finger.cur_q_dot[i] +
                                      (1.0f - alpha) * 0.5f * (raw_vel + index_sys.finger.prev_q_dot_raw[i]);
      index_sys.finger.cur_q_dot_raw[i] = raw_vel;
      index_sys.finger.prev_q_dot_raw[i] = raw_vel;
      index_sys.finger.prev_q[i] = index_sys.finger.cur_q[i];
    }

    for (int i = 0; i < MIDDLE_JOINT_NUM; i++)
    {
      float period = middle_sys.controller.control_period;
      float raw_vel = period != 0.0 ? (middle_sys.finger.cur_q[i] - middle_sys.finger.prev_q[i]) / period : 0.0f;
      float alpha = middle_sys.finger.vel_filter_alpha[i];
      middle_sys.finger.cur_q_dot[i] = alpha * middle_sys.finger.cur_q_dot[i] +
                                       (1.0f - alpha) * 0.5f * (raw_vel + middle_sys.finger.prev_q_dot_raw[i]);
      middle_sys.finger.cur_q_dot_raw[i] = raw_vel;
      middle_sys.finger.prev_q_dot_raw[i] = raw_vel;
      middle_sys.finger.prev_q[i] = middle_sys.finger.cur_q[i];
    }
  }

  //////////////////

  //  Get servo current of the joint
  if (syncRead_PresentCurrent(myServoList, TOTAL_SERVO_NUM, cur_current_list, 5))
  {
    for (int i = 0; i < THUMB_JOINT_NUM; i++)
      thumb_sys.finger.cur_current[i] = cur_current_list[i];
    for (int i = 0; i < INDEX_JOINT_NUM; i++)
      index_sys.finger.cur_current[i] = cur_current_list[THUMB_JOINT_NUM + i];
    for (int i = 0; i < MIDDLE_JOINT_NUM; i++)
      middle_sys.finger.cur_current[i] = cur_current_list[THUMB_JOINT_NUM + INDEX_JOINT_NUM + i];
  }
  else
  {
    return false;
  }

  //////////////////////

  return true;
}

// CAN1 parsing function
// ---------------------------------------------------------- //
void CAN_Parsing_PingReq(CAN_Frame *frame)
{
  uint8_t data[8] = {};
  addCANMsgToQueue(CANID_PING_RESP, 8, data);
}

void CAN_Parsing_FgrMode(CAN_Frame *frame)
{
  int finger_ID = frame->data[0];
  if (finger_ID >= 0 && finger_ID < FINGER_NUM)
  {
    finger_ptr_list[finger_ID]->controller.control_mode_buffer = frame->data[1];
  }
  else
    return;
}

void CAN_Parsing_FgrModeTrig(CAN_Frame *frame)
{
  for (int i = 0; i < FINGER_NUM; i++)
  {
    if ((frame->data[0] & (1 << i)) != 0)
    {
      if (finger_ptr_list[i]->controller.control_mode != FINGER_POS_CTRL &&
          finger_ptr_list[i]->controller.control_mode_buffer == FINGER_POS_CTRL)
      {

        for (int j = 0; j < finger_ptr_list[i]->finger.joint_num; j++)
        {
          TrajectoryPoint current_point = {};
          current_point.pos = finger_ptr_list[i]->finger.cur_q[j];
          current_point.vel = 0.0f;
          current_point.time = 0.0f;
          if (finger_ptr_list[i]->controller.traj_intp[j].queue_is_enabled == true)
          {
            if (!firstSectionUpdate(&finger_ptr_list[i]->controller.traj_intp[j], &current_point))
            {
              // Clear the trajectory queue
              // finger_ptr_list[i]->controller.traj_intp[j].queue.count = 0;
              // finger_ptr_list[i]->controller.traj_intp[j].queue.head = 0;
              // finger_ptr_list[i]->controller.traj_intp[j].queue.tail = 0;
              // finger_ptr_list[i]->controller.traj_intp[j].traj_progress = 0.0f;

              finger_ptr_list[i]->controller.traj_intp[j].queue_is_enabled = false;
              finger_ptr_list[i]->controller.traj_intp[j].end_point = current_point;

              finger_ptr_list[i]->controller.warning_code[j] |= WARNING_NO_TRAJ_IN_QUEUE;
            }
          }
          else
          {
            finger_ptr_list[i]->controller.traj_intp[j].end_point = current_point;
          }
        }
      }
      else if (finger_ptr_list[i]->controller.control_mode == FINGER_POS_CTRL &&
               finger_ptr_list[i]->controller.control_mode_buffer != FINGER_POS_CTRL)
      {
        for (int j = 0; j < finger_ptr_list[i]->finger.joint_num; j++)
        {
          // Clear the trajectory queue
          finger_ptr_list[i]->controller.traj_intp[j].queue.count = 0;
          finger_ptr_list[i]->controller.traj_intp[j].queue.head = 0;
          finger_ptr_list[i]->controller.traj_intp[j].queue.tail = 0;
          finger_ptr_list[i]->controller.traj_intp[j].traj_progress = 0.0f;
          finger_ptr_list[i]->controller.traj_intp[j].queue_is_enabled = false;
        }
      }

      if (finger_ptr_list[i]->controller.control_mode == FINGER_FORCE_CTRL &&
          finger_ptr_list[i]->controller.control_mode_buffer != FINGER_FORCE_CTRL)
      {
        // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
        ForceRamp_Reset(&finger_ptr_list[i]->controller.f_ramp_force, 3);
        ForceRamp_Reset(&finger_ptr_list[i]->controller.f_ramp_torque, 3);
        memset(&finger_ptr_list[i]->controller.f_cmd_buffer, 0, sizeof(finger_ptr_list[i]->controller.f_cmd_buffer));
        memset(&finger_ptr_list[i]->controller.f_cmd_org, 0, sizeof(finger_ptr_list[i]->controller.f_cmd_org));
        memset(&finger_ptr_list[i]->controller.f_cmd_proj, 0, sizeof(finger_ptr_list[i]->controller.f_cmd_proj));
        finger_ptr_list[i]->controller.contact_f_num_force = 0;
        finger_ptr_list[i]->controller.contact_f_num_torque = 0;
      }

      finger_ptr_list[i]->controller.control_mode = finger_ptr_list[i]->controller.control_mode_buffer;
    }
  }
}

void CAN_Parsing_JntPosCmd(CAN_Frame *frame)
{
  int finger_ID = frame->data[0];
  int joint_ID = frame->data[1];
  if (finger_ID >= 0 && finger_ID < FINGER_NUM)
  {
    if (joint_ID >= 0 && joint_ID < finger_ptr_list[finger_ID]->finger.joint_num)
    {
      TrajectoryPoint cmd_point;
      uint16_t time = (uint16_t)(((uint16_t)frame->data[2]) |
                                 ((uint16_t)frame->data[3] << 8));
      int16_t pos = (int16_t)(((uint16_t)frame->data[4]) |
                              ((uint16_t)frame->data[5] << 8));
      int16_t vel = (int16_t)(((uint16_t)frame->data[6]) |
                              ((uint16_t)frame->data[7] << 8));

      cmd_point.time = (float)time * CAN_TIME_RESOLUTION;
      cmd_point.pos = (float)pos * CAN_POS_RESOLUTION;
      cmd_point.vel = (float)vel * CAN_VEL_RESOLUTION;
      if (!TrajectoryQueue_Push(&finger_ptr_list[finger_ID]->controller.traj_intp[joint_ID].queue, &cmd_point))
      {
        // error_code |= ERR_TRAJECTORY_QUEUE_FULL;
        finger_ptr_list[finger_ID]->controller.warning_code[joint_ID] |= WARNING_TRAJECTORY_QUEUE_FULL;
        return;
      }
      else
      {
        finger_ptr_list[finger_ID]->controller.traj_intp[joint_ID].final_time = cmd_point.time;
        return;
      }
    }
    else
      return;
  }
  else
    return;
}

void CAN_Parsing_JntPosCmdEnable(CAN_Frame *frame)
{
  for (int i = 0; i < FINGER_NUM; i++)
  {
    for (int j = 0; j < finger_ptr_list[i]->finger.joint_num; j++)
    {
      if (finger_ptr_list[i]->controller.control_mode == FINGER_POS_CTRL)
      {
        // First time to enable the trajectory interpolation
        if ((finger_ptr_list[i]->controller.traj_intp[j].queue_is_enabled == false) &&
            ((frame->data[0] & (1 << i)) != 0))
        {
          TrajectoryPoint current_point = {};
          current_point.time = 0.0f;
          current_point.pos = finger_ptr_list[i]->controller.traj_intp[j].end_point.pos;
          current_point.vel = 0.0f;
          if (!firstSectionUpdate(&finger_ptr_list[i]->controller.traj_intp[j], &current_point))
          {
            // Clear the trajectory queue
            // finger_ptr_list[i]->controller.traj_intp[j].queue.count = 0;
            // finger_ptr_list[i]->controller.traj_intp[j].queue.head = 0;
            // finger_ptr_list[i]->controller.traj_intp[j].queue.tail = 0;
            // finger_ptr_list[i]->controller.traj_intp[j].traj_progress = 0.0f;
            finger_ptr_list[i]->controller.warning_code[j] |= WARNING_NO_TRAJ_IN_QUEUE;
          }
        }
        // Disable the trajectory interpolation and hold the current position
        else if ((finger_ptr_list[i]->controller.traj_intp[j].queue_is_enabled == true) &&
                 ((frame->data[0] & (1 << i)) == 0))
        {
          // Clear the trajectory queue
          finger_ptr_list[i]->controller.traj_intp[j].queue.count = 0;
          finger_ptr_list[i]->controller.traj_intp[j].queue.head = 0;
          finger_ptr_list[i]->controller.traj_intp[j].queue.tail = 0;

          // Set the endpoint as the current position command
          finger_ptr_list[i]->controller.traj_intp[j].end_point.pos = finger_ptr_list[i]->controller.traj_intp[j].q_cmd;
          finger_ptr_list[i]->controller.traj_intp[j].end_point.time = finger_ptr_list[i]->controller.traj_intp[j].cur_time;
          finger_ptr_list[i]->controller.traj_intp[j].end_point.vel = 0.0f;
        }
      }

      finger_ptr_list[i]->controller.traj_intp[j].queue_is_enabled = (frame->data[0] >> i) & 1;
    }
  }
}

void CAN_Parsing_FgrContInfoTran(CAN_Frame *frame)
{
  int finger_ID = frame->data[0];

  if (finger_ID >= 0 && finger_ID < FINGER_NUM)
  {
    // wrench basis part
    int f_num = 0;
    for (int i = 5; i >= 0; i--)
    {
      bool val = (frame->data[1] >> i) & 1;
      finger_ptr_list[finger_ID]->finger.sensors.wrench_basis[5 - i] = val;
      if (val)
        f_num++;
    }
    finger_ptr_list[finger_ID]->finger.sensors.contact_f_num = f_num;

    int f_num_force = 0, f_num_torque = 0;
    for (int i = 0; i < 3; i++)
    {
      if (finger_ptr_list[finger_ID]->finger.sensors.wrench_basis[i])
        f_num_force++;
    }
    for (int i = 3; i < 6; i++)
    {
      if (finger_ptr_list[finger_ID]->finger.sensors.wrench_basis[i])
        f_num_torque++;
    }
    finger_ptr_list[finger_ID]->controller.contact_f_num_force = f_num_force;
    finger_ptr_list[finger_ID]->controller.contact_f_num_torque = f_num_torque;

    // tf_SensC part
    // int8_t x = frame->data[2];
    // int8_t y = frame->data[3];
    // int8_t z = frame->data[4];
    // int8_t roll = frame->data[5];
    // int8_t pitch = frame->data[6];
    // int8_t yaw = frame->data[7];
    // finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.param[0] = x * CAN_TF_TRANS_RESOLUTION;                         // x
    // finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.param[1] = y * CAN_TF_TRANS_RESOLUTION;   // y
    // finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.param[2] = z * CAN_TF_TRANS_RESOLUTION;   // z
    // finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.param[3] = roll * CAN_TF_ROT_RESOLUTION;  // roll
    // finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.param[4] = pitch * CAN_TF_ROT_RESOLUTION; // pitch
    // finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.param[5] = yaw * CAN_TF_ROT_RESOLUTION;   // yaw

    int16_t x = (int16_t)(((uint16_t)frame->data[2]) |
                          ((uint16_t)frame->data[3] << 8));
    int16_t y = (int16_t)(((uint16_t)frame->data[4]) |
                          ((uint16_t)frame->data[5] << 8));
    int16_t z = (int16_t)(((uint16_t)frame->data[6]) |
                          ((uint16_t)frame->data[7] << 8));

    // float param[6] = {};
    // param[0] = ((int8_t)frame->data[2]) * CAN_TF_TRANS_RESOLUTION;
    // param[1] = ((int8_t)frame->data[3]) * CAN_TF_TRANS_RESOLUTION;
    // param[2] = ((int8_t)frame->data[4]) * CAN_TF_TRANS_RESOLUTION;
    // param[3] = ((int8_t)frame->data[5]) * CAN_TF_ROT_RESOLUTION;
    // param[4] = ((int8_t)frame->data[6]) * CAN_TF_ROT_RESOLUTION;
    // param[5] = ((int8_t)frame->data[7]) * CAN_TF_ROT_RESOLUTION;
    // param2Transform(param, &finger_ptr_list[finger_ID]->finger.sensors.tf_SensC);

    finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.p[0] = (float)x * CAN_TF_TRANS_RESOLUTION;
    finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.p[1] = (float)y * CAN_TF_TRANS_RESOLUTION;
    finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.p[2] = (float)z * CAN_TF_TRANS_RESOLUTION;
  }
  else
    return;
}

void CAN_Parsing_FgrContInfoRot(CAN_Frame *frame)
{
  int finger_ID = frame->data[0];

  if (finger_ID >= 0 && finger_ID < FINGER_NUM)
  {
    // wrench basis part
    int f_num = 0;
    for (int i = 5; i >= 0; i--)
    {
      bool val = (frame->data[1] >> i) & 1;
      finger_ptr_list[finger_ID]->finger.sensors.wrench_basis[5 - i] = val;
      if (val)
        f_num++;
    }
    finger_ptr_list[finger_ID]->finger.sensors.contact_f_num = f_num;

    int f_num_force = 0, f_num_torque = 0;
    for (int i = 0; i < 3; i++)
    {
      if (finger_ptr_list[finger_ID]->finger.sensors.wrench_basis[i])
        f_num_force++;
    }
    for (int i = 3; i < 6; i++)
    {
      if (finger_ptr_list[finger_ID]->finger.sensors.wrench_basis[i])
        f_num_torque++;
    }
    finger_ptr_list[finger_ID]->controller.contact_f_num_force = f_num_force;
    finger_ptr_list[finger_ID]->controller.contact_f_num_torque = f_num_torque;

    // tf_SensC part
    // int8_t x = frame->data[2];
    // int8_t y = frame->data[3];
    // int8_t z = frame->data[4];
    // int8_t roll = frame->data[5];
    // int8_t pitch = frame->data[6];
    // int8_t yaw = frame->data[7];
    // finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.param[0] = x * CAN_TF_TRANS_RESOLUTION;                         // x
    // finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.param[1] = y * CAN_TF_TRANS_RESOLUTION;   // y
    // finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.param[2] = z * CAN_TF_TRANS_RESOLUTION;   // z
    // finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.param[3] = roll * CAN_TF_ROT_RESOLUTION;  // roll
    // finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.param[4] = pitch * CAN_TF_ROT_RESOLUTION; // pitch
    // finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.param[5] = yaw * CAN_TF_ROT_RESOLUTION;   // yaw

    int16_t roll = (int16_t)(((uint16_t)frame->data[2]) |
                             ((uint16_t)frame->data[3] << 8));
    int16_t pitch = (int16_t)(((uint16_t)frame->data[4]) |
                              ((uint16_t)frame->data[5] << 8));
    int16_t yaw = (int16_t)(((uint16_t)frame->data[6]) |
                            ((uint16_t)frame->data[7] << 8));

    float param[6] = {};
    param[0] = finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.p[0];
    param[1] = finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.p[1];
    param[2] = finger_ptr_list[finger_ID]->finger.sensors.tf_SensC.p[2];
    param[3] = (float)roll * CAN_TF_ROT_RESOLUTION;
    param[4] = (float)pitch * CAN_TF_ROT_RESOLUTION;
    param[5] = (float)yaw * CAN_TF_ROT_RESOLUTION;
    param2Transform(param, &finger_ptr_list[finger_ID]->finger.sensors.tf_SensC);
  }
  else
    return;
}

void CAN_Parsing_FgrForceCmd(CAN_Frame *frame)
{
  int finger_ID = frame->data[0];
  if (finger_ID >= 0 && finger_ID < FINGER_NUM)
  {

    int16_t fx = (int16_t)((uint16_t)(frame->data[1]) |
                           ((uint16_t)frame->data[2] << 8));
    int16_t fy = (int16_t)((uint16_t)(frame->data[3]) |
                           ((uint16_t)frame->data[4] << 8));
    int16_t fz = (int16_t)((uint16_t)(frame->data[5]) |
                           ((uint16_t)frame->data[6] << 8));

    finger_ptr_list[finger_ID]->controller.f_cmd_buffer[0] = (float)fx * CAN_FORCE_RESOLUTION; // in N
    finger_ptr_list[finger_ID]->controller.f_cmd_buffer[1] = (float)fy * CAN_FORCE_RESOLUTION; // in N
    finger_ptr_list[finger_ID]->controller.f_cmd_buffer[2] = (float)fz * CAN_FORCE_RESOLUTION; // in N
  }
  else
    return;
}

void CAN_Parsing_FgrTorqueCmd(CAN_Frame *frame)
{
  int finger_ID = frame->data[0];
  if (finger_ID >= 0 && finger_ID < FINGER_NUM)
  {

    int16_t Taux = (int16_t)((uint16_t)(frame->data[1]) |
                             ((uint16_t)frame->data[2] << 8));
    int16_t Tauy = (int16_t)((uint16_t)(frame->data[3]) |
                             ((uint16_t)frame->data[4] << 8));
    int16_t Tauz = (int16_t)((uint16_t)(frame->data[5]) |
                             ((uint16_t)frame->data[6] << 8));

    finger_ptr_list[finger_ID]->controller.f_cmd_buffer[3] = (float)Taux * CAN_TORQUE_RESOLUTION; // in N-mm
    finger_ptr_list[finger_ID]->controller.f_cmd_buffer[4] = (float)Tauy * CAN_TORQUE_RESOLUTION; // in N-mm
    finger_ptr_list[finger_ID]->controller.f_cmd_buffer[5] = (float)Tauz * CAN_TORQUE_RESOLUTION; // in N-mm
  }
  else
    return;
}

void CAN_Parsing_FgrWrenchCmdTrig(CAN_Frame *frame)
{
  for (int i = 0; i < FINGER_NUM; i++)
  {
    if ((frame->data[0] & (1 << i)) != 0)
    {
      int f_idx = 0;
      for (int j = 0; j < 6; j++)
      {
        if (finger_ptr_list[i]->finger.sensors.wrench_basis[j] == 1)
        {
          finger_ptr_list[i]->controller.f_cmd_org[f_idx] = finger_ptr_list[i]->controller.f_cmd_buffer[j];
          f_idx++;
        }
      }
    }
  }
}

void CAN_Parsing_GravInPalm(CAN_Frame *frame)
{

  int16_t gx = (int16_t)((uint16_t)(frame->data[0]) |
                         ((uint16_t)frame->data[1] << 8));
  int16_t gy = (int16_t)((uint16_t)(frame->data[2]) |
                         ((uint16_t)frame->data[3] << 8));
  int16_t gz = (int16_t)((uint16_t)(frame->data[4]) |
                         ((uint16_t)frame->data[5] << 8));

  g_palm[0] = (float)gx * CAN_GRAV_RESOLUTION;
  g_palm[1] = (float)gy * CAN_GRAV_RESOLUTION;
  g_palm[2] = (float)gz * CAN_GRAV_RESOLUTION;
}

void CAN_Parsing_SettingStalled(CAN_Frame *frame)
{
  float t = ((uint8_t)frame->data[1]) * CAN_TIME_RESOLUTION;
  float err_q = ((int8_t)frame->data[2]) * CAN_POS_RESOLUTION * 5.0f;
  float vel = ((int8_t)frame->data[3]) * CAN_VEL_RESOLUTION;
  float tau_m = ((int8_t)frame->data[4]) * CAN_TAUM_RESOLUTION;
  for (int i = 0; i < FINGER_NUM; i++)
  {
    if ((frame->data[0] & (1 << i)) != 0)
    {
      finger_ptr_list[i]->controller.stalled_thrld_time = t;
      finger_ptr_list[i]->controller.stalled_thrld_err_q = err_q;
      finger_ptr_list[i]->controller.stalled_thrld_q_dot = vel;
      finger_ptr_list[i]->controller.stalled_thrld_tau_m = tau_m;
    }
  }
}

void CAN_Parsing_SettingFgrStateResp(CAN_Frame *frame)
{
  for (int i = 0; i < FINGER_NUM; i++)
  {
    if ((frame->data[0] & (1 << i)) != 0)
    {
      finger_ptr_list[i]->finger_SR.resp_switch = true;
      finger_ptr_list[i]->finger_SR.counter = 0;
      finger_ptr_list[i]->finger_SR.freq = frame->data[1] * CAN_RESP_FREQ_RESOLUTION;
    }
    else
    {
      finger_ptr_list[i]->finger_SR.resp_switch = false;
    }
  }
}

void CAN_Parsing_SettingJntStateResp(CAN_Frame *frame)
{
  for (int i = 0; i < FINGER_NUM; i++)
  {
    if ((frame->data[0] & (1 << i)) != 0)
    {
      finger_ptr_list[i]->joint_SR.resp_switch = true;
      finger_ptr_list[i]->joint_SR.counter = 0;
      finger_ptr_list[i]->joint_SR.freq = frame->data[1] * CAN_RESP_FREQ_RESOLUTION;
    }
    else
    {
      finger_ptr_list[i]->joint_SR.resp_switch = false;
    }
  }
}

void CAN_Parsing_SettingTfPSensResp(CAN_Frame *frame)
{
  for (int i = 0; i < FINGER_NUM; i++)
  {
    if ((frame->data[0] & (1 << i)) != 0)
    {
      finger_ptr_list[i]->tf_PSens_SR.resp_switch = true;
      finger_ptr_list[i]->tf_PSens_SR.counter = 0;
      finger_ptr_list[i]->tf_PSens_SR.freq = frame->data[1] * CAN_RESP_FREQ_RESOLUTION;
    }
    else
    {
      finger_ptr_list[i]->tf_PSens_SR.resp_switch = false;
    }
  }
}

void CAN_Parsing_ClearWarning(CAN_Frame *frame)
{

  for (int i = 0; i < FINGER_NUM; i++)
  {
    if ((frame->data[0] & (1 << i)) != 0)
    {
      for (int j = 0; j < finger_ptr_list[i]->finger.joint_num; j++)
      {
        finger_ptr_list[i]->controller.warning_code[j] = 0;
        finger_ptr_list[i]->controller.traj_intp[j].traj_progress = 0.0f;
      }
    }
  }
}

void CAN_AddRespMsg_FgrState(void)
{
  for (int i = 0; i < FINGER_NUM; i++)
  {
    if (finger_ptr_list[i]->finger_SR.resp_switch &&
        STATE_COUNTER_PERIOD * finger_ptr_list[i]->finger_SR.counter >= 1000.0f / finger_ptr_list[i]->finger_SR.freq)
    {
      uint8_t data[8] = {};
      data[0] = i;
      data[1] = finger_ptr_list[i]->controller.control_mode;
      data[2] = 1;

      for (int j = 0; j < finger_ptr_list[i]->finger.joint_num; j++)
      {
        if (finger_ptr_list[i]->controller.traj_intp[j].traj_progress < 0.9999f)
        {
          data[2] = 0;
          break;
        }
      }

      int temp;
      float max_stall = finger_ptr_list[i]->controller.stalled_level[0];
      for (int j = 1; j < finger_ptr_list[i]->finger.joint_num; j++)
      {
        max_stall = fmax(finger_ptr_list[i]->controller.stalled_level[j],
                         finger_ptr_list[i]->controller.stalled_level[j - 1]);
      }
      temp = (int)roundf(max_stall / CAN_UNIT_BYTE_RESOLUTION);
      data[3] = (uint8_t)(int8_t)(temp > 127 ? 127 : (temp < -128 ? -128 : temp));

      for (int j = 0; j < finger_ptr_list[i]->finger.joint_num; j++)
      {
        if (finger_ptr_list[i]->controller.warning_code[j])
          data[4] |= (1 << j);
      }
      addCANMsgToQueue(CANID_FINGER_STATE_RESP, 8, data);
      finger_ptr_list[i]->finger_SR.counter = 0;
    }
  }
}

void CAN_AddRespMsg_JntState(void)
{
  for (int i = 0; i < FINGER_NUM; i++)
  {
    if (finger_ptr_list[i]->joint_SR.resp_switch &&
        STATE_COUNTER_PERIOD * finger_ptr_list[i]->joint_SR.counter >= 1000.0f / finger_ptr_list[i]->joint_SR.freq)
    {
      for (int j = 0; j < finger_ptr_list[i]->finger.joint_num; j++)
      {
        int temp;
        uint8_t data[8] = {};
        data[0] = i;
        data[1] = j;

        temp = (int)roundf(finger_ptr_list[i]->controller.traj_intp[j].traj_progress / CAN_UNIT_BYTE_RESOLUTION);
        data[2] = (uint8_t)(int8_t)(temp > 127 ? 127 : (temp < -128 ? -128 : temp));

        int16_t pos = roundf(finger_ptr_list[i]->finger.cur_q[j] / CAN_POS_RESOLUTION);
        data[3] = pos & 0xff;
        data[4] = (pos >> 8) & 0xff;

        temp = (int)roundf(finger_ptr_list[i]->controller.stalled_level[j] / CAN_UNIT_BYTE_RESOLUTION);
        data[5] = (uint8_t)(int8_t)(temp > 127 ? 127 : (temp < -128 ? -128 : temp));

        data[6] = finger_ptr_list[i]->controller.warning_code[j] & 0xff;
        data[7] = (finger_ptr_list[i]->controller.warning_code[j] >> 8) & 0xff;
        addCANMsgToQueue(CANID_JOINT_STATE_RESP, 8, data);
      }
      finger_ptr_list[i]->joint_SR.counter = 0;
    }
  }
}

void CAN_AddRespMsg_TfPSensState(void)
{
  for (int i = 0; i < FINGER_NUM; i++)
  {
    if (finger_ptr_list[i]->tf_PSens_SR.resp_switch &&
        STATE_COUNTER_PERIOD * finger_ptr_list[i]->tf_PSens_SR.counter >= 1000.0f / finger_ptr_list[i]->tf_PSens_SR.freq)
    {
      float param[6] = {};
      transform2Param(&finger_ptr_list[i]->finger.tf_PSens, param);
      // memcpy(&finger_ptr_list[i]->param, param, sizeof(param));

      uint8_t data_tran[8] = {};
      uint8_t data_rot[8] = {};
      data_tran[0] = i;
      data_rot[0] = i;

      for (int j = 0; j < 3; j++)
      {
        int16_t temp = (int)roundf(param[j] / CAN_TF_TRANS_RESOLUTION);
        data_tran[2 * j + 1] = (uint8_t)temp & 0x00ff;
        data_tran[2 * j + 2] = (uint8_t)(temp >> 8) & 0x00ff;
      }

      for (int j = 0; j < 3; j++)
      {
        int16_t temp = (int)roundf(param[j + 3] / CAN_TF_ROT_RESOLUTION);
        data_rot[2 * j + 1] = (uint8_t)temp & 0x00ff;
        data_rot[2 * j + 2] = (uint8_t)(temp >> 8) & 0x00ff;
      }
      addCANMsgToQueue(CANID_TF_PSENS_TRAN_RESP, 8, data_tran);
      addCANMsgToQueue(CANID_TF_PSENS_ROT_RESP, 8, data_rot);
      finger_ptr_list[i]->tf_PSens_SR.counter = 0;
    }
  }
}

// void CAN_Parsing_ErrorCheck(CAN_Frame *frame)
// {
//   CAN_Frame response_frame = {};
//   response_frame.id = CANID_RESP_ERROR_CHECK; // Set the response ID
//   response_frame.dlc = 8;                     // Set the data length
//   response_frame.data[0] = error_code;        // Set a specific byte to indicate successful connection
//   if (!CAN_Queue_Push(&can_tx_queue, &response_frame))
//   {
//     error_code |= ERR_CAN_TX_QUEUE_FULL;
//     Error_Handler();
//   }
// }

void addCANMsgToQueue(const uint32_t id, const uint8_t dlc, const uint8_t *data)
{
  CAN_Frame response_frame = {};
  response_frame.id = id;
  response_frame.dlc = dlc;
  memcpy(response_frame.data, data, dlc);
  if (!CAN_Queue_Push(&can_tx_queue, &response_frame))
  {
    hand_error_code |= ERR_CAN_TX_QUEUE_FULL;
    Error_Handler();
  }
}

// CAN1 receive interrupt
// ---------------------------------------------------------- //
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

  CAN_RxHeaderTypeDef rxHeader;
  // uint8_t rxData[8];
  CAN_Frame frame;

  // get all can_frame messages in the FIFO
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
      // store the received message in a CAN_Frame struct
      frame.id = (rxHeader.IDE == CAN_ID_STD) ? rxHeader.StdId : rxHeader.ExtId;
      frame.dlc = rxHeader.DLC;
      memcpy(frame.data, rxData, 8);

      // push to queue
      if (!CAN_Queue_Push(&can_rx_queue, &frame))
      {
        hand_error_code |= ERR_CAN_RX_QUEUE_FULL;
        Error_Handler();
      }
    }
  }
}

// CAN setting function
// ---------------------------------------------------------- //
void CAN_TxheaderSetup(CAN_TxHeaderTypeDef *Txheader,
                       uint32_t StdId, uint32_t ExtId,
                       uint32_t IDE, uint32_t RTR, uint32_t DLC,
                       FunctionalState TransmitGlobalTime)
{
  Txheader->StdId = StdId;
  Txheader->ExtId = ExtId;
  Txheader->IDE = IDE;
  Txheader->RTR = RTR;
  Txheader->DLC = DLC;
  Txheader->TransmitGlobalTime = TransmitGlobalTime;
}

void CAN1_FilterInit(uint32_t FilterBank, uint32_t FilterMode, uint32_t FilterScale,
                     uint32_t FilterIdHigh, uint32_t FilterIdLow, uint32_t MaskIdHigh, uint32_t MaskIdLow,
                     uint32_t FilterFIFOAssignment, uint32_t FilterActivation)
{
  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = FilterBank;

  sFilterConfig.FilterMode = FilterMode;
  sFilterConfig.FilterScale = FilterScale;
  sFilterConfig.FilterIdHigh = FilterIdHigh;
  sFilterConfig.FilterIdLow = FilterIdLow;
  sFilterConfig.FilterMaskIdHigh = MaskIdHigh;
  sFilterConfig.FilterMaskIdLow = MaskIdLow;
  sFilterConfig.FilterFIFOAssignment = FilterFIFOAssignment;
  sFilterConfig.FilterActivation = FilterActivation;
  // sFilterConfig.SlaveStartFilterBank = 0; // only used for dual CAN
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    // HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
  }
}

// External button interrupt
// ---------------------------------------------------------- //
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)
  {
    exc_btn_isPressed = true;
  }
  else
  {
    __NOP();
  }
}

// Control flag update by timer interrupt
// ---------------------------------------------------------- //
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  // if (htim == &htim6)
  if (htim->Instance == htim6.Instance)
  {
    controlFlag = true;
  }
  else if (htim->Instance == htim7.Instance)
  {
    for (int i = 0; i < FINGER_NUM; i++)
    {
      if (finger_ptr_list[i]->finger_SR.resp_switch)
        finger_ptr_list[i]->finger_SR.counter++;
      if (finger_ptr_list[i]->joint_SR.resp_switch)
        finger_ptr_list[i]->joint_SR.counter++;
      if (finger_ptr_list[i]->tf_PSens_SR.resp_switch)
        finger_ptr_list[i]->tf_PSens_SR.counter++;
    }
  }
}

// Servo communication
// ---------------------------------------------------------- //
uint16_t servoException(ServoXM4340 *servo, int max_rx_err_ctr)
{
  if (servo->RX_err_ctr > max_rx_err_ctr)
  {
    return ERR_SERVO_RX_FAIL;
  }
  if (servo->state == SERVO_SHUTDOWN)
  {
    getServo_Shutdown(servo, 10);
    return ERR_SERVO_SHUTDOWN;
  }
  return 0;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    DXL_SetTxFinished(true);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  if (huart->Instance == USART6)
  {
    UNUSED(huart);
    DXL_SetRxFinished(true);
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0 && !CAN_Queue_IsEmpty(&can_tx_queue))
  {
    // CAN_TxHeaderTypeDef TxHeader;
    // uint32_t TxMailbox;
    // CAN_Frame tx_frame;

    // tx_frame.id = CANID_RESP_ERROR_CHECK; // Set the response ID
    // tx_frame.dlc = 8;                     // Set the data length
    // tx_frame.data[0] = error_code;

    // CAN_TxheaderSetup(&TxHeader, tx_frame.id, 0, CAN_ID_STD, CAN_RTR_DATA, tx_frame.dlc, DISABLE);
    // HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_frame.data, &TxMailbox);
  }

  for (int i = 0; i < TOTAL_SERVO_NUM; i++)
  {
    setServo_TorqueENA(&myServoList[i], TORQUE_DISABLE, 10);
  }

  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    // HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_11);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

    for (volatile int i = 0; i < 1000000; i++)
    {
    }

    // HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_11);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

    for (volatile int i = 0; i < 1000000; i++)
    {
    }
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
