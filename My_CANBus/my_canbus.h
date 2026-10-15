#ifndef MY_CANBUS_H
#define MY_CANBUS_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define CAN_FIFO_SIZE 128

typedef struct CAN_Frame
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} CAN_Frame;

typedef struct CAN_Queue
{
    CAN_Frame buffer[CAN_FIFO_SIZE];
    volatile uint32_t head; // written by ISR only
    volatile uint32_t tail; // written by main loop only
} CAN_Queue;

void CAN_Queue_Init(CAN_Queue *can_queue);
bool CAN_Queue_Push(CAN_Queue *can_queue, CAN_Frame *can_frame);
bool CAN_Queue_Pop(CAN_Queue *can_queue, CAN_Frame *can_frame);
bool CAN_Queue_IsEmpty(CAN_Queue *can_queue);
bool CAN_Queue_IsFull(CAN_Queue *can_queue);

#endif // MY_CANBUS_H