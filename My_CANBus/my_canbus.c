#include "my_canbus.h"

void CAN_Queue_Init(CAN_Queue *can_queue)
{
    memset(can_queue, 0, sizeof(CAN_Queue));
}

bool CAN_Queue_Push(CAN_Queue *can_queue, CAN_Frame *can_frame)
{
    if (CAN_Queue_IsFull(can_queue))
    {
        return false;
    }
    can_queue->buffer[can_queue->head] = *can_frame;
    can_queue->head = (can_queue->head + 1) % CAN_FIFO_SIZE;
    can_queue->count++;
    return true;
}

bool CAN_Queue_Pop(CAN_Queue *can_queue, CAN_Frame *can_frame)
{
    if (CAN_Queue_IsEmpty(can_queue))
    {
        return false;
    }
    *can_frame = can_queue->buffer[can_queue->tail];
    can_queue->tail = (can_queue->tail + 1) % CAN_FIFO_SIZE;
    can_queue->count--;
    return true;
}

bool CAN_Queue_IsEmpty(CAN_Queue *can_queue)
{
    return can_queue->count == 0;
}

bool CAN_Queue_IsFull(CAN_Queue *can_queue)
{
    return can_queue->count == CAN_FIFO_SIZE;
}
