#include "my_canbus.h"

void CAN_Queue_Init(CAN_Queue *can_queue)
{
    memset(can_queue, 0, sizeof(CAN_Queue));
}

bool CAN_Queue_Push(CAN_Queue *can_queue, CAN_Frame *can_frame)
{
    uint32_t next_head = (can_queue->head + 1) % CAN_FIFO_SIZE;
    if (next_head == can_queue->tail) // full
    {
        return false;
    }
    can_queue->buffer[can_queue->head] = *can_frame;
    can_queue->head = next_head;
    return true;
}

bool CAN_Queue_Pop(CAN_Queue *can_queue, CAN_Frame *can_frame)
{
    if (can_queue->head == can_queue->tail) // empty
    {
        return false;
    }
    *can_frame = can_queue->buffer[can_queue->tail];
    can_queue->tail = (can_queue->tail + 1) % CAN_FIFO_SIZE;
    return true;
}

bool CAN_Queue_IsEmpty(CAN_Queue *can_queue)
{
    return can_queue->head == can_queue->tail;
}

bool CAN_Queue_IsFull(CAN_Queue *can_queue)
{
    return ((can_queue->head + 1) % CAN_FIFO_SIZE) == can_queue->tail;
}
