#include "my_trajectory_gen.h"
#include <string.h>

bool firstSectionUpdate(TrajectoryInterpolator *traj_intp, const TrajectoryPoint *current_point)
{
    TrajectoryPoint first_point = {};
    bool pop_result = TrajectoryQueue_Pop(&traj_intp->queue, &first_point);
    if (!pop_result || first_point.time <= 0.0f)
    {
        // No first trajectory point or invalid first trajectory point
        traj_intp->queue.count = 0;
        traj_intp->queue.head = 0;
        traj_intp->queue.tail = 0;
        traj_intp->traj_progress = 0.0f;
        return false;
    }
    else
    {
        traj_intp->start_point = *current_point;
        traj_intp->end_point = first_point;
        traj_intp->cur_time = 0.0f;
        traj_intp->traj_progress = 0.0f;
        return true;
    }
}

void jointCmdGeneration(TrajectoryInterpolator *traj_intp, const float delta_t)
{
    // ===== case 1: Trajectory enable =====
    if (traj_intp->queue_is_enabled)
    {
        traj_intp->cur_time += delta_t;
        traj_intp->traj_progress = traj_intp->cur_time / traj_intp->final_time;
        if (traj_intp->traj_progress > 1.0f)
            traj_intp->traj_progress = 1.0f;

        // ===== 尚未到 end_time =====
        if (traj_intp->cur_time < traj_intp->end_point.time)
        {
            Cubic_Interpolation(
                traj_intp->start_point,
                traj_intp->end_point,
                traj_intp->cur_time,
                &traj_intp->q_cmd,
                &traj_intp->q_dot_cmd,
                &traj_intp->q_dot2_cmd);
            // Quintic_Interpolation(traj_intp->start_point,
            //                       traj_intp->end_point,
            //                       traj_intp->cur_time,
            //                       &traj_intp->q_cmd,
            //                       &traj_intp->q_dot_cmd,
            //                       &traj_intp->q_dot2_cmd);

            return;
        }

        // ===== 到達 end_time =====
        TrajectoryPoint next_point;

        if (TrajectoryQueue_Pop(&traj_intp->queue, &next_point) && next_point.time > traj_intp->end_point.time)
        {
            // 有下一段 → 更新區間
            traj_intp->start_point = traj_intp->end_point;
            traj_intp->end_point = next_point;
            Cubic_Interpolation(
                traj_intp->start_point,
                traj_intp->end_point,
                traj_intp->cur_time,
                &traj_intp->q_cmd,
                &traj_intp->q_dot_cmd,
                &traj_intp->q_dot2_cmd);
            // Quintic_Interpolation(traj_intp->start_point,
            //                       traj_intp->end_point,
            //                       traj_intp->cur_time,
            //                       &traj_intp->q_cmd,
            //                       &traj_intp->q_dot_cmd,
            //                       &traj_intp->q_dot2_cmd);
        }
        else
        {
            // ===== queue 空或不合法點 → 結束 =====
            traj_intp->queue_is_enabled = false;

            // 直接清空queue
            traj_intp->queue.head = 0;
            traj_intp->queue.tail = 0;
            traj_intp->queue.count = 0;

            traj_intp->q_cmd = traj_intp->end_point.pos;
            traj_intp->q_dot_cmd = 0.0f;
            traj_intp->q_dot2_cmd = 0.0f;

            traj_intp->cur_time = traj_intp->end_point.time;
        }

        return;
    }
    else
    {
        // ===== case 2: Trajectory disabled =====
        // if (traj_intp->cur_time < traj_intp->end_point.time)
        // {
        //     // 還沒到終點 → 強制追到 end_pos
        //     traj_intp->end_point.pos = traj_intp->q_cmd;
        //     traj_intp->end_point.vel = 0.0f;
        // }

        traj_intp->q_cmd = traj_intp->end_point.pos;
        traj_intp->q_dot_cmd = 0.0f;
        traj_intp->q_dot2_cmd = 0.0f;
        traj_intp->cur_time = traj_intp->end_point.time;
        return;
    }
}

void Cubic_Interpolation(const TrajectoryPoint start,
                         const TrajectoryPoint end,
                         const float cur_time,
                         float *pos,
                         float *vel,
                         float *acc)
{
    float T = end.time - start.time;
    float t = cur_time - start.time;

    if (t <= 0.0f)
    {
        *pos = start.pos;
        *vel = start.vel;
        *acc = 0.0f;
        return;
    }

    if (t >= T)
    {
        *pos = end.pos;
        *vel = end.vel;
        *acc = 0.0f;
        return;
    }

    float T2 = T * T;
    float T3 = T2 * T;

    // 係數
    float a0 = start.pos;
    float a1 = start.vel;
    float a2 = (3.0f * (end.pos - start.pos) - (2.0f * start.vel + end.vel) * T) / T2;
    float a3 = (2.0f * (start.pos - end.pos) + (start.vel + end.vel) * T) / T3;

    // 計算
    float t2 = t * t;
    float t3 = t2 * t;

    *pos = a0 + a1 * t + a2 * t2 + a3 * t3;
    *vel = a1 + 2.0f * a2 * t + 3.0f * a3 * t2;
    *acc = 2.0f * a2 + 6.0f * a3 * t;

    return;
}

void Quintic_Interpolation(const TrajectoryPoint start,
                           const TrajectoryPoint end,
                           const float cur_time,
                           float *pos,
                           float *vel,
                           float *acc)
{
    float T = end.time - start.time;
    float t = cur_time - start.time;

    if (T <= 0.0f)
    {
        *pos = end.pos;
        *vel = end.vel;
        *acc = 0.0f;
        return;
    }

    if (t <= 0.0f)
    {
        *pos = start.pos;
        *vel = start.vel;
        *acc = 0.0f; // 邊界加速度假設為 0
        return;
    }

    if (t >= T)
    {
        *pos = end.pos;
        *vel = end.vel;
        *acc = 0.0f; // 邊界加速度假設為 0
        return;
    }

    float T2 = T * T;
    float T3 = T2 * T;
    float T4 = T3 * T;
    float T5 = T4 * T;

    float dq = end.pos - start.pos;

    // 係數（邊界加速度為 0，故 a2 = 0）
    float a0 = start.pos;
    float a1 = start.vel;
    float a3 = (10.0f * dq - (6.0f * start.vel + 4.0f * end.vel) * T) / T3;
    float a4 = (-15.0f * dq + (8.0f * start.vel + 7.0f * end.vel) * T) / T4;
    float a5 = (6.0f * dq - 3.0f * (start.vel + end.vel) * T) / T5;

    float t2 = t * t;
    float t3 = t2 * t;
    float t4 = t3 * t;
    float t5 = t4 * t;

    *pos = a0 + a1 * t + a3 * t3 + a4 * t4 + a5 * t5;
    *vel = a1 + 3.0f * a3 * t2 + 4.0f * a4 * t3 + 5.0f * a5 * t4;
    *acc = 6.0f * a3 * t + 12.0f * a4 * t2 + 20.0f * a5 * t3;
}

/*
 *
 *
 */
// Trajectory FIFO buffer functions
/********************************************************************************** */
void TrajectoryQueue_Init(TrajectoryQueue *traj_queue)
{
    memset(traj_queue, 0, sizeof(TrajectoryQueue));
}

bool TrajectoryQueue_Push(TrajectoryQueue *traj_queue, const TrajectoryPoint *traj_point)
{
    if (TrajectoryQueue_IsFull(traj_queue))
    {
        return false;
    }

    traj_queue->buffer[traj_queue->head] = *traj_point;
    traj_queue->head = (traj_queue->head + 1) % TRAJ_FIFO_SIZE;
    traj_queue->count++;

    return true;
}

bool TrajectoryQueue_Pop(TrajectoryQueue *traj_queue, TrajectoryPoint *traj_point)
{
    if (TrajectoryQueue_IsEmpty(traj_queue))
    {
        return false;
    }

    *traj_point = traj_queue->buffer[traj_queue->tail];
    traj_queue->tail = (traj_queue->tail + 1) % TRAJ_FIFO_SIZE;
    traj_queue->count--;

    return true;
}

bool TrajectoryQueue_IsEmpty(TrajectoryQueue *traj_queue)
{
    return traj_queue->count == 0;
}

bool TrajectoryQueue_IsFull(TrajectoryQueue *traj_queue)
{
    return traj_queue->count == TRAJ_FIFO_SIZE;
}