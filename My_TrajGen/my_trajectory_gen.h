#include <stdint.h>
#include <stdbool.h>

#define TRAJ_FIFO_SIZE 128

typedef struct TrajectoryPoint
{
    float pos;  // in rad
    float vel;  // in rad/s
    float time; // in s
} TrajectoryPoint;

typedef struct TrajectoryQueue
{
    TrajectoryPoint buffer[TRAJ_FIFO_SIZE];
    uint32_t head;
    uint32_t tail;
    uint32_t count;
} TrajectoryQueue;

typedef struct TrajectoryInterpolator
{
    TrajectoryQueue queue;
    TrajectoryPoint start_point;
    TrajectoryPoint end_point;
    float cur_time;        // in s, time since the start of the trajectory control
    float final_time;      // the time command of the last point in trajectory queue
    float traj_progress;   // the ratio of cur_time and final_time, 0 ~ 1
    bool queue_is_enabled; // whether the interpolator generate command based on trajectory queue

    // Output commands
    float q_cmd;      // in rad
    float q_dot_cmd;  // in rad/s
    float q_dot2_cmd; // in rad/s^2

} TrajectoryInterpolator;

bool firstSectionUpdate(TrajectoryInterpolator *traj_intp, const TrajectoryPoint *current_point);

void jointCmdGeneration(TrajectoryInterpolator *traj_intp, const float delta_t);

void Cubic_Interpolation(const TrajectoryPoint start,
                         const TrajectoryPoint end,
                         const float cur_time,
                         float *pos,
                         float *vel,
                         float *acc);
void Quintic_Interpolation(const TrajectoryPoint start,
                           const TrajectoryPoint end,
                           const float cur_time,
                           float *pos,
                           float *vel,
                           float *acc);
void TrajectoryQueue_Init(TrajectoryQueue *traj_queue);
bool TrajectoryQueue_Push(TrajectoryQueue *traj_queue, const TrajectoryPoint *traj_point);
bool TrajectoryQueue_Pop(TrajectoryQueue *traj_queue, TrajectoryPoint *traj_point);
bool TrajectoryQueue_IsEmpty(TrajectoryQueue *traj_queue);
bool TrajectoryQueue_IsFull(TrajectoryQueue *traj_queue);