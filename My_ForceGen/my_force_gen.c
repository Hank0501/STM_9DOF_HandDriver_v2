#include "my_force_gen.h"
#include <math.h>

// Below this distance, treat fa and fb as identical. Prevents 0/0 when
// normalizing (target - f_cmd) into a unit vector, and prevents command
// chatter from floating point noise sitting right at the target.
#define FORCE_RAMP_EPS 1e-6f

void ForceRamp_Reset(ForceVectorRamp *ramp, int dim)
{
    for (int i = 0; i < dim; i++)
    {
        ramp->f_cmd[i] = 0.0f;
        ramp->cur_vel[i] = 0.0f;
    }
    ramp->ramping = false;
}

void ForceRamp_Update(ForceVectorRamp *ramp, const float *target, float rate, float delta_t, int dim)
{
    float diff[FORCE_RAMP_MAX_DIM];
    float distance_sq = 0.0f;

    // (1). diff = target - f_cmd, and the remaining Euclidean distance to go
    // ------------------------------------------------ //
    for (int i = 0; i < dim; i++)
    {
        diff[i] = target[i] - ramp->f_cmd[i];
        distance_sq += diff[i] * diff[i];
    }
    float distance = sqrtf(distance_sq);

    // (2). Guard: already at target (or rate/dt mis-configured) -> snap, no ramp
    // ------------------------------------------------ //
    if (distance < FORCE_RAMP_EPS || rate <= 0.0f || delta_t <= 0.0f)
    {
        for (int i = 0; i < dim; i++)
        {
            ramp->f_cmd[i] = target[i];
            ramp->cur_vel[i] = 0.0f;
        }
        ramp->ramping = false;
        return;
    }

    float step = rate * delta_t; // how far we're allowed to travel this cycle

    // (3). Clamp: if this step would reach/overshoot the target, snap exactly
    //      to it instead of overshooting and oscillating around it.
    //      Because this uses ONE scalar "step" and ONE "distance" for the
    //      whole vector, every component snaps to its target on the exact
    //      same call -> guarantees simultaneous arrival.
    // ------------------------------------------------ //
    if (step >= distance)
    {
        for (int i = 0; i < dim; i++)
        {
            ramp->f_cmd[i] = target[i];
            ramp->cur_vel[i] = 0.0f;
        }
        ramp->ramping = false;
        return;
    }

    // (4). Still ramping: move by `step` along unit_vector(diff)
    // ------------------------------------------------ //
    float inv_distance = 1.0f / distance;
    for (int i = 0; i < dim; i++)
    {
        float unit_i = diff[i] * inv_distance;
        ramp->f_cmd[i] += unit_i * step;
        ramp->cur_vel[i] = unit_i * rate;
    }
    ramp->ramping = true;
}