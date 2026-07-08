#ifndef MY_FORCE_GEN_H
#define MY_FORCE_GEN_H

#include <stdbool.h>

#define FORCE_RAMP_MAX_DIM 6 // matches f_cmd_org[6] / f_cmd_buffer[6]

// Vector-valued linear (1st-order) ramp for a force/torque command.
//
// Given a start vector fa and a target vector fb (both up to FORCE_RAMP_MAX_DIM
// dimensions), this moves the command vector along the STRAIGHT LINE from fa to
// fb at a single scalar speed (rate), so:
//
//   f_cmd(t) = fa + rate * t * unit_vector(fb - fa)      for t < T
//   f_cmd(t) = fb                                         for t >= T
//
// This guarantees:
//   - f_cmd(t) itself is 1st-order in time (piecewise-linear, constant speed)
//   - every component of the vector reaches its target AT THE SAME INSTANT,
//     because they all share the same scalar progress along the same direction
//   - re-triggering with a new target at any time is safe: since this is
//     evaluated incrementally every cycle from the CURRENT f_cmd, there is no
//     separate "start point" to re-plan.
//
// Caveat: distance/direction here are computed as a plain Euclidean norm over
// whatever dimensions you pass in. That is only physically meaningful if all
// dimensions share the same unit (e.g. 3 force channels in N, or 3 torque
// channels in N-mm). If your wrench_basis mixes force and torque channels in
// the same call, the "single rate" and "straight line" interpretation loses
// its clean physical meaning (see comments in my_force_gen.c). If that's your
// situation, call ForceRamp_Update() once per unit-consistent subset instead
// of once for the whole packed 6-vector.
typedef struct ForceVectorRamp
{
    float f_cmd[FORCE_RAMP_MAX_DIM];   // current output vector
    float cur_vel[FORCE_RAMP_MAX_DIM]; // current velocity vector actually applied last update (0 once settled)
    bool ramping;                      // true while still moving toward target, false once settled
} ForceVectorRamp;

// Hard-reset the ramp to a raw vector value with no interpolation
// (e.g. at startup, or when leaving force-control mode).
void ForceRamp_Reset(ForceVectorRamp *ramp, int dim);

// Call every control cycle with:
//   target   : the desired vector (e.g. the packed f_cmd_buffer values), size >= dim
//   rate     : scalar speed limit for the straight-line motion, in [unit]/s
//   delta_t  : control period, in s
//   dim      : number of valid vector components (<= FORCE_RAMP_MAX_DIM)
// Moves ramp->f_cmd toward target along the straight line joining them, by at
// most rate*delta_t, and writes the result back into ramp->f_cmd.
void ForceRamp_Update(ForceVectorRamp *ramp, const float *target, float rate, float delta_t, int dim);

#endif