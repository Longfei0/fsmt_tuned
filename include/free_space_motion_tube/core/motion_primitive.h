/* ----------------------------------------------------------------------------
 * Free space templates,
 * ROB @ KU Leuven, Leuven, Belgium
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file motion_primitive.h
 * @date March 15, 2022
 * Authors: Rômulo Rodrigues
 **/

#ifndef FREE_SPACE_MOTION_PRIMITIVE_H
#define FREE_SPACE_MOTION_PRIMITIVE_H

#include <free_space_motion_tube/core/basic.h>

#ifdef __cplusplus
extern "C"
{
#endif

void sample_unicycle_motion_primitive(const maneuver_t *maneuver, 
    const point2d_t *offset, double sampling_interval, point2d_array_t *samples);

void excite_unicycle(const pose2d_t *pose_init,
    const unicycle_control_t *control, double time, 
    pose2d_t *pose_final);

#ifdef __cplusplus
} // extern "C"
#endif

#endif