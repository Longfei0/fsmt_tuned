/* ----------------------------------------------------------------------------
 * Free space motion tubes
 * ROB @ KU Leuven, Leuven, Belgium
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file motion_tube_cartesian.h
 * @date March 15, 2022
 * Authors: Rômulo Rodrigues
 **/

#ifndef FREE_SPACE_MOTION_TUBE_CORE_motion_tube_cartesian_H
#define FREE_SPACE_MOTION_TUBE_CORE_motion_tube_cartesian_H

#include<stdlib.h>

#include <free_space_motion_tube/core/basic.h>

#ifdef __cplusplus
extern "C" {
#endif

// Methods are collected in MotionTubeCartesian
extern const struct MotionTubeCartesian MotionTubeCartesian;
struct MotionTubeCartesian{
    void (*create)(motion_tube_cartesian_t*);
    void (*allocate_memory)(motion_tube_cartesian_t*, size_t *, uint8_t);
    void (*deallocate_memory)(motion_tube_cartesian_t*);
};

void motion_tube_cartesian_create(motion_tube_cartesian_t *motion_tube);

void motion_tube_cartesian_allocate_memory(motion_tube_cartesian_t *motion_tube, 
    size_t *max_number_of_samples, uint8_t ALLOCATION_MODE);

void motion_tube_cartesian_deallocate_memory(motion_tube_cartesian_t *motion_tube);


#ifdef __cplusplus
}  // extern C
#endif

#endif
