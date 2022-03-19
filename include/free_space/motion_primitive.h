/* ----------------------------------------------------------------------------
 * Free space templates,
 * ROB @ KU Leuven, Leuven, Belgium
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file free_space.h
 * @date March 15, 2022
 * Authors: Rômulo Rodrigues
 **/

#ifndef MOTION_PRIMITIVE_H
#define MOTION_PRIMITIVE_H

#include <free_space/basic.h>

/*** MANEUVERS ***/
typedef struct arc_maneuver_s{
    double angular_rate;    // measured in the center of mass [rad/s]
    double forward_velocity;    // measured in the the center of mass [m/s] 
    double time_horizon;    // temporal length of the motion [s]
}arc_maneuver_t;

typedef struct straight_maneuver_s{
    double forward_velocity;    // measured in the the center of mass [m/s] 
    double time_horizon;    // temporal length of the motion [s]
}straight_maneuver_t;


/**
 * Circular arc motion primitive
void sample_motion_primitive_circular_arc(
    const arc_maneuver_t *maneuver,
    const point2d_t *offset,
    double sampling_interval,
    int max_number_of_samples,
    point2d_t *samples,
    int *number_of_samples
);

 * */
void sample_motion_primitive_circular_arc(
    const arc_maneuver_t *maneuver,
    const sampling_params_t *sampling_params,
    const point2d_t *offset,
    polyline_t *samples
);

/**
 * Straight line motion primitive
 * */
void sample_motion_primitive_straight_line(
    const straight_maneuver_t *maneuver,
    const sampling_params_t *sampling_params,
    const point2d_t *offset,
    polyline_t *samples
);

#endif