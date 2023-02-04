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

#ifndef FREE_SPACE_H
#define FREE_SPACE_H

#include <free_space_motion_tube/core/basic.h>
#include <free_space_motion_tube/core/motion_primitive.h>

#include <sensor_data_structure/range_scan.h>
#include <sensor_data_structure/range_sensor.h>
#include <building_data_structure/corridor.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

void motion_tube_cartesian_to_sensor_space(const motion_tube_cartesian_t *free_space_template,
    const range_sensor_t *range_sensor,const point2d_t *sensor_pos,
    const maneuver_t *maneuver,
    motion_tube_sensor_space_t *motion_tube_sensor_space);

void sample_move_straight_motion_tube_in_cartesian(const maneuver_t *maneuver,
    const body_t *body, double sampling_interval, 
    motion_tube_cartesian_t *move_straight_template);

void sample_steer_left_motion_tube_in_cartesian(const maneuver_t *maneuver,
    const body_t *body, double sampling_interval, 
    motion_tube_cartesian_t *move_straight_template);

void sample_steer_right_motion_tube_in_cartesian(const maneuver_t *maneuver,
    const body_t *body, double sampling_interval, 
    motion_tube_cartesian_t *move_straight_template);

void sample_free_space_motion_tube_in_cartesian(const maneuver_t *maneuver,
    const body_t *body, double sampling_interval,
    motion_tube_cartesian_t *free_space_template);

// Deprecated
void monitor_motion_tube_availability(const motion_tube_sensor_space_t *free_space_template,
    const range_scan_t *range_scan, const range_sensor_t *range_sensor,
    bool *is_available);

#ifdef __cplusplus
}   // extern C
#endif

#endif