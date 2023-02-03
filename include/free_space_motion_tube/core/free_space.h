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

void template_cartesian_to_sensor_space(const template_cartesian_t *free_space_template,
    const range_sensor_t *range_sensor,const point2d_t *sensor_pos,
    const maneuver_t *maneuver,
    template_sensor_space_t *template_sensor_space);

void sample_move_straight_template_in_cartesian(const maneuver_t *maneuver,
    const body_t *body, double sampling_interval, 
    template_cartesian_t *move_straight_template);

void sample_steer_left_template_in_cartesian(const maneuver_t *maneuver,
    const body_t *body, double sampling_interval, 
    template_cartesian_t *move_straight_template);

void sample_steer_right_template_in_cartesian(const maneuver_t *maneuver,
    const body_t *body, double sampling_interval, 
    template_cartesian_t *move_straight_template);

void sample_free_space_template_in_cartesian(const maneuver_t *maneuver,
    const body_t *body, double sampling_interval,
    template_cartesian_t *free_space_template);

void monitor_template_availability(const template_sensor_space_t *free_space_template,
    const range_scan_t *range_scan, const range_sensor_t *range_sensor,
    bool *is_available);

#ifdef __cplusplus
}   // extern C
#endif

#endif