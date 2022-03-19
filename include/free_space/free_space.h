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

#include <free_space/basic.h>
#include <free_space/motion_primitive.h>

enum free_space_sample {FREE_SPACE, FREE_SPACE_AND_OCCULSION}; 
enum free_space_template {MOVE_STRAIGHT, STEER_LEFT, STEER_RIGHT}; 

/*
typedef struct free_space_sample_in_sensor_space_s{
    enum free_space_sample type;
    int index;
    double range_inner;
    double range_intermediate;
    double range_outer;
}free_space_sample_in_sensor_space_t;

typedef struct free_space_template_in_sensor_space_s{
    int nb_samples;
    free_space_sample_in_sensor_space_t *samples;
}free_space_template_in_sensor_space_t;

typedef struct free_space_template_in_cartesian_s{
    enum free_space_template type;
    int nb_samples;
    point2d_t *samples;
}free_space_template_in_cartesian_t;
*/

/* cartesian_samples_to_sensor_space() */
/*
typedef struct cartesian_samples_to_sensor_space_input_s{
    free_space_template_in_cartesian_t *template_in_cartesian;
    range_sensor_t *range_sensor;
    pose2d_t *sensor_pose;
}cartesian_samples_to_sensor_space_input_t; 

typedef struct cartesian_samples_to_sensor_space_output_s{
    free_space_template_in_sensor_space_t *template_in_sensor_space;
}cartesian_samples_to_sensor_space_output_t; 

void cartesian_samples_to_sensor_space(
    cartesian_samples_to_sensor_space_input_t *input,
    cartesian_samples_to_sensor_space_output_t *output);

*/
/* cartesian_samples_to_sensor_space() */
/*
typedef struct sample_template_in_cartesian_input_s{
    double angular_rate;
    double forward_speed;
    double time_horizon;
    double sampling_interval;
    double augment_distance;
    point2d_t p_front_left, p_front_right, p_axle_left, p_axle_right;
}sample_template_in_cartesian_input_t;

typedef struct sample_template_in_cartesian_output_s{
    free_space_template_in_cartesian_t *template_in_cartesian;
}sample_template_in_cartesian_output_t;
*/
/*
void sample_template_in_cartesian(
    const sample_template_in_cartesian_input_t *input,
    sample_template_in_cartesian_output_t *output);

*/

void sample_move_straight_template_in_cartesian(
    const sampling_params_t *sampling_params,
    const straight_maneuver_t *maneuver,
    const body_t *body,
    polyline_t *samples);

#endif