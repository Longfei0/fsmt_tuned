#include <free_space/free_space.h>
#include <math.h>
#include <stdio.h>

void sample_move_straight_template_in_cartesian(
    const sampling_params_t *sampling_params,
    const straight_maneuver_t *maneuver,
    const body_t *body,
    polyline_t *samples)
{
    point2d_t *p_front_left, *p_front_right;
    polyline_t *platform_geometry;
    sampling_params_t primitive_sampling_params;
    polyline_t primitive_samples;

    primitive_sampling_params.sampling_interval = sampling_params->sampling_interval;
    // Left side
    // Configuring
    primitive_sampling_params.max_number_of_samples = sampling_params->max_number_of_samples;
    primitive_samples.points = &samples->points[0];
    // Compute
    p_front_left = &platform_geometry->points[FRONT_LEFT];
    sample_motion_primitive_straight_line(maneuver, &primitive_sampling_params,
        p_front_left, &primitive_samples);
    samples->number_of_points = primitive_samples.number_of_points;
    // Front

    // Right side
    // Configuring
    primitive_sampling_params.max_number_of_samples = sampling_params->max_number_of_samples - 
        samples.number_of_points;
    primitive_samples.points = &samples->points[primitive_samples.number_of_points];
    // Compute
    p_front_left = &platform_geometry->points[FRONT_LEFT];
    sample_motion_primitive_straight_line(maneuver, &primitive_sampling_params,
        p_front_left, samples);


}
/*
void sample_template_in_cartesian(
    const sample_template_in_cartesian_input_t *input,
    sample_template_in_cartesian_output_t *output){

    if (input->angular_rate == 0){
        output->template_in_cartesian->type = MOVE_STRAIGHT;
    } else if (input->angular_rate > 0){
        output->template_in_cartesian->type = STEER_LEFT;
    } else if (input->angular_rate < 0){
        output->template_in_cartesian->type = STEER_RIGHT;
    }

}
*/
