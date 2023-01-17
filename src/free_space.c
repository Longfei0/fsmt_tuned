#include <free_space/free_space.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

void sample_move_straight_template_in_cartesian(const maneuver_t *maneuver,
    const body_t *body, double sampling_interval, template_t *move_straight_template)
{
    polyline_t *platform_geometry = (polyline_t *) body->geometry;
    point2d_t *p_front_left = &platform_geometry->points[FRONT_LEFT];
    point2d_t *p_front_right = &platform_geometry->points[FRONT_RIGHT];

    // Left side
    sample_unicycle_motion_primitive(maneuver, p_front_left, 
        sampling_interval, &move_straight_template->left);

    // Right side
    sample_unicycle_motion_primitive(maneuver, p_front_right, 
        sampling_interval, &move_straight_template->right);

    // Front
    line_segment2d_t front;
    front.endpoints[0] = move_straight_template->left.points[move_straight_template->left.number_of_points-1];
    front.endpoints[1] = move_straight_template->right.points[move_straight_template->right.number_of_points-1];
    sample_line_segment(&front, sampling_interval, &move_straight_template->front);
}

void sample_steer_left_template_in_cartesian(const maneuver_t *maneuver, 
    const body_t *body, double sampling_interval, template_t *sl_template)
{
    polyline_t *platform_geometry = (polyline_t *) body->geometry;
    point2d_t *p_axle_right = &platform_geometry->points[AXLE_LEFT];
    point2d_t *p_front_left = &platform_geometry->points[FRONT_LEFT];
    point2d_t *p_front_right = &platform_geometry->points[FRONT_RIGHT];
    
    /* Left side of the template */
    sample_unicycle_motion_primitive(maneuver, p_axle_right, 
        sampling_interval, &sl_template->left);

    unicycle_state_t platform_state;
    point2d_t p_front_left_endtime;
    line_segment2d_t left_side;
    left_side.endpoints[0] = sl_template->left.points[sl_template->left.number_of_points-1];
    left_side.endpoints[1] = p_front_left_endtime; 
    excite_unicycle(NULL, (unicycle_control_t *) maneuver->control, 
        maneuver->time_horizon, &platform_state);
    // Position of point of interest for corresponding platform pose
    rigid_body_2d_transformation(&platform_state.pose, p_front_left,
            &p_front_left_endtime);
    sample_line_segment(&left_side, sampling_interval, &sl_template->left);

    /* Right side of the template */
    sample_unicycle_motion_primitive(maneuver, p_front_right, 
        sampling_interval, &sl_template->right);

    /* Front of the template */
    line_segment2d_t front;
    front.endpoints[0] = p_front_left_endtime;
    front.endpoints[1] = sl_template->right.points[sl_template->right.number_of_points-1];
    sample_line_segment(&front, sampling_interval, &sl_template->front);
}

void sample_steer_right_template_in_cartesian(const maneuver_t *maneuver, 
    const body_t *body, double sampling_interval, template_t *sr_template)
{
    polyline_t *platform_geometry = (polyline_t *) body->geometry;
    point2d_t *p_front_left = &platform_geometry->points[FRONT_LEFT];
    point2d_t *p_front_right = &platform_geometry->points[FRONT_RIGHT];
    point2d_t *p_axle_right = &platform_geometry->points[AXLE_RIGHT];
    

    /* Left side of the template */
    sample_unicycle_motion_primitive(maneuver, p_front_left, 
        sampling_interval, &sr_template->left);

    /* Right side of the template */
    sample_unicycle_motion_primitive(maneuver, p_axle_right, 
        sampling_interval, &sr_template->right);

    unicycle_state_t platform_state;
    point2d_t p_front_right_endtime;
    line_segment2d_t right_side;
    right_side.endpoints[0] = sr_template->right.points[sr_template->right.number_of_points-1];
    right_side.endpoints[1] = p_front_right_endtime; 
    excite_unicycle(NULL, (unicycle_control_t *) maneuver->control, 
        maneuver->time_horizon, &platform_state);
    // Position of point of interest for corresponding platform pose
    rigid_body_2d_transformation(&platform_state.pose, p_front_right,
            &p_front_right_endtime);
    sample_line_segment(&right_side, sampling_interval, &sr_template->right);


    /* Front of the template */
    line_segment2d_t front;
    front.endpoints[0] = p_front_right_endtime;
    front.endpoints[1] = sr_template->left.points[sr_template->left.number_of_points-1];
    sample_line_segment(&front, sampling_interval, &sr_template->front);
}

void sample_free_space_template_in_cartesian(const maneuver_t *maneuver,
    const body_t *body, double sampling_interval, template_t *free_space_template)
{
    unicycle_control_t *control = (unicycle_control_t *) maneuver->control;

    if (control->angular_rate == 0){
        sample_move_straight_template_in_cartesian(maneuver, body, 
            sampling_interval, free_space_template);
    } else if (control->angular_rate > 0){
        sample_steer_left_template_in_cartesian(maneuver, body, 
            sampling_interval, free_space_template);
    } else if (control->angular_rate < 0){
        sample_steer_right_template_in_cartesian(maneuver, body, 
            sampling_interval, free_space_template);
    }

}

void template_to_sensor_space_deprecated(const template_t *template_cartesian,
    const range_sensor_t *range_sensor, const point2d_t *sensor_pos,
    const maneuver_t *maneuver, template_sensor_space_t *template_sensor_space)
{
    free_space_beam_t *beams = template_sensor_space->beams;
    int *number_of_beams = &template_sensor_space->number_of_beams;
    int max_number_of_beams = template_sensor_space->max_number_of_beams;
    
    const point2d_array_t *samples[3];
    samples[0] = &template_cartesian->left;
    samples[1] = &template_cartesian->front;
    samples[2] = &template_cartesian->right;

    vector2d_t ray; 

    for (int k=0; k<3; k++){
        for(int i=0; i<samples[k]->number_of_points; i++){
            points_to_vector2d(sensor_pos, &samples[k]->points[i], &ray);
            if ( (ray.direction >= range_sensor->min_angle) && 
                 (ray.direction <= range_sensor->max_angle) &&
                 (ray.magnitude > range_sensor->min_distance) &&
                 (ray.magnitude < range_sensor->max_distance))
            {
                beams[*number_of_beams].type = FREE_SPACE;
                beams[*number_of_beams].range_outer = ray.magnitude;
                beams[*number_of_beams].angle = ray.direction;
                beams[*number_of_beams].index = round((ray.direction - 
                        range_sensor->min_angle)/range_sensor->angular_resolution);
                *number_of_beams += 1;
            }
        }
    }  
    if (maneuver->kinematic_model == UNICYCLE){
        template_sensor_space->maneuver.control = (void *) malloc(sizeof(unicycle_control_t));
        ((unicycle_control_t *) template_sensor_space->maneuver.control)->forward_velocity = 
            ((unicycle_control_t *) maneuver->control)->forward_velocity;
        ((unicycle_control_t *) template_sensor_space->maneuver.control)->angular_rate = 
            ((unicycle_control_t *) maneuver->control)->angular_rate;
        
    }

    template_sensor_space->maneuver.kinematic_model = maneuver->kinematic_model;
    template_sensor_space->maneuver.time_horizon = maneuver->time_horizon;
}

void monitor_template_availability(const template_sensor_space_t *free_space_template,
    const range_scan_t *range_scan, const range_sensor_t *range_sensor,
    bool *is_available){

    double measurement;
    *is_available = true;
    for(int i=0; i<free_space_template->number_of_beams; i++){
        measurement = range_scan->measurements[free_space_template->beams[i].index];
        if(measurement > range_sensor->min_distance && measurement < range_sensor->max_distance){
            if (measurement < free_space_template->beams[i].range_outer){
                *is_available = false;
                break;
            }
        }
    }
}

