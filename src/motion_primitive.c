#include <free_space/motion_primitive.h>
#include <math.h>

void sample_motion_primitive_circular_arc(
    const arc_maneuver_t *maneuver,
    const sampling_params_t *sampling_params,
    const point2d_t *offset,
    polyline_t *samples)
{
    // Computed by the function
    double x0, y0;
    double radius_at_point;
    double radius_at_origin;
    double sampling_time;
    int number_of_samples;
    double orientation, cosz, sinz;

    x0 = offset->x;
    y0 = offset->y;    
    // Find sampling time given parameters of the circular arc trajectory
    radius_at_origin = maneuver->forward_velocity/maneuver->angular_rate;
    radius_at_point = sqrt(pow(x0,2) + pow(radius_at_origin - y0,2) );
    sampling_time = (sampling_params->sampling_interval/(fabs(maneuver->angular_rate)*radius_at_point));
    // Refine sampling time such that samples corresponding to t=0 and
    // t=sampling_time are included
    number_of_samples = (int) ceil(maneuver->time_horizon/sampling_time) + 1; 
    sampling_time = maneuver->time_horizon/(number_of_samples - 1); 

    samples->number_of_points = 0;
    for(int i=0; i<number_of_samples; i++){
        if (i >= sampling_params->max_number_of_samples){   
            break;
        }
        orientation = i*sampling_time*maneuver->angular_rate;
        cosz = cos(orientation);
        sinz = sin(orientation);        

        samples->points[i].x = x0*cosz - y0*sinz + radius_at_origin*sinz;
        samples->points[i].y = x0*sinz + y0*cosz + radius_at_origin*(1-cosz);
        samples->number_of_points += 1;
    }
}

void sample_motion_primitive_straight_line(
    const straight_maneuver_t *maneuver,
    const sampling_params_t *sampling_params,
    const point2d_t *offset,
    polyline_t *samples)
{
    // Computed by the function
    double x0, y0;
    double sampling_time;
    double cosz, sinz, orientation;
    int number_of_samples;

    x0 = offset->x;
    y0 = offset->y;    
    // Find sampling time given parameters of the straigh line trajectory
    sampling_time = sampling_params->sampling_interval/fabs(maneuver->forward_velocity);
    // Refine sampling time such that samples corresponding to t=0 and
    // t=sampling_time are included
    number_of_samples = (int) ceil(maneuver->time_horizon/sampling_time) + 1; 
    sampling_time = maneuver->time_horizon/(number_of_samples - 1); 

    samples->number_of_points = 0;
    orientation = 0.0;
    cosz = cos(orientation);
    sinz = sin(orientation);        
    for(int i=0; i<number_of_samples; i++){
        if (i >= sampling_params->max_number_of_samples){   
            break;
        }
        samples->points[i].x = x0*cosz - y0*sinz + maneuver->forward_velocity*i*sampling_time;
        samples->points[i].y = x0*sinz + y0*cosz + maneuver->forward_velocity*i*sampling_time*sinz;
        samples->number_of_points += 1;
    }
}