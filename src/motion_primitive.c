#include <free_space/motion_primitive.h>
#include <math.h>
#include <stdlib.h>

void sample_unicycle_motion_primitive(const maneuver_t *maneuver,
    const point2d_t *point_of_interest, double sampling_interval, 
    point2d_array_t *samples)
{
    // Variables defined to keep notation compact
    unicycle_control_t *control = (unicycle_control_t *) maneuver->control;
    
    // Variables computed by this function
    double sampling_time;
    int number_of_samples;

    if (control->angular_rate == 0){    
        sampling_time = sampling_interval/fabs(control->forward_velocity);
    }else{
        double radius_at_origin = control->forward_velocity/control->angular_rate;
        double radius_at_point = sqrt(pow(point_of_interest->x,2) + 
            pow(radius_at_origin - point_of_interest->y,2) );
        sampling_time = (sampling_interval/(fabs(control->angular_rate)*radius_at_point));
    }
    // Refine sampling time such that samples at t=0 and t=time_horizon are included
    number_of_samples = (int) ceil(maneuver->time_horizon/sampling_time) + 1; 
    sampling_time = maneuver->time_horizon/(number_of_samples - 1); 

    unicycle_state_t vehicle_state;
    for(int i=0; i<number_of_samples; i++){
        if (samples->number_of_points >= samples->max_number_of_points){   
            break;
        }
        // Pose of the unicycle model for control input and time
        excite_unicycle(NULL, control, i*sampling_time, &vehicle_state);
        // Position of point of interest for corresponding platform pose
        rigid_body_2d_transformation(&vehicle_state.pose, point_of_interest,
            &samples->points[samples->number_of_points]);
        samples->number_of_points += 1;
    }
}

void excite_unicycle(const unicycle_state_t *state_init,
    const unicycle_control_t *control, double time, 
    unicycle_state_t *state_final)
{
    double x0=0.0;
    double y0=0.0;
    double yaw0=0.0;
    
    if (state_init!=NULL){
        yaw0 = state_init->pose.yaw;
        x0 = state_init->pose.x;
        y0 = state_init->pose.y;
    }
    state_final->pose.yaw = control->angular_rate*time + yaw0; 
    double sinz = sin(state_final->pose.yaw);     
    double cosz = cos(state_final->pose.yaw);

    if (control->angular_rate == 0){
        state_final->pose.x = x0*cosz - y0*sinz + control->forward_velocity*time;
        state_final->pose.y = x0*sinz + y0*cosz + control->forward_velocity*time*sinz;
    }else{
        double radius = control->forward_velocity/control->angular_rate;
        state_final->pose.x = x0*cosz - y0*sinz + radius*sinz;
        state_final->pose.y = x0*sinz + y0*cosz + radius*(1-cosz);
    }
}

void rigid_body_2d_transformation(const pose2d_t *pose, const point2d_t *p_reference,
    point2d_t *p_target){
    
    double cosz = cos(pose->yaw);
    double sinz = sin(pose->yaw);
    
    p_target->x = p_reference->x*cosz - p_reference->y*sinz + pose->x;
    p_target->y = p_reference->x*sinz + p_reference->y*cosz + pose->y;  
}

