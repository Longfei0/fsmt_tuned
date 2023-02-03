#include <free_space_motion_tube/core/motion_primitive.h>
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

    pose2d_t pose;
    for(int i=0; i<number_of_samples; i++){
        if (samples->number_of_points >= samples->max_number_of_points){   
            break;
        }
        // Pose of the unicycle model for control input and time
        excite_unicycle(NULL, control, i*sampling_time, &pose);
        // Position of point of interest for corresponding platform pose
        rigid_body_2d_transformation(&pose, point_of_interest,
            &samples->points[samples->number_of_points]);
        samples->number_of_points += 1;
    }
}

void excite_unicycle(const pose2d_t *pose_init,
    const unicycle_control_t *control, double time, 
    pose2d_t *pose_final)
{
    double x0=0.0;
    double y0=0.0;
    double yaw0=0.0;
    
    if (pose_init!=NULL){
        yaw0 = pose_init->yaw;
        x0 = pose_init->x;
        y0 = pose_init->y;
    }
    pose_final->yaw = control->angular_rate*time + yaw0; 
    double sinz = sin(pose_final->yaw);     
    double cosz = cos(pose_final->yaw);

    if (control->angular_rate == 0){
        pose_final->x = x0*cosz - y0*sinz + control->forward_velocity*time;
        pose_final->y = x0*sinz + y0*cosz + control->forward_velocity*time*sinz;
    }else{
        double radius = control->forward_velocity/control->angular_rate;
        pose_final->x = x0*cosz - y0*sinz + radius*sinz;
        pose_final->y = x0*sinz + y0*cosz + radius*(1-cosz);
    }
}


