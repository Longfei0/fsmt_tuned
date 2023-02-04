#include<free_space_motion_tube/core/motion_tube_cartesian.h>

const struct MotionTubeCartesian MotionTubeCartesian ={
    .create = motion_tube_cartesian_create,
    .allocate_memory = motion_tube_cartesian_allocate_memory,
    .deallocate_memory = motion_tube_cartesian_deallocate_memory,
    .sample = motion_tube_cartesian_sample
};

void motion_tube_cartesian_create(motion_tube_cartesian_t *motion_tube){
    point2d_array_t *side[3] = {&motion_tube->left, &motion_tube->front, &motion_tube->right};
    for(int i=0; i<3; i++){
        side[i]->number_of_points = 0;
        side[i]->max_number_of_points = 0;
        side[i]->points = NULL;    
    }
    motion_tube->footprint.points = NULL;
    motion_tube->footprint.max_number_of_points = 0;
    motion_tube->footprint.number_of_points = 0;
}

void motion_tube_cartesian_allocate_memory(motion_tube_cartesian_t *motion_tube, 
    size_t *max_number_of_samples, uint8_t ALLOCATION_MODE){
    
    point2d_array_t *side[3] = {&motion_tube->left, &motion_tube->front, &motion_tube->right};

    if (ALLOCATION_MODE == 1){
        if (max_number_of_samples[0] > 0){
            for(int i=0; i<3; i++){ 
                side[i]->number_of_points = 0;
                side[i]->max_number_of_points = max_number_of_samples[0];    
                side[i]->points = (point2d_t *) malloc(max_number_of_samples[0] * sizeof(point2d_t));  
                if (side[i]->points != NULL){
                    side[i]->max_number_of_points = max_number_of_samples[0];
                }else{
                    // Failed to allocate memory
                    side[i]->max_number_of_points = 0;
                }      
            } 
        }
    }

    motion_tube->footprint.points = (point2d_t *) malloc(4* sizeof(point2d_t));
    motion_tube->footprint.max_number_of_points = 4;
    motion_tube->footprint.number_of_points = 0;    
}

void motion_tube_cartesian_deallocate_memory(motion_tube_cartesian_t *motion_tube){
    point2d_array_t *side[3] = {&motion_tube->left, &motion_tube->front, &motion_tube->right};

    for(int i=0; i<3; i++){
        side[i]->number_of_points = 0;
        side[i]->max_number_of_points = 0;
        if (side[i]->points != NULL){
            free(side[i]->points);
            side[i]->points = NULL;        
        }    
    }
    motion_tube->footprint.max_number_of_points = 0;
    motion_tube->footprint.number_of_points = 0;    
    if (motion_tube->footprint.points != NULL) {
        free(motion_tube->footprint.points);
        motion_tube->footprint.points = NULL;
    }
}

void motion_tube_cartesian_sample(motion_tube_cartesian_t *motion_tube,
    const motion_primitive_t *motion_primitive)
{
    if (motion_primitive->model == UNICYCLE){
        unicycle_control_t *control = (unicycle_control_t *) motion_primitive->control;
        if (control->angular_rate == 0){
            motion_tube_cartesian_sample_move_straight(motion_tube,  
                motion_primitive, &MotionPrimitiveUnicycle);
        // } else if (control->angular_rate > 0){
        //     sample_steer_left_motion_tube_in_cartesian(maneuver, body, 
        //         sampling_interval, free_space_template);
        // } else if (control->angular_rate < 0){
        //     sample_steer_right_motion_tube_in_cartesian(maneuver, body, 
        //         sampling_interval, free_space_template);
        // }
        }
    }
}

void motion_tube_cartesian_sample_move_straight(motion_tube_cartesian_t *motion_tube,
    const motion_primitive_t *motion_primitive, const struct MotionPrimitive *MotionPrimitive)
{
    polyline_t *footprint = &motion_tube->footprint;
    point2d_t *p_front_left = &footprint->points[FRONT_LEFT];
    point2d_t *p_front_right = &footprint->points[FRONT_RIGHT];

    // Left side
    MotionPrimitive->sample(motion_primitive, p_front_left, 
        motion_tube->sampling_interval, &motion_tube->left );
    // Right side
    MotionPrimitive->sample(motion_primitive, p_front_right, 
        motion_tube->sampling_interval, &motion_tube->right );
    // Front
    line_segment2d_t front;
    front.endpoints[0] = motion_tube->left.points[motion_tube->left.number_of_points-1];
    front.endpoints[1] = motion_tube->right.points[motion_tube->right.number_of_points-1];
    sample_line_segment(&front, motion_tube->sampling_interval, &motion_tube->front);
}
