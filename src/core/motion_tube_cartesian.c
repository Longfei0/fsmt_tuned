#include<free_space_motion_tube/core/motion_tube_cartesian.h>

const struct MotionTubeCartesian MotionTubeCartesian ={
    .create = motion_tube_cartesian_create,
    .allocate_memory = motion_tube_cartesian_allocate_memory,
    .deallocate_memory = motion_tube_cartesian_deallocate_memory,
};

void motion_tube_cartesian_create(motion_tube_cartesian_t *motion_tube){
    point2d_array_t *side[3] = {&motion_tube->left, &motion_tube->front, &motion_tube->right};
    for(int i=0; i<3; i++){
        side[i]->number_of_points = 0;
        side[i]->max_number_of_points = 0;
        side[i]->points = NULL;    
    }
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
            } // for
        }
    }
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
}