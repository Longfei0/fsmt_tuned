#include<free_space_motion_tube/core/template_cartesian.h>

void template_cartesian_create(template_cartesian_t *template){
    point2d_array_t *side[3] = {&template->left, &template->front, &template->right};
    for(int i=0; i<3; i++){
        side[i]->number_of_points = 0;
        side[i]->max_number_of_points = 0;
        side[i]->points = NULL;    
    }
}

void template_cartesian_allocate_memory(template_cartesian_t *template, 
    size_t *max_number_of_samples, uint8_t ALLOCATION_MODE){
    
    point2d_array_t *side[3] = {&template->left, &template->front, &template->right};

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

void template_cartesian_deallocate_memory(template_cartesian_t *template){   
    point2d_array_t *side[3] = {&template->left, &template->front, &template->right};

    for(int i=0; i<3; i++){
        side[i]->number_of_points = 0;
        side[i]->max_number_of_points = 0;
        if (side[i]->points != NULL){
            free(side[i]->points);
            side[i]->points = NULL;        
        }    
    }
}