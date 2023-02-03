#include<free_space_motion_tube/core/template_sensor_space.h>

const struct TemplateSensorSpace TemplateSensorSpace ={
    .create = template_sensor_space_create,
    .allocate_memory = template_sensor_space_allocate_memory,
    .deallocate_memory = template_sensor_space_deallocate_memory,
};

void template_sensor_space_create(template_sensor_space_t *template){
    template->beams = NULL;
    template->number_of_beams = 0;
    template->max_number_of_beams = 0;
}

void template_sensor_space_allocate_memory(template_sensor_space_t *template,
    size_t max_number_of_beams){
    template->number_of_beams = 0;

    if(max_number_of_beams > 0){
        template->beams = malloc(sizeof(template->beams)*max_number_of_beams);
        if (template->beams == NULL){
            template->max_number_of_beams = max_number_of_beams;
        }else{
            // Failed to allocate memory
            template->max_number_of_beams = 0;
        }
    }

}

void template_sensor_space_deallocate_memory(template_sensor_space_t *template){   
    if (template->beams != NULL){
        free(template->beams);
        template->beams = NULL; 
    }    
}