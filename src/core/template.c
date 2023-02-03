#include<free_space_motion_tube/core/template.h>
#include<free_space_motion_tube/core/template_cartesian.h>
#include<free_space_motion_tube/core/template_sensor_space.h>

const struct Template Template ={
    .create = template_create,
    .allocate_memory = template_allocate_memory,
    .deallocate_memory = template_deallocate_memory,
};

void template_create(template_t *template){
    TemplateCartesian.create(&template->cartesian);
    TemplateSensorSpace.create(&template->sensor_space);
}

void template_allocate_memory(template_t *template, 
    size_t *max_number_of_samples, uint8_t ALLOCATION_MODE){
    // Cartesian template
    TemplateCartesian.allocate_memory(&template->cartesian, max_number_of_samples, ALLOCATION_MODE);

    // Sensor space template
    int max_number_of_beams = 0;
    point2d_array_t *side[3] = {&template->cartesian.left, &template->cartesian.front, &template->cartesian.right};
    for(int i=0; i<3; i++){
        if (side[i]->points != NULL && side[i]->max_number_of_points > 0){
            max_number_of_beams += side[i]->max_number_of_points;
        }
    }
    TemplateSensorSpace.allocate_memory(&template->sensor_space, max_number_of_beams);
}

void template_deallocate_memory(template_t *template){
    TemplateCartesian.deallocate_memory(&template->cartesian);
    TemplateSensorSpace.deallocate_memory(&template->sensor_space);
}