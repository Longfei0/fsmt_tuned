#ifndef FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_SENSOR_SPACE_H
#define FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_SENSOR_SPACE_H

#include<stdlib.h>

#include<free_space_motion_tube/core/basic.h>

#ifdef __cplusplus
extern "C" {
#endif

void template_sensor_space_create(template_sensor_space_t *template);

void template_sensor_space_allocate_memory(template_sensor_space_t *template, 
    size_t max_number_of_samples);

void template_sensor_space_deallocate_memory(template_sensor_space_t *template);

#ifdef __cplusplus
}  // extern C
#endif

#endif
