#ifndef FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_SENSOR_SPACE_H
#define FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_SENSOR_SPACE_H

#include<stdlib.h>

#include<free_space_motion_tube/core/basic.h>

#ifdef __cplusplus
extern "C" {
#endif

// Methods are collected in TemplateSensorSpace
extern const struct TemplateSensorSpace TemplateSensorSpace;
struct TemplateSensorSpace{
    void (*create)(template_sensor_space_t*);
    void (*allocate_memory)(template_sensor_space_t*, size_t);
    void (*deallocate_memory)(template_sensor_space_t*);
};

void template_sensor_space_create(template_sensor_space_t *template);

void template_sensor_space_allocate_memory(template_sensor_space_t *template, 
    size_t max_number_of_samples);

void template_sensor_space_deallocate_memory(template_sensor_space_t *template);

#ifdef __cplusplus
}  // extern C
#endif

#endif
