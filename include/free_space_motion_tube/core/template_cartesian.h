#ifndef FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_CARTESIAN_H
#define FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_CARTESIAN_H

#include<stdlib.h>

#include<free_space_motion_tube/core/basic.h>

#ifdef __cplusplus
extern "C" {
#endif

void template_cartesian_create(template_cartesian_t *template);

void template_cartesian_allocate_memory(template_cartesian_t *template, 
    size_t *max_number_of_samples, uint8_t ALLOCATION_MODE);

void template_cartesian_deallocate_memory(template_cartesian_t *template);

#ifdef __cplusplus
}  // extern C
#endif

#endif
