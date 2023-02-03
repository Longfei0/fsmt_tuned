#ifndef FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_H
#define FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_H

#include<stdlib.h>

#include<free_space_motion_tube/core/basic.h>

#ifdef __cplusplus
extern "C" {
#endif

void template_create(template_t *template);

void template_allocate_memory(template_t *template, 
    size_t *max_number_of_samples, uint8_t ALLOCATION_MODE);

void template_deallocate_memory(template_t *template);

#ifdef __cplusplus
}  // extern C
#endif

#endif
