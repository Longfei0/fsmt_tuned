/* ----------------------------------------------------------------------------
 * Free space motion tubes
 * ROB @ KU Leuven, Leuven, Belgium
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file template_cartesian.h
 * @date March 15, 2022
 * Authors: Rômulo Rodrigues
 **/

#ifndef FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_CARTESIAN_H
#define FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_CARTESIAN_H

#include<stdlib.h>

#include <free_space_motion_tube/core/basic.h>

#ifdef __cplusplus
extern "C" {
#endif

// Methods are collected in TemplateCartesian
extern const struct TemplateCartesian TemplateCartesian;
struct TemplateCartesian{
    void (*create)(template_cartesian_t*);
    void (*allocate_memory)(template_cartesian_t*, size_t *, uint8_t);
    void (*deallocate_memory)(template_cartesian_t*);
};

void template_cartesian_create(template_cartesian_t *template);

void template_cartesian_allocate_memory(template_cartesian_t *template, 
    size_t *max_number_of_samples, uint8_t ALLOCATION_MODE);

void template_cartesian_deallocate_memory(template_cartesian_t *template);


#ifdef __cplusplus
}  // extern C
#endif

#endif
