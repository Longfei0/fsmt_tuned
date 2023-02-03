#ifndef FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_H
#define FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_H

#include<stdlib.h>

#include<free_space_motion_tube/core/basic.h>

#ifdef __cplusplus
extern "C" {
#endif

// Methods are collected in TemplateCartesian
extern const struct Template Template;
struct Template{
    void (*create)(template_t*);
    void (*allocate_memory)(template_t*, size_t *, uint8_t);
    void (*deallocate_memory)(template_t*);
};

void template_create(template_t *template);

void template_allocate_memory(template_t *template, 
    size_t *max_number_of_samples, uint8_t ALLOCATION_MODE);

void template_deallocate_memory(template_t *template);

#ifdef __cplusplus
}  // extern C
#endif

#endif
