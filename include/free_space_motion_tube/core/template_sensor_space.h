/* ----------------------------------------------------------------------------
 * Free space motion tubes
 * ROB @ KU Leuven, Leuven, Belgium
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file template_sensor_space.h
 * @date March 15, 2022
 * Authors: Rômulo Rodrigues
 **/

#ifndef FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_SENSOR_SPACE_H
#define FREE_SPACE_MOTION_TUBE_CORE_TEMPLATE_SENSOR_SPACE_H

#include<stdlib.h>
#include<stdbool.h>

#include <sensor_data_structure/lidar.h>

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
    struct Monitor{
        void(*availability)(const template_sensor_space_t*, 
            const lidar_t*, bool*);
    }Monitor;
};

void template_sensor_space_create(template_sensor_space_t *template);

void template_sensor_space_allocate_memory(template_sensor_space_t *template, 
    size_t max_number_of_samples);

void template_sensor_space_deallocate_memory(template_sensor_space_t *template);

void template_sensor_space_monitor_availability(
    const template_sensor_space_t *template, const lidar_t *lidar, bool *is_available);

#ifdef __cplusplus
}  // extern C
#endif

#endif
