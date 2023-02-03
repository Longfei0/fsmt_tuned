#include<free_space_motion_tube/core/template_sensor_space.h>

const struct TemplateSensorSpace TemplateSensorSpace ={
    .create = template_sensor_space_create,
    .allocate_memory = template_sensor_space_allocate_memory,
    .deallocate_memory = template_sensor_space_deallocate_memory,
    .Monitor = {
        .availability = template_sensor_space_monitor_availability 
    }
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

void template_sensor_space_monitor_availability(
    const template_sensor_space_t *template, const lidar_t *lidar, 
    bool *is_available){
    range_sensor_t *range_sensor = lidar->range_sensor;
    range_scan_t *range_scan = lidar->range_scan;
    free_space_beam_t *beams = template->beams;

    *is_available = true;   // until proven otherwise..
    for(int i=0; i<template->number_of_beams; i++){
        double measurement = range_scan->measurements[template->beams[i].index];
        // Check if measurements is valid (within bounds).
        if(measurement > range_sensor->min_distance && measurement < range_sensor->max_distance){
            // Check if there is free space along beam direction.
            if (measurement < beams[i].range_outer){
                // Space is not free along beam, search can stop.
                *is_available = false;
                break;
            }
        }
    }

}