/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file free_space_activity.h
 * @date Mar 22, 2022
 **/

#ifndef FREE_SPACE_ACTIVITY_H
#define FREE_SPACE_ACTIVITY_H

// Mutex
#include <pthread.h>

// AACAL
#include <five_c/activity/activity.h>
#include <read_file/read_file.h>

#include <sensor_data_structure/range_scan.h>
#include <sensor_data_structure/range_sensor.h>
#include <geometry_data_structure/pose2d.h>
#include <mechanics_data_structure/body.h>
#include <mechanics_data_structure/velocity.h>
#include <platform_data_structure/kelo_tricycle.h>
#include <navigation_data_structure/range_motion_tube.h>

// Free space
#include <free_space_motion_tube/core/free_space.h>
#include <free_space_motion_tube/core/basic.h>
#include <free_space_motion_tube/core/motion_tube.h>

#define NSEC_TO_SEC 0.000000001

typedef struct free_space_activity_s{
    void (*create_lcsm)(activity_t*, const char* activity_name);
    void (*resource_configure_lcsm)(activity_t*);
    void (*destroy_lcsm)(activity_t*);
}free_space_activity_t;

// Parameters
typedef struct free_space_activity_params_s{
    // input data
    char configuration_file[100]; 
    range_sensor_t *rt_range_sensor, range_sensor;
    range_scan_t *rt_range_scan, range_scan;
    velocity_t *rt_current_velocity, current_velocity;
    velocity_t *rt_des_platform_velocity; 
    
    // platform and sensor params
    kelo_tricycle_t *platform;
    
    // Template
    motion_tube_t *motion_tube;
    maneuver_t maneuver;
    point2d_t sensor_pos;
    body_t body;

    // Parameters
    int number_of_maneuvers;
    double time_horizon;
    double nominal_forward_velocity;
    double template_sampling_interval;
    int template_number_of_samples;
    double max_relative_orientation, min_relative_orientation;

    range_motion_tube_t *rt_range_motion_tube;    
    
}free_space_activity_params_t;

// Continuous state
typedef struct free_space_activity_continuous_state_s{
    // control commands
    velocity_t des_platform_velocity;
    range_motion_tube_t range_motion_tube;
}free_space_activity_continuous_state_t;

// Discrete state
typedef struct free_space_activity_discrete_state_s{
}free_space_activity_discrete_state_t;

typedef struct free_space_coordination_state_s {
    // Coordination flags
    bool *execution_request;
    bool *deinitialisation_request;
    // Coordination with other activities
    bool *platform_control_ready, *platform_control_dead;
    bool *lidar_ready, *lidar_dead; 
    // Mutex
    pthread_mutex_t *range_scan_lock, *platform_control_lock, *velocity_lock;
    pthread_mutex_t *motion_tube_lock;
} free_space_activity_coordination_state_t;

extern const free_space_activity_t ec_free_space_activity;

void configure_free_space_activity_from_file(const char *file_path, 
    free_space_activity_params_t *params, int *status);
    
#endif //FREE_SPACE_ACTIVITY_H

