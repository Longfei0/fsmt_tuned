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
#include <aacal/activity/activity.h>
#include <read_file/read_file.h>
#include <sensor_data_structure/range_scan.h>
#include <sensor_data_structure/range_sensor.h>
#include <geometry_data_structure/pose2d.h>
#include <mechanics_data_structure/body.h>
#include <mechanics_data_structure/velocity.h>
#include <platform_data_structure/kelo_tricycle.h>

// Free space
#include <free_space/free_space.h>
#include <free_space/basic.h>

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
    range_scan_t *rt_range_scan, range_scan;
    pose2d_t *rt_proprioception_pose, proprioception_pose;
    velocity_t *rt_proprioception_velocity, proprioception_velocity;
    // platform and sensor params
    kelo_tricycle_t *platform;
    range_sensor_t *range_sensor;
    // Template
    body_t body;
    template_sensor_space_t *free_space_template;
    int number_of_maneuvers;
    double time_horizon;
    double nominal_forward_velocity;
    double template_sampling_interval;
    int template_number_of_samples;
    double max_relative_orientation, min_relative_orientation;
    point2d_t sensor_pos;
    maneuver_t maneuver;
    // Waypoints
    point2d_array_t waypoints;
    // File pointer
    FILE *logfile_ptr;
}free_space_activity_params_t;

// Continuous state
typedef struct free_space_activity_continuous_state_s{
    // control commands
    velocity_t *rt_des_platform_velocity, des_platform_velocity;
    int current_waypoint;
}free_space_activity_continuous_state_t;

// Discrete state
typedef struct free_space_activity_discrete_state_s{
}free_space_activity_discrete_state_t;

typedef struct free_space_coordination_state_s {
    // Coordination flags
    bool execution_request;
    bool deinitialisation_request;
    // Coordination with other activities
    bool *platform_control_ready, *platform_control_dead;
    bool *lidar_ready, *lidar_dead; 
    // Mutex
    pthread_mutex_t *range_sensor_lock, *platform_control_lock, *proprioception_lock;
} free_space_activity_coordination_state_t;

extern const free_space_activity_t ec_free_space_activity;

void free_space_activity_config(activity_t* activity);

void configure_free_space_activity_from_file(const char *file_path, 
    free_space_activity_params_t *params, int *status);
    
#endif //FREE_SPACE_ACTIVITY_H

