/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file free_space_motion_tube_activity.c
 * @date March 22, 2022
 **/

#include <math.h>
#include <free_space_motion_tube/activity/activity.h>
#include <time.h>
/** 
 * The config() has to be scheduled everytime a change in the LCSM occurs, 
 * so it properly configures the schedule for the next iteration according
 * to the LCSM state, resources, task, ..  
 * @param[in] activity data structure for the free space activity
*/
void free_space_activity_config(activity_t* activity){
    // Remove config() from the eventloop schedule in the next iteration
    remove_schedule_from_eventloop(&activity->schedule_table, "activity_config");
    // Deciding which schedule to add
    switch (activity->lcsm.state){
        case CREATION:
            printf("creation\n");
            add_schedule_to_eventloop(&activity->schedule_table, "creation");
            break;
        case RESOURCE_CONFIGURATION:
            printf("resource configuration\n");
            add_schedule_to_eventloop(&activity->schedule_table, "resource_configuration");
            break;
        case PAUSING:
            printf("pausing\n");
            add_schedule_to_eventloop(&activity->schedule_table, "pausing");
            break;
        case RUNNING:
            printf("running\n");
            add_schedule_to_eventloop(&activity->schedule_table, "running");
            break;
        case CLEANING:
            break;
        case DONE:
            break;
    }
};

// Creation
void free_space_activity_creation_coordinate(activity_t *activity){
    free_space_activity_coordination_state_t * coord_state = (free_space_activity_coordination_state_t *) activity->state.coordination_state;
    // Coordinating own activity
    if (activity->state.lcsm_flags.creation_complete)
        activity->lcsm.state = RESOURCE_CONFIGURATION;
    update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void free_space_activity_creation_configure(activity_t *activity){
    if (activity->lcsm.state != CREATION){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "creation");
    }
}

void free_space_activity_creation_compute(activity_t *activity){
    // Set the flag below to true when the creation behaviour has finished    
    free_space_activity_params_t *params = (free_space_activity_params_t*) activity->conf.params; 
    free_space_activity_coordination_state_t *coord_state = (free_space_activity_coordination_state_t *) activity->state.coordination_state;
    free_space_activity_continuous_state_t *continuous_state = (free_space_activity_continuous_state_t *) activity->state.computational_state.continuous;

    activity->state.lcsm_flags.creation_complete = false;

    if (coord_state->lidar_ready){
        params->range_sensor = *(params->rt_range_sensor);
        params->range_scan.measurements = malloc(sizeof * params->range_scan.measurements * 
            params->range_sensor.nb_measurements);
        params->range_scan.angles = malloc(sizeof *params->range_scan.angles * 
            params->range_sensor.nb_measurements); 
        continuous_state->range_motion_tube.angle = (double *) malloc(300*sizeof(double));
        continuous_state->range_motion_tube.range = (double *) malloc(300*sizeof(double));
        continuous_state->range_motion_tube.index = (int *) malloc(300*sizeof(int));
        continuous_state->range_motion_tube.number_of_elements = 0;
        params->rt_range_motion_tube->angle = (double *) malloc(300*sizeof(double));
        params->rt_range_motion_tube->range = (double *) malloc(300*sizeof(double));
        params->rt_range_motion_tube->index = (int *) malloc(300*sizeof(int));
        params->rt_range_motion_tube->number_of_elements = 0;
        // Check whether memory has been properly allocated
        if (params->range_scan.measurements!= NULL && params->range_scan.angles!=NULL)   
            activity->state.lcsm_flags.creation_complete = true; 
    }
}

void free_space_activity_creation(activity_t *activity){
    free_space_activity_creation_compute(activity);
    free_space_activity_creation_coordinate(activity);
    free_space_activity_creation_configure(activity);
}

// Resource configuration
void free_space_activity_resource_configuration_coordinate(activity_t *activity){
    free_space_activity_coordination_state_t *coord_state = (free_space_activity_coordination_state_t *) activity->state.coordination_state;
    // Internal coordination
    if (activity->state.lcsm_flags.resource_configuration_complete){
        switch (activity->state.lcsm_protocol){ 
            case INITIALISATION:
                activity->lcsm.state = PAUSING;
                break;
            case EXECUTION:
                activity->lcsm.state = RUNNING;
                break;
            case DEINITIALISATION:
                activity->lcsm.state = DONE;
                activity->state.lcsm_flags.deletion_complete = true;
                break;
        }
        update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
    }
}

void free_space_activity_resource_configuration_configure(activity_t *activity){
    if (activity->lcsm.state != RESOURCE_CONFIGURATION){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "resource_configuration");
        // Update flags for next visit to the resource configuration LCS 
        activity->state.lcsm_flags.resource_configuration_complete = false;
    }
}

void free_space_activity_resource_configuration_compute(activity_t *activity){
    free_space_activity_params_t *params = (free_space_activity_params_t*) activity->conf.params; 
    free_space_activity_continuous_state_t *continuous_state = (free_space_activity_continuous_state_t *) activity->state.computational_state.continuous;

    // Read file
    // int config_status_free_space_activity;
    // configure_free_space_activity_from_file(params->configuration_file, 
    //     params, &config_status_free_space_activity);
    
    // Set the flag below to true when the resource configuartion behaviour has finished
    activity->state.lcsm_flags.resource_configuration_complete = true;
}

void free_space_activity_resource_configuration(activity_t *activity){
    free_space_activity_resource_configuration_compute(activity);
    free_space_activity_resource_configuration_coordinate(activity);
    free_space_activity_resource_configuration_configure(activity);
}

// Pausing
void free_space_activity_pausing_coordinate(activity_t *activity){
    free_space_activity_coordination_state_t * coord_state = (free_space_activity_coordination_state_t *) activity->state.coordination_state;
    // Coordinating with other activities
    if (*coord_state->lidar_ready)
        activity->state.lcsm_protocol = EXECUTION;
    if (*coord_state->lidar_dead)
        activity->state.lcsm_protocol = DEINITIALISATION;

    // Coordinating own activity
    switch (activity->state.lcsm_protocol){ 
        case EXECUTION:
            activity->lcsm.state = RUNNING;
            break;
        case DEINITIALISATION:
            activity->lcsm.state = RESOURCE_CONFIGURATION;
            break;
    }
    update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void free_space_activity_pausing_configure(activity_t *activity){
    if (activity->lcsm.state != PAUSING){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "pausing");
    }
}

void free_space_activity_pausing(activity_t *activity){
    free_space_activity_pausing_coordinate(activity);
    free_space_activity_pausing_configure(activity);
}

// Running
void free_space_activity_running_communicate_sensor_and_estimation(activity_t *activity){
    free_space_activity_params_t *params = (free_space_activity_params_t*) activity->conf.params; 
    free_space_activity_continuous_state_t *continuous_state = (free_space_activity_continuous_state_t *) activity->state.computational_state.continuous;
    free_space_activity_coordination_state_t *coord_state = (free_space_activity_coordination_state_t *) activity->state.coordination_state;

    // Copying range measurements from shared memory to a local buffer
    pthread_mutex_lock(coord_state->range_scan_lock);
    params->range_scan.nb_measurements = params->rt_range_scan->nb_measurements;
    for(int i=0; i<params->range_scan.nb_measurements; i++){ 
        params->range_scan.measurements[i] = params->rt_range_scan->measurements[i]; 
        params->range_scan.angles[i] = params->rt_range_scan->angles[i]; 
    }
    pthread_mutex_unlock(coord_state->range_scan_lock);

    // Copying proprioception pose from shared memory to a local buffer
    // pthread_mutex_lock(coord_state->velocity_lock);
    // params->current_velocity = *params->rt_current_velocity;
    // pthread_mutex_unlock(coord_state->velocity_lock);
}

void free_space_activity_running_coordinate(activity_t *activity){
    free_space_activity_coordination_state_t *coord_state = (free_space_activity_coordination_state_t *) activity->state.coordination_state;
    // Coordinating with other activities
    if (*coord_state->deinitialisation_request || 
            *coord_state->lidar_dead)
        activity->state.lcsm_protocol = DEINITIALISATION;

    switch (activity->state.lcsm_protocol){ 
        case DEINITIALISATION:
            activity->lcsm.state = RESOURCE_CONFIGURATION;
            break;
    }
    update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void free_space_activity_running_configure(activity_t *activity){
    free_space_activity_params_t *params = (free_space_activity_params_t*) activity->conf.params;
    if (activity->lcsm.state != RUNNING){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "running");
    }
}

void free_space_activity_running_compute(activity_t *activity){
    free_space_activity_params_t *params = (free_space_activity_params_t*) activity->conf.params; 
    free_space_activity_continuous_state_t *continuous_state = (free_space_activity_continuous_state_t *) activity->state.computational_state.continuous;
    free_space_activity_coordination_state_t *coord_state = (free_space_activity_coordination_state_t *) activity->state.coordination_state;

    // Data available (see communication_sensor_and_estimation)
    range_scan_t *range_scan = &params->range_scan;  // lastest measurements made available by the lidar activity

    // Platform and sensor parameters
    range_sensor_t *range_sensor = &params->range_sensor;  // parameters of the range sensor

    // Motion tube
    range_motion_tube_t *range_motion_tube = &continuous_state->range_motion_tube;

    bool is_available = false;

    // Allocating memory for template
    point2d_t sensor_pos = {.x =0, .y =0};
    polyline_t body_geometry;
    body_t body;
    body.type = POLYLINE;
    body.geometry = (void *) &body_geometry;
    body_geometry.points = (point2d_t *) malloc(4 * sizeof(point2d_t));
    body_geometry.points[FRONT_LEFT].x = .4;
    body_geometry.points[FRONT_LEFT].y = .15;
    body_geometry.points[AXLE_LEFT].x = .0;
    body_geometry.points[AXLE_LEFT].y = .15; 
    body_geometry.points[FRONT_RIGHT].x = .4;
    body_geometry.points[FRONT_RIGHT].y = -.15;
    body_geometry.points[AXLE_RIGHT].x = .0;
    body_geometry.points[AXLE_RIGHT].y = -.15; 

    // maneuver
    maneuver_t maneuver;
    unicycle_control_t control;
    maneuver.control = (void *) &control;
    // template
    double sampling_interval = 0.1;
    template_t cartesian_template;
    cartesian_template.left.number_of_points = 0;
    cartesian_template.left.max_number_of_points = 100;
    cartesian_template.left.points = (point2d_t *) malloc(cartesian_template.left.max_number_of_points * sizeof(point2d_t));
    cartesian_template.front.number_of_points = 0;
    cartesian_template.front.max_number_of_points = 100;
    cartesian_template.front.points = (point2d_t *) malloc(cartesian_template.front.max_number_of_points * sizeof(point2d_t));
    cartesian_template.right.number_of_points = 0;
    cartesian_template.right.max_number_of_points = 100;
    cartesian_template.right.points = (point2d_t *) malloc(cartesian_template.right.max_number_of_points * sizeof(point2d_t));


    template_sensor_space_t sensor_space_template;
    int nb_cartesian_points = cartesian_template.left.max_number_of_points + 
        cartesian_template.front.max_number_of_points +
        cartesian_template.right.max_number_of_points;

    sensor_space_template.number_of_beams = 0;
    sensor_space_template.max_number_of_beams = nb_cartesian_points;
    sensor_space_template.beams = (free_space_beam_t *) 
        malloc(nb_cartesian_points * sizeof(free_space_beam_t));  

    maneuver.time_horizon = 2 ;
    control.forward_velocity = 0.5;
    control.angular_rate = .0   ;

    sample_free_space_template_in_cartesian(&maneuver, &body, 
        sampling_interval, &cartesian_template);    

    template_to_sensor_space(&cartesian_template, range_sensor, 
        &sensor_pos, &maneuver, &sensor_space_template);

    monitor_template_availability(&sensor_space_template, range_scan, 
        range_sensor, &is_available);

    for(int i=0; i<sensor_space_template.number_of_beams; i++){
        range_motion_tube->angle[i] = sensor_space_template.beams[i].angle;
        range_motion_tube->range[i] = sensor_space_template.beams[i].range_outer;
        range_motion_tube->index[i] = sensor_space_template.beams[i].index;
    }
    range_motion_tube->number_of_elements = sensor_space_template.number_of_beams;
    range_motion_tube->available = is_available;

    free(body_geometry.points);
    free(cartesian_template.right.points);
    free(cartesian_template.left.points);
    free(cartesian_template.front.points);
    free(sensor_space_template.beams);

    body_geometry.points = NULL;
    cartesian_template.right.points = NULL;
    cartesian_template.left.points = NULL;
    cartesian_template.front.points = NULL;
    sensor_space_template.beams = NULL;
    //*coord_state->lidar_dead = true;
}

void free_space_activity_running_communicate_control(activity_t *activity){
    free_space_activity_params_t *params = (free_space_activity_params_t*) activity->conf.params; 
    free_space_activity_continuous_state_t *continuous_state = (free_space_activity_continuous_state_t *) activity->state.computational_state.continuous;
    free_space_activity_coordination_state_t *coord_state = (free_space_activity_coordination_state_t *) activity->state.coordination_state;

    range_motion_tube_t *range_motion_tube = &continuous_state->range_motion_tube;
    range_motion_tube_t *rt_range_motion_tube = params->rt_range_motion_tube;

    // Copying range measurements from shared memory to a local buffer
    pthread_mutex_lock(coord_state->motion_tube_lock);
    for(int i=0; i<range_motion_tube->number_of_elements; i++){
        rt_range_motion_tube->angle[i] = range_motion_tube->angle[i];
        rt_range_motion_tube->range[i] = range_motion_tube->range[i];
        rt_range_motion_tube->index[i] = range_motion_tube->index[i];
    }
    rt_range_motion_tube->number_of_elements = range_motion_tube->number_of_elements;
    rt_range_motion_tube->available = range_motion_tube->available;
    pthread_mutex_unlock(coord_state->motion_tube_lock);

}

void free_space_activity_running(activity_t *activity){
    free_space_activity_running_communicate_sensor_and_estimation(activity);
    free_space_activity_running_coordinate(activity);
    free_space_activity_running_configure(activity);
    free_space_activity_running_compute(activity);
    free_space_activity_running_communicate_control(activity);
}

// SCHEDULER 
void free_space_activity_register_schedules(activity_t *activity){
    schedule_t schedule_config = {.number_of_functions = 0};
    register_function(&schedule_config, (function_ptr_t) free_space_activity_config, 
        activity, "activity_config");
    register_schedule(&activity->schedule_table, schedule_config, "activity_config");

    schedule_t schedule_creation = {.number_of_functions = 0};
    register_function(&schedule_creation, (function_ptr_t) free_space_activity_creation, 
        activity, "creation");
    register_schedule(&activity->schedule_table, schedule_creation, "creation");

    schedule_t schedule_resource_configuration = {.number_of_functions = 0};
    register_function(&schedule_resource_configuration, (function_ptr_t) free_space_activity_resource_configuration, 
        activity, "resource_configuration");
    register_schedule(&activity->schedule_table, schedule_resource_configuration, 
        "resource_configuration");

    schedule_t schedule_pausing = {.number_of_functions = 0};
    register_function(&schedule_pausing, (function_ptr_t) free_space_activity_pausing, 
        activity, "pausing");
    register_schedule(&activity->schedule_table, schedule_pausing, 
        "pausing");

    schedule_t schedule_running = {.number_of_functions = 0};
    register_function(&schedule_running, (function_ptr_t) free_space_activity_running, 
        activity, "running");
    register_schedule(&activity->schedule_table, schedule_running, 
        "running");

}

void free_space_activity_create_lcsm(activity_t *activity, const char* name_algorithm){
    activity->conf.params = malloc(sizeof(free_space_activity_params_t));
    activity->state.computational_state.continuous = malloc(sizeof(free_space_activity_continuous_state_t));
    activity->state.computational_state.discrete = malloc(sizeof(free_space_activity_discrete_state_t));
    activity->state.coordination_state = malloc(sizeof(free_space_activity_coordination_state_t));
}

void free_space_activity_resource_configure_lcsm(activity_t *activity){
    resource_configure_lcsm_activity(activity);
    // Select the inital state of LCSM for this activity
    activity->lcsm.state = CREATION;
    activity->state.lcsm_protocol = INITIALISATION;

    // Schedule table (adding config() for the first eventloop iteration)
    free_space_activity_register_schedules(activity);
    add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
}

void free_space_activity_destroy_lcsm(activity_t *activity){
    destroy_activity(activity);
}

const free_space_activity_t ec_free_space_activity ={
    .create_lcsm = free_space_activity_create_lcsm,
    .resource_configure_lcsm = free_space_activity_resource_configure_lcsm,
    .destroy_lcsm = free_space_activity_destroy_lcsm,
};

// Configuration from file
void configure_free_space_activity_from_file(const char *file_path, 
    free_space_activity_params_t *params, int *status)
{
    // define array 
    int number_of_params = 7;  // Amount of parameters to be read, set by user
    param_array_t param_array[number_of_params];

    int i = 0;  // Keeps tracks of amount of elements inside param_array
 
    // User input where to write data to 
    char platform_configuration_file[100];

    param_array[i] = (param_array_t) {"number_of_maneuvers", &params->number_of_maneuvers, PARAM_TYPE_INT}; i++;
    param_array[i] = (param_array_t) {"min_relative_orientation", &(params->min_relative_orientation), PARAM_TYPE_DOUBLE}; i++;
    param_array[i] = (param_array_t) {"max_relative_orientation", &(params->max_relative_orientation), PARAM_TYPE_DOUBLE}; i++;
    param_array[i] = (param_array_t) {"template/sampling_interval", &(params->template_sampling_interval), PARAM_TYPE_DOUBLE}; i++;
    param_array[i] = (param_array_t) {"template/number_of_samples", &(params->template_number_of_samples), PARAM_TYPE_INT}; i++;    
    param_array[i] = (param_array_t) {"template/maneuver/nominal_forward_velocity", &(params->nominal_forward_velocity), PARAM_TYPE_DOUBLE}; i++;        
    param_array[i] = (param_array_t) {"template/maneuver/time_horizon", &(params->time_horizon), PARAM_TYPE_DOUBLE}; i++;        

    // generic reader function 
    int config_status_activity;    

    // Activity itself    
    read_from_input_file(file_path, param_array, number_of_params, &config_status_activity);

    // Other parameters

    // Verification
    if (config_status_activity == CONFIGURATION_FROM_FILE_SUCCEEDED   ){
            *status = CONFIGURATION_FROM_FILE_SUCCEEDED;
    }else{
        *status = CONFIGURATION_FROM_FILE_FAILED;
    }

}




// Debugging prints!
/*

    printf("maneuver. T: %f, maneuver.v: %.2f, maneuver.w: %.2f\n", maneuver.time_horizon,
        control.forward_velocity, control.angular_rate);

    printf("left has %d points:\n", cartesian_template.left.number_of_points);
    for(int i=0; i<cartesian_template.left.number_of_points; i++){
        int j = i;
        printf("%d: (x,y) = (%f, %f)", i, cartesian_template.left.points[i].x, cartesian_template.left.points[i].y);
        printf(", range: %.2f, angle: %.2f, index: %d\n",
            sensor_space_template.beams[j].range_outer,
            sensor_space_template.beams[j].angle*180/M_PI,
            sensor_space_template.beams[j].index);
    }   

    printf("front has %d points:\n", cartesian_template.front.number_of_points);
    for(int i=0; i<cartesian_template.front.number_of_points; i++){
        printf("%d: (x,y) = (%f, %f)", i, cartesian_template.front.points[i].x, cartesian_template.front.points[i].y);
        int j = i+cartesian_template.left.number_of_points;
        printf(", range: %.2f, angle: %.2f, index: %d\n",
            sensor_space_template.beams[j].range_outer,
            sensor_space_template.beams[j].angle*180/M_PI,
            sensor_space_template.beams[j].index);
    }

    printf("right has %d points:\n", cartesian_template.right.number_of_points);
    for(int i=0; i<cartesian_template.right.number_of_points; i++){
        printf("%d: (x,y) = (%f, %f)", i, cartesian_template.right.points[i].x, cartesian_template.right.points[i].y);
        int j = i+cartesian_template.front.number_of_points+cartesian_template.left.number_of_points;
        if (j+1 > sensor_space_template.number_of_beams){
            printf("\n");
            continue;
        }
        printf(", range: %.2f, angle: %.2f, index: %d\n",
            sensor_space_template.beams[j].range_outer,
            sensor_space_template.beams[j].angle*180/M_PI,
            sensor_space_template.beams[j].index);
    }

    printf("--------------------\n");


*/