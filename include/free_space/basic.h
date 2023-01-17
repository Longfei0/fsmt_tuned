#ifndef FREE_SPACE_BASIC_H
#define FREE_SPACE_BASIC_H

#include <geometry_data_structure/pose2d.h>
#include <geometry_data_structure/point2d.h>
#include <geometry_data_structure/vector2d.h>
#include <geometry_data_structure/line_segment2d.h>
#include <geometry_data_structure/polyline.h>
#include <sensor_data_structure/range_sensor.h>
#include <mechanics_data_structure/body.h>

#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define AXLE_LEFT 2
#define AXLE_RIGHT 3

enum free_space_range {FREE_SPACE, FREE_SPACE_AND_OCCLUSION}; 
enum kinematic_model {UNICYCLE};

typedef struct unicycle_state_s{
    pose2d_t pose;
}unicycle_state_t;

typedef struct unicycle_control_s{
    double forward_velocity;
    double angular_rate;
}unicycle_control_t;

typedef struct maneuver_s{
    enum kinematic_model kinematic_model;
    double time_horizon;
    void *control;
}maneuver_t;

typedef struct template_s{
    point2d_array_t left;
    point2d_array_t right;
    point2d_array_t front;
}template_t;

typedef struct free_space_beam_s{
    enum free_space_range type;
    int index;
    double angle;
    double range_inner;
    double range_intermediate;
    double range_outer;
}free_space_beam_t;

typedef struct template_sensor_space_s{
    free_space_beam_t *beams;
    int number_of_beams;
    int max_number_of_beams;
    maneuver_t maneuver;   
}template_sensor_space_t;

void sample_line_segment(const line_segment2d_t *line_segment, 
    double sampling_interval, point2d_array_t *samples);

void points_to_vector2d(const point2d_t *origin,
    const point2d_t *end, vector2d_t *vector);


#endif
