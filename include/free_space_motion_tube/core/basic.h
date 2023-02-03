#ifndef FREE_SPACE_MOTION_TUBE_CORE_BASIC_H
#define FREE_SPACE_MOTION_TUBE_CORE_BASIC_H

#include <geometry_data_structure/pose2d.h>
#include <geometry_data_structure/point2d.h>
#include <geometry_data_structure/vector2d.h>
#include <geometry_data_structure/line_segment2d.h>
#include <geometry_data_structure/polyline.h>
#include <mechanics_data_structure/body.h>
#include <motion_primitive_data_structure/maneuver.h>
#include <sensor_data_structure/range_sensor.h>


#ifdef __cplusplus
extern "C" {
#endif

#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define AXLE_LEFT 2
#define AXLE_RIGHT 3

enum free_space_range {FREE_SPACE, FREE_SPACE_AND_OCCLUSION}; 

typedef struct free_space_beam_s{
    enum free_space_range type;
    int index;
    double angle;
    double range_inner;
    double range_intermediate;
    double range_outer;
}free_space_beam_t;

typedef struct template_cartesian_s{
    point2d_array_t left;
    point2d_array_t right;
    point2d_array_t front;
}template_cartesian_t;

typedef struct template_sensor_space_s{
    maneuver_t maneuver;
    free_space_beam_t *beams;
    int number_of_beams;
    int max_number_of_beams;   
}template_sensor_space_t;

typedef struct template_s{
    template_cartesian_t cartesian;
    template_sensor_space_t sensor_space;
}template_t;

void sample_line_segment(const line_segment2d_t *line_segment, 
    double sampling_interval, point2d_array_t *samples);

void points_to_vector2d(const point2d_t *origin,
    const point2d_t *end, vector2d_t *vector);


#ifdef __cplusplus
}  // extern C
#endif

#endif
