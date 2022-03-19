#ifndef BASIC_H
#define BASIC_H

#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define AXLE_LEFT 2
#define AXLE_RIGHT 3

enum geometry{RECTANGLE};

typedef struct point2d_s{
    double x, y;
} point2d_t;

typedef struct polyline_s{
    point2d_t *points;
    int number_of_points;
}polyline_t;

typedef struct line_segment2d_s{
    point2d_t *p_init, *p_end;
} line_segment2d_t;

typedef struct pose2d_s{
    double x, y, yaw;
} pose2d_t;

typedef struct range_sensor_s{
;
} range_sensor_t;

typedef struct body_s{
    enum geometry type;
    void *geometry;
    point2d_t center_of_mass;
}body_t;

typedef struct rectangle_s{
    // SVG standard
    double width, height;   
}rectangle_t;

typedef struct sampling_params_s
{
    double sampling_interval;   // distance between samples [m]
    int max_number_of_samples;  // max number of samples
}sampling_params_t;

void sample_line_segment(const line_segment2d_t *line_segment, 
    const sampling_params_t *sampling_params,
    polyline_t *samples);
#endif
