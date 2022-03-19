#include <free_space/basic.h>
#include <math.h>

void sample_line_segment(const line_segment2d_t *line_segment, 
    const sampling_params_t *sampling_params,
    polyline_t *samples)
{
    int number_of_samples;
    double length;
    double sampling_interval;
    point2d_t *p_init, *p_end, direction;
    
    p_init = line_segment->p_init;
    p_end = line_segment->p_end;
    
    length = sqrt(pow(p_init->x - p_end->x,2) + pow(p_init->y - p_end->y,2));
    direction.x = (p_end->x - p_init->x)/length;
    direction.y = (p_end->y - p_init->y)/length;
    number_of_samples = (int) ceil(length/sampling_params->sampling_interval) + 1;
    // Refine sample interval to contain end-points
    sampling_interval = length/(number_of_samples-1);

    samples->number_of_points = 0;
    for(int i=0; i<number_of_samples; i++){
        if (i >= sampling_params->max_number_of_samples){   
            break;
        }
        samples->points[i].x = p_init->x + i*sampling_interval*direction.x;
        samples->points[i].y = p_init->y + i*sampling_interval*direction.y;
        samples->number_of_points += 1;
    }
}

