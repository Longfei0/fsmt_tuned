#include<free_space_motion_tube/core/motion_tube.h>
#include<free_space_motion_tube/core/motion_tube_cartesian.h>
#include<free_space_motion_tube/core/motion_tube_sensor_space.h>

const struct MotionTube MotionTube ={
    .create = motion_tube_create,
    .allocate_memory = motion_tube_allocate_memory,
    .deallocate_memory = motion_tube_deallocate_memory,
    .Monitor.availability = motion_tube_availability
};

void motion_tube_create(motion_tube_t *motion_tube){
    MotionTubeCartesian.create(&motion_tube->cartesian);
    MotionTubeSensorSpace.create(&motion_tube->sensor_space);
}

void motion_tube_allocate_memory(motion_tube_t *motion_tube, 
    size_t *max_number_of_samples, uint8_t ALLOCATION_MODE){
    // Cartesian motion_tube
    MotionTubeCartesian.allocate_memory(&motion_tube->cartesian, max_number_of_samples, ALLOCATION_MODE);

    // Sensor space motion_tube
    int max_number_of_beams = 0;
    point2d_array_t *side[3] = {&motion_tube->cartesian.left, &motion_tube->cartesian.front, &motion_tube->cartesian.right};
    for(int i=0; i<3; i++){
        if (side[i]->points != NULL && side[i]->max_number_of_points > 0){
            max_number_of_beams += side[i]->max_number_of_points;
        }
    }
    MotionTubeSensorSpace.allocate_memory(&motion_tube->sensor_space, max_number_of_beams);
}

void motion_tube_deallocate_memory(motion_tube_t *motion_tube){
    MotionTubeCartesian.deallocate_memory(&motion_tube->cartesian);
    MotionTubeSensorSpace.deallocate_memory(&motion_tube->sensor_space);
}

void motion_tube_availability(const motion_tube_t* motion_tube, const lidar_t* lidar, bool* is_available){
    MotionTubeSensorSpace.Monitor.availability(&motion_tube->sensor_space, lidar, is_available);
}
