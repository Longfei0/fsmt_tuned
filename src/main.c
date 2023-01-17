#include <aacal/thread/thread.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <free_space_activity/free_space_activity.h>

bool platform_control_dead, lidar_dead;
static int interrupted;

static void sigint_handler(int sig) { 
  lidar_dead = true;
  platform_control_dead = true; }

int main(void) {
  signal(SIGINT, sigint_handler);
  thread_t main_thread;
  activity_t free_space_activity;
  // free_space_activity
  ec_free_space_activity.create_lcsm(&free_space_activity,
                                    "free_space_activity");
  ec_free_space_activity.resource_configure_lcsm(&free_space_activity);
  
  // Mocking activities
  bool platform_control_ready, lidar_ready;
  range_scan_t rt_range_scan={
    .nb_measurements = 627,
    .angles = (double*) malloc(sizeof(double)*627),
    .measurements = (double*) malloc(sizeof(double)*627),
  };
  range_sensor_t range_sensor={
    .angular_resolution = 0.006136,
    .max_angle = 1.920544,
    .min_angle = -1.920544,
    .max_distance = 5.5,
    .min_distance = .1,
    .nb_measurements = 627
  };
  for (int i=0; i< range_sensor.nb_measurements; i++){
    rt_range_scan.measurements[i] = 5.;
  }
  pose2d_t rt_proprioception_pose;
  velocity_t rt_proprioception_velocity;
  kelo_tricycle_t platform;
  velocity_t rt_des_platform_velocity;
  pthread_mutex_t range_sensor_lock, platform_control_lock, proprioception_lock;

  // Shared memory
  free_space_activity_coordination_state_t  *coord_state = 
  (free_space_activity_coordination_state_t *) free_space_activity.state.coordination_state; 
  free_space_activity_continuous_state_t  *continuous_state = 
  (free_space_activity_continuous_state_t *) free_space_activity.state.computational_state.continuous; 
  free_space_activity_params_t  *params = 
  (free_space_activity_params_t *) free_space_activity.conf.params; 

  coord_state->platform_control_ready = &platform_control_ready;
  coord_state->lidar_ready = &lidar_ready;
  coord_state->platform_control_dead = &platform_control_dead;
  coord_state->lidar_dead = &lidar_dead;
  coord_state->range_sensor_lock = &range_sensor_lock;
  coord_state->proprioception_lock = &proprioception_lock;
  coord_state->platform_control_lock = &platform_control_lock;
  continuous_state->rt_des_platform_velocity = &rt_des_platform_velocity;
  params->rt_proprioception_pose = &rt_proprioception_pose;
  params->rt_proprioception_velocity = &rt_proprioception_velocity;
  params->rt_range_scan = &rt_range_scan;
  params->range_sensor = &range_sensor;
  

  strcpy(params->configuration_file, "../configuration/free_space.json");
  // Setting configuration file

  create_thread(&main_thread, "main_thread", 100);
  register_activity(&main_thread, &free_space_activity,
                    "free_space_activity");
  
  platform_control_ready = true;
  lidar_ready = true;

  do_thread_loop((void*) &main_thread);

  printf("Exiting...\n");
  return 0;
}
