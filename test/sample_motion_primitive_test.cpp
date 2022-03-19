#include "gtest/gtest.h"
extern "C"{
  #include <free_space/motion_primitive.h>
}

TEST(MotionPrimitiveStraightLine, TwoPoints) {
  // The sampling interval is equal to the length of the trajectory
  // The number of samples must be two.
  straight_maneuver_t maneuver;
  sampling_params_t sampling_params;
  point2d_t position;
  polyline_t samples;

  // Configuring motion primitive
  maneuver.forward_velocity = .5;
  maneuver.time_horizon = 1;
  sampling_params.sampling_interval = .5;
  sampling_params.max_number_of_samples = 5;
  position.x = 0;
  position.y = 0;

  samples.number_of_points = 0;
  samples.points = (point2d_t *) 
    malloc(sampling_params.max_number_of_samples * sizeof(point2d_t));
  
  sample_motion_primitive_straight_line(&maneuver, &sampling_params, &position, &samples);
  // ASSERT number of samples
  ASSERT_EQ(samples.number_of_points, 2) << "Wrong number of samples";
  // EXPECT values at samples 
  EXPECT_EQ(samples.points[0].x, 0);
  EXPECT_EQ(samples.points[0].y, 0);
  EXPECT_EQ(samples.points[1].x, 0.5);
  EXPECT_EQ(samples.points[1].y, 0);
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}