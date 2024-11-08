/*
 * File:          my_controller_wall_follower_right.c
 * Description:   Robot follows the right-hand wall using distance sensors.
 * Author:        
 * Modifications: 
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <stdio.h>
#include <stdbool.h>

#define TIME_STEP 64       // Time step for simulation (in ms)
#define MAX_SPEED 6.28     // Maximum speed for the robot motors
#define NUMBER_SENSORS 8   // Number of proximity and light sensors

int main(int argc, char **argv) {
  // Initialize Webots API
  wb_robot_init();
  
  // Get the motors of the robot
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  // Set the motors to velocity control mode (no position limits)
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Enable the 8 proximity and light sensors of the robot
  WbDeviceTag prox_sensors[NUMBER_SENSORS];
  WbDeviceTag light_sensors[NUMBER_SENSORS];
  char sensor_name[50];
  for (int i = 0; i < NUMBER_SENSORS; ++i) {
    // Proximity sensor initialization
    sprintf(sensor_name, "ps%d", i);
    prox_sensors[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(prox_sensors[i], TIME_STEP);
    
    // Light sensor initialization
    sprintf(sensor_name, "ls%d", i);
    light_sensors[i] = wb_robot_get_device(sensor_name);
    wb_light_sensor_enable(light_sensors[i], TIME_STEP);
  }

  // Main control loop
  while (wb_robot_step(TIME_STEP) != -1) {
    double dist_sum = 0.0, light_sum = 0.0;
    printf("Sensor Values:\n");

    // Read all sensors and calculate average sensor values
    for (int i = 0; i < NUMBER_SENSORS; i++) {
      double dist_value = wb_distance_sensor_get_value(prox_sensors[i]);
      double light_value = wb_light_sensor_get_value(light_sensors[i]);
      printf("ps%d: %.1f, ls%d: %.1f\n", i, dist_value, i, light_value);
      
      dist_sum += dist_value;
      light_sum += light_value;
    }

    printf("Average distance sensor value: %.2f\n", dist_sum / NUMBER_SENSORS);
    printf("Average light sensor value: %.2f\n", light_sum / NUMBER_SENSORS);

    // Robot movement variables
    double left_speed = MAX_SPEED;
    double right_speed = MAX_SPEED;

    // Sensor readings for wall-following logic
    bool left_wall = wb_distance_sensor_get_value(prox_sensors[5]) > 80;    // Obstacle on the left side
    bool left_corner = wb_distance_sensor_get_value(prox_sensors[6]) > 80;  // Obstacle in the left-front corner
    bool front_wall = wb_distance_sensor_get_value(prox_sensors[7]) > 80;   // Obstacle directly in front

    // Robot movement logic
    if (front_wall) {
      // If there's a wall in front, turn right
      left_speed = MAX_SPEED;
      right_speed = -MAX_SPEED;
    } else if (left_wall) {
      // If there's a wall on the left, go straight
      left_speed = MAX_SPEED;
      right_speed = MAX_SPEED;
    } else {
      // If no wall on the left, turn left to find the wall
      left_speed = MAX_SPEED / 8;
      right_speed = MAX_SPEED;
    }

    if (left_corner) {
      // If there's a corner on the left, turn slightly right
      left_speed = MAX_SPEED;
      right_speed = MAX_SPEED / 8;
    }

    // Set the calculated motor speeds
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  // Clean up and exit
  wb_robot_cleanup();
  return 0;
}
