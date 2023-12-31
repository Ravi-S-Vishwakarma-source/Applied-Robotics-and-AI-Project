#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <time.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/range_finder.h>
#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>
#include <unistd.h> // this library uses for timing control

#define TIME_STEP 64
#define MAX_SPEED 5.24
#define CRUISING_SPEED 5
#define TOLERANCE -0.1
#define OBSTACLE_THRESHOLD 0.05
#define SLOWDOWN_FACTOR 0.5
#define COMMUNICATION_CHANNEL 1


static WbDeviceTag left_wheel, right_wheel;
static WbDeviceTag communication;
static WbDeviceTag camera;
static int time_step = 0;
static int tps = 32;



/* Define Left Distacne Sensors */
static WbDeviceTag so0, so1, so2 ;
/* Define Front Distance Sensors */
static WbDeviceTag so3, so4;
/* Deigne Right Distance Sensors */
static WbDeviceTag so5, so6, so7; 

static double red = 100;
static double blue = 100;
static int message_printed = 0;  /* used to avoid printing continuously the communication state */
int r = 0;
int g = 0;
int b = 0;

static void initialize() {
// necessary to initialize Webots
  wb_robot_init();
  // get time step and robot's devices
  /* Get Left Distances Sensors Devices */
  so0 = wb_robot_get_device("so0");
  so1 = wb_robot_get_device("so1");
  so2 = wb_robot_get_device("so2");
  
  /* Get Front Distance Sensors Devices */
  so3 = wb_robot_get_device("so3");
  so4 = wb_robot_get_device("so4");
  
  /* Get Right Distance Sensors Devices */ 
  so5 = wb_robot_get_device("so5");
  so6 = wb_robot_get_device("so6");
  so7 = wb_robot_get_device("so7");
  
  time_step = wb_robot_get_basic_time_step();
  communication = wb_robot_get_device("receiver");
  camera = wb_robot_get_device("camera");
  wb_receiver_enable(communication, time_step);
  wb_camera_enable(camera, time_step);
  
  //Add any sensors you need to delcare here add any other sensors declare them under here look at the base pioneer3dx code to try to work out which variables need to be declared
 
  /* Enable Left Distance Sensors */
  wb_distance_sensor_enable(so0, time_step);
  wb_distance_sensor_enable(so1, time_step);
  wb_distance_sensor_enable(so2, time_step);
  
  /* Enable Front Distance Sensors */
  wb_distance_sensor_enable(so3, time_step);
  wb_distance_sensor_enable(so4, time_step);
  
  /* Enable Right Distance Sensors */
  wb_distance_sensor_enable(so5, time_step);
  wb_distance_sensor_enable(so6, time_step);
  wb_distance_sensor_enable(so7, time_step);
  
  
  left_wheel = wb_robot_get_device("left wheel");
  right_wheel = wb_robot_get_device("right wheel");

  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  wb_motor_set_velocity(left_wheel, 5);
  wb_motor_set_velocity(right_wheel, 5);
}

static int homeostasis() {
//You must not change this function. This is a simple decay of the Blue and Red resouces. you may only comment out the print statement
  red -= (1.0 / tps);
  blue -= (1.0 / tps);
  printf(" \nRed is %f and Blue is %f\n", red, blue);
  return (red > 0 && blue > 0);
}

static void recoverRed() {
//You must not change this function
  if (red <= 100) {
    red += 1;
  }
}

static void recoverBlue() {
//You must not change this function
  if (blue <= 100) {
    blue += 1;
  }
}

static void message() {
//You may change this function but be very careful!
  if (wb_receiver_get_queue_length(communication) > 0) {
    const char *buffer = wb_receiver_get_data(communication);
    if (message_printed != 1) {
      message_printed = 1;
    }
    int redCheck = strcmp(buffer, "Red");
    int blueCheck = strcmp(buffer, "Blue");
    if (redCheck == 0) {
      recoverRed();
    }
    if (blueCheck == 0) {
      recoverBlue();
    }
    wb_receiver_next_packet(communication);
  }
}


static void move(int l, int r) {
  wb_motor_set_velocity(left_wheel, l * MAX_SPEED);
  wb_motor_set_velocity(right_wheel, r * MAX_SPEED);
}




static void readSensors(double *left_distance, double *front_distance, double *right_distance) {
    // Left Distance Sensor
    double distance_so0 = wb_distance_sensor_get_value(so0);
    double distance_so1 = wb_distance_sensor_get_value(so1);
    double distance_so2 = wb_distance_sensor_get_value(so2);

    // Front Distance Sensor
    double distance_so3 = wb_distance_sensor_get_value(so3);
    double distance_so4 = wb_distance_sensor_get_value(so4);

    // Right Distance Sensor
    double distance_so5 = wb_distance_sensor_get_value(so5);
    double distance_so6 = wb_distance_sensor_get_value(so6);
    double distance_so7 = wb_distance_sensor_get_value(so7);

    // Combine distances sensors (sensor fusion)
    /*Calculate the sum of the so0 +  so1 + so2 and then dividing sum by 3.0 to get the average distances values
    store in the (left_distance) memory location */
    *left_distance = (distance_so0 + distance_so1 + distance_so2)/3.0; 
    /*Calculate the sum of the so3 +  so4 and then dividing sum by 2.0 to get the average distances values
    store in the (front_distance) memory location */
    *front_distance = (distance_so3 + distance_so4)/ 2.0;
    
    /*Calculate the sum of the so5 +  so6 + so7 and then dividing sum by 3.0 to get the average distances values
    store in the (right_distance) memory location */
    *right_distance = (distance_so5 + distance_so6 + distance_so7)/ 3.0;

    // Printing or using the distances sensor values
    printf(" \nLeft Sensor: %lf\n", *left_distance);
    printf("Front Sensor: %lf\n", *front_distance);
    printf("Right Sensor: %lf\n", *right_distance);
    
}

//This function is to detecting the obstacles based on distances in different direction 
static void detectObstacles(double left_distance, double front_distance, double right_distance) {
    
    // Check if the left distance sensors detects obstacles on the left
    if (left_distance > OBSTACLE_THRESHOLD) { 
        printf("Obstacle detected on the Left!\n"); // when obstacles detected then print the message in the string (" ")
    }
    
    //Check if the obstacles detected in front.
    if (front_distance > OBSTACLE_THRESHOLD) {
        printf("Obstacle detected in Front!\n");
    }
    
    // Check if the obstacles detected on the right
    if (right_distance > OBSTACLE_THRESHOLD) {
        printf("Obstacle detected on the Right!\n");
    }
    
}

static void avoidObstacles() {
    double left_distance = 0, front_distance = 0, right_distance = 0; //intial value set to 0, when no obstacles detected at the start of the program.
    readSensors(&left_distance, &front_distance, &right_distance); // calling the readSensors function
    double obstacle_threshold = 500.0; // define the obstacle threshold for Corners
    
    // Check if the robot detected obstacles in all three distance sensors
    if (left_distance >= obstacle_threshold && front_distance >= obstacle_threshold && right_distance >= obstacle_threshold) {
        
        // Move backward and turn right until no obstacles are detected
        while (left_distance >= obstacle_threshold && front_distance >= obstacle_threshold && right_distance >= obstacle_threshold) {
            usleep(TIME_STEP * 1000); // time delay to control the loop speed and allow robot to respond to the movement commands
            move(-1, -1); // move backward
            usleep(TIME_STEP * 1000); 
            //readSensors(&left_distance, &front_distance, &right_distance);
        }
        
    } else { // continue with the obstacles avoidance function below 
/* Check if obstacles are detected on the left sensors, and take actions
if left distance sensors greater then or equal to certain distance ( distance can be adjusted if requuires) */
        if (left_distance >= 330) {   
            // If an obstacle is detected on the left, turn right
            usleep(TIME_STEP * 1000);
            move(1, -1);
            printf("\nTurning Right!...\n");
            return;
        }
    
        if (front_distance >= 500){ // Check if obstacles are detected in front
            // If an obstacle is detected in front, stop and turn right
            usleep(TIME_STEP  * 1000);
            move(1, -1);
            printf("\nTurning Right!...\n");
            return;
        } 
    
        if (right_distance >= 350) { // Check if obstacles are detected on the right 
            // If an obstacle is detected on the right, turn Left
            usleep(TIME_STEP * 1000);
            move(0, 1);
            printf("\nTurning Left!..\n");
            return;
        } 
    
        // If no obstacle detected, continue moving forward
        printf("\nMoving Forward!.....\n");
        move(1, 1);
    }  
}



static void color() {
  //Set the color variables to 0
  r = 0;
  g = 0;
  b = 0;
  //Capture an image using camera on robot's
  const unsigned char *image = wb_camera_get_image(camera);
  
  //Set dimensions image window
  int image_width = 64;  //Horizontal lenght of the image
  int image_height = 64; //Vertical length of the image
  for (int y = 0; y < image_height; ++y) {
    for (int x = 0; x < image_width; ++x) {
      r += wb_camera_image_get_red(image, image_width, x, y); 
      g += wb_camera_image_get_green(image, image_width, x, y);
      b += wb_camera_image_get_blue(image, image_width, x, y);
    }
  }
  r = r / 4096;
  g = g / 4096;
  b = b / 4096;
}

static void explore() {
    
    // Check if red resource is low
    if (red <= 50) { 
        printf(" \nI need Food...\n");
        move(1,1);
        usleep(TIME_STEP * 1000);
        avoidObstacles();
        return;  // Exit the function to avoid further actions
    }

    // Check if blue resource is low
    if (blue <= 50) {
        printf(" \nI need Water...\n");
        move(1,1);
        usleep(TIME_STEP * 1000);
        avoidObstacles();
        return;  // Exit the function to avoid further actions
    }
    
    
    printf(" \nExploring....\n");
    // Continue obstacle avoidance
    avoidObstacles();
}

int main() {
  initialize();

  while (wb_robot_step(time_step) != -1 && homeostasis() == 1) { // While the robot is alive
    move(1, 1); // move forward
    message();
    printf(" Red = %d, Green = %d, Blue = %d\n", r, g, b); // print the average color values from the camera
    color(); //Calling color() function in main to execute 
    
    double left_distance, front_distance, right_distance; // Declare variables to store distance sensor readings
    readSensors(&left_distance, &front_distance, &right_distance); // read the sensors values and update the variables using readSensors function
    detectObstacles(left_distance, front_distance, right_distance); // Detect obstacles using the updated distances values
    explore();
    
    
  }

  move(0, 0);
  printf(" \nShutting down, GOODBYE:(\n");
  wb_robot_cleanup();
  return 0;
}