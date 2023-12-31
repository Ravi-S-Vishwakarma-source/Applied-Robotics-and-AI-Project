#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/range_finder.h>
#include <webots/robot.h>
#include <webots/receiver.h>


#define MAX_SPEED 5.24
#define CRUISING_SPEED 5
#define TOLERANCE -0.1
#define OBSTACLE_THRESHOLD 0.5
#define SLOWDOWN_FACTOR 0.5
#define COMMUNICATION_CHANNEL 1


static WbDeviceTag communication;
const float *kinect_values;
static int time_step = 0;
static int tps =32;
//Any variables that you may needs to access at 

double red = 100;
double blue =100; 
int message_printed = 0; /* used to avoid printing continuously the communication state */





static void initialize(){
  // necessary to initialize Webots
  wb_robot_init();
  // get time step and robot's devices
  time_step = wb_robot_get_basic_time_step();
  communication = wb_robot_get_device("receiver");
  wb_receiver_enable(communication, time_step);
  //Add any sensors you need to delcare here add any other sensors declare them under here look at the base pioneer3dx code to try to work out which variables need to be declared
  time_step = wb_robot_get_basic_time_step();
  left_wheel = wb_robot_get_device("left wheel");
  right_wheel = wb_robot_get_device("right wheel");
  kinectColor = wb_robot_get_device("kinect color");
  kinectRange = wb_robot_get_device("kinect range");
  wb_camera_enable(kinectColor, time_step);
  wb_range_finder_enable(kinectRange, time_step);
  const int kinect_width = wb_range_finder_get_width(kinectRange);
  const int kinect_height = wb_range_finder_get_height(kinectRange);
  const int half_width = kinect_width / 2;
  const int view_height = kinect_height / 2 + 10;
  const double max_range = wb_range_finder_get_max_range(kinectRange);
  const double range_threshold = 1.5;
  const double inv_max_range_times_width = 1.0 / (max_range * kinect_width);
  // set motors' positions
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  // set speeds
  wb_motor_set_velocity(left_wheel, 1.0);
  wb_motor_set_velocity(right_wheel, 1.0);

  // init dynamic variables
  double left_obstacle = 0.0, right_obstacle = 0.0;
  double left_speed, right_speed;
  float value;
  int i;
 
  }
  
  
static int homeostasis(){
//You must not change this function. This is a simple decay of the Blue and Red resouces. you may only comment out the print statement
 red-=(1.0/tps);
 blue-=(1.0/tps);
 printf ("My red is %f  and my blue is %f\n", red, blue);
 if (red <=0 || blue <0){
   return false;}
  else {
    return true;}
}

static void recoverRed(){
//You must not change this function
  if (red <=100){
  red +=1;
}}

static void recoverBlue(){
//You must not change this function
  if (blue <=100){
 blue+=1;
}}  

static void message(){
//You may change this function but be very careful!
  int message_printed = 0; 
  if (wb_receiver_get_queue_length(communication) > 0) {
    const char *buffer = wb_receiver_get_data(communication);
    if (message_printed != 1) {
          message_printed = 1;
        }
    int redCheck =1;
    int blueCheck =1;
    redCheck = strcmp(buffer,"Red");
    blueCheck = strcmp(buffer, "Blue");
    if (redCheck == 0){
       recoverRed();
       }
    if (blueCheck == 0){
       recoverBlue();
      }
     wb_receiver_next_packet(communication);            
 }}

// Add any new functions you need under this I've give a few dummies to get you started



static void move(int l,int r){
 int left_speed = l;
 int right_speed =r;
 wb_motor_set_velocity(left_wheel, 1.0);
 wb_motor_set_velocity(right_wheel, 1.0);
}  

//static int readSensors(){
  //return 0;}

//static void detectObstacles(){}

//static void avoidObstacles(){}

//static void explore(){}


int main() {  
  initialize();
  while (wb_robot_step(time_step) != -1 && homeostasis() == true) { // While the robot is alive
  move(1,1);
  message();
  }
  move(0,0);
  printf ("Shutting down");
  wb_robot_cleanup(); 
  }
