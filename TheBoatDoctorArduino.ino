// ARDUINO MEGA PIN LAYOUT DEFINES
//#define 0
//#define 1
#define LeftMotorEncoderHallA 2 // Interrupt Pin
#define FrontMotorEncoderHallA 3 // Interrupt Pin
#define RightMotorEnable 4 // PWM Pin
#define BackMotorEnable 5 // PWM Pin
#define LeftMotorEnable 6 // PWM Pin
#define FrontMotorEnable 7 // PWM Pin
//#define 8 // PWM Pin
//#define 9 // PWM Pin
//#define 10 // PWM Pin
//#define 11 // PWM Pin
//#define 12 // PWM Pin
//#define 13 // PWM Pin
//#define 14
//#define 15
//#define 16
//#define 17
#define BackMotorEncoderHallA 18 // Interrupt Pin
#define RightMotorEncoderHallA 19 // Interrupt Pin
#define IMUSDA 20 // Interrupt Pin, SDA
#define IMUSCL 21 // Interrupt Pin, SCL
#define FrontMotorEncoderHallB 22
#define BackMotorEncoderHallB 23
#define LeftMotorEncoderHallB 24
#define RightMotorEncoderHallB 25
#define FrontMotorIn1 26
#define FrontMotorIn2 27
#define LeftMotorIn1 28
#define LeftMotorIn2 29
#define BackMotorIn1 30
#define BackMotorIn2 31
#define RightMotorIn1 32
#define RightMotorIn2 33
#define TurntableStepperEnable 34 
#define TurntableStepperDirection 35
#define TurntableStepperPulse 36
#define XGantryStepperEnable 37
#define XGantryStepperDirection 38
#define XGantryStepperPulse 39
#define ZGantryStepperEnable 40
#define ZGantryStepperDirection 41
#define ZGantryStepperPulse 42
#define PumpSwitch 43
#define FrontUltrasonicTrigger 44
#define FrontUltrasonicEcho 45
#define RightUltrasonicTrigger 46
#define RightUltrasonicEcho 47
#define XAxisLimitSwitch 48
#define ZAxisLimitSwitch 49
#define LEDSwitch 50
//#define 51
//#define 52
//#define 53

// DEBUGGING FLAGS
// #define PrintMotorEncoderValues true

// ROS INCLUDES
#include <ros.h> // ROS Serial
#include <ros/time.h> // ROS time for publishing messages
#include <tf/transform_broadcaster.h> // Transform Broadcaster for TF
#include <std_msgs/Empty.h> // Empty msg for Homing everything
#include <std_msgs/Bool.h> // Bool msg for Pump and LED switches
#include <geometry_msgs/Pose2D.h> // Pose2D msg for moving the robot base and commands the turntable, X and Z Gantry
#include <geometry_msgs/Twist.h> // Twist msg for cmd_vel
#include <sensor_msgs/JointState.h> // JointState msg for publishing the robot's current joint values
#include <sensor_msgs/Range.h> // Sensor_msgs Range message for Ultrasonic Sensors
#include <sensor_msgs/Imu.h> // Sensor_msgs Imu message for IMU

// ROS Globals
ros::NodeHandle nh;

// TF Broadcaster Globals
geometry_msgs::TransformStamped t;
char base_link[] = "/base_link";
char odom[] = "/odom";

// TF Broadcaster
tf::TransformBroadcaster broadcaster;

// Ultrasonic Sensor Publisher Globals
float current_x_position;
float current_y_position;
sensor_msgs::Range front_ultrasonic_range_msg;
sensor_msgs::Range right_ultrasonic_range_msg;
geometry_msgs::Pose2D ultrasonic_pose_msg;
long front_ultrasonic_range_duration;
long right_ultrasonic_range_duration;
float front_ultrasonic_range_distance;
float right_ultrasonic_range_distance;
char front_ultrasonic_frameid[] = "/front_ultrasonic";
char right_ultrasonic_frameid[] = "/right_ultrasonic";
float ultrasonic_sensor_offset_from_center = 0.219; // Ultrasonic sensors are around 0.219m or ~8.62in. from the center of the robot

// Ultrasonic Sensor Publishers
ros::Publisher front_ultrasonic_range_pub("/TheBoatDoctor/front/ultrasonic", &front_ultrasonic_range_msg);
ros::Publisher right_ultrasonic_range_pub("/TheBoatDoctor/right/ultrasonic", &right_ultrasonic_range_msg);
ros::Publisher ultrasonic_pose_pub("/TheBoatDoctor/ultrasonic_pose", &ultrasonic_pose_msg);

// Joint States Publisher Globals
sensor_msgs::JointState joint_states_msg;
char *joint_names[] = {"turntable", "X_motion", "Z_motion"};
float joint_state_positions[3];

// Joint States Publisher
ros::Publisher joint_states_pub("/TheBoatDoctor/joint_states", &joint_states_msg);

// IMU INCLUDES
#include <SPI.h> // SPI library included for SparkFunLSM9DS1
#include <Wire.h> // I2C library included for SparkFunLSM9DS1
#include <SparkFunLSM9DS1.h> // SparkFun LSM9DS1 library

// IMU DEFINES
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 9.20 // Declination (degrees) in Pittsburgh, PA.

// IMU Publisher Globals
sensor_msgs::Imu imu_msg;
LSM9DS1 imu;
float heading;

// IMU Publisher
ros::Publisher imu_pub("/TheBoatDoctor/imu", &imu_msg);

// Motor Encoder Globals
long front_motor_encoder_count = 0;
long left_motor_encoder_count = 0;
long back_motor_encoder_count = 0;
long right_motor_encoder_count = 0;
const float wheel_diameter = 0.1016; // 4in wheels = 0.1016m
const int gear_ratio = 131;
const float motor_rpm = 80.0;
const float distance_between_wheels = 0.33333; // There is around 0.333m (13.1") between each pair of wheels
const float distance_traveled_per_wheel_revolution = wheel_diameter * PI; // m
const float max_base_speed = distance_traveled_per_wheel_revolution * motor_rpm / 60.0; // m/s
const int encoder_counts_per_revolution = (64 / 2) * gear_ratio; // 64 CPR motor encoder, but only using an interrupt for channel A
const float min_x_position = 0.25;
const float min_y_position = 0.25;
const float x_position_threshold = 0.003;
const float y_position_threshold = 0.003;
const float avg_x_position_threshold = 0.007;
const float avg_y_position_threshold = 0.007;
const int min_base_motor_speed = 100;
const int max_base_motor_speed = 255;
float current_avg_x_position;
float current_avg_y_position;
const float avg_filter_size = 10;

// Turntable Globals
float current_turntable_theta = 0.0;
long current_turntable_step_count = 0;
const int turntable_step_interval = 300;
const int turntable_step_time = 3000;
const int turntable_steps_per_revolution = 6400;
const int min_turntable_steps = -turntable_steps_per_revolution / 4; 
const int max_turntable_steps = turntable_steps_per_revolution / 4; 
const float turntable_threshold = 2 * PI / turntable_steps_per_revolution;

// X Gantry Globals
float current_x_gantry_position = 0.0;
long x_gantry_step_count = 0;
const int num_x_gantry_steps_from_limit_switch = 300;
const int x_gantry_step_interval = 3000;
const int x_gantry_step_time = 300;
const int x_gantry_steps_per_revolution = 1600;
const float x_gantry_distance_per_revolution = 0.005; // 5 mm pitch
const float x_gantry_length = 0.25; // 300 mm length, but safety of 250mm
const long max_x_gantry_steps = (long)(x_gantry_length / x_gantry_distance_per_revolution) * x_gantry_steps_per_revolution;
const float x_gantry_threshold = x_gantry_distance_per_revolution / x_gantry_steps_per_revolution;

// Z Gantry Globals
float current_z_gantry_position;
long z_gantry_step_count;
const int num_z_gantry_steps_from_limit_switch = 100;
const int z_gantry_step_interval = 3000;
const int z_gantry_step_time = 300;
const int z_gantry_steps_per_revolution = 1600;
const float z_gantry_distance_per_revolution = 0.008; // 8 mm pitch
const float z_gantry_length = 0.35; // 350mm or ~13.78"
const long max_z_gantry_steps = (long)(z_gantry_length / z_gantry_distance_per_revolution) * z_gantry_steps_per_revolution;
const float z_gantry_threshold = z_gantry_distance_per_revolution / z_gantry_steps_per_revolution;

// ROS Callback Functions and Subscribers

// LED SWITCH

// led status Publisher
std_msgs::Bool led_status_msg;
ros::Publisher led_status_pub("/TheBoatDoctor/led_status", &led_status_msg);

// LED Switch Callback
// Turns on the LEDs if the msg contains true, 
// otherwise turns off the LEDs if the msg contains false
void ledSwitchCallback(const std_msgs::Bool& led_switch_msg){
  // If the led_switch_msg contains true, turn on the LEDs
  if(led_switch_msg.data)
  {
    digitalWrite(LEDSwitch, HIGH);
  }
  // Otherwise turn off the LEDs
  else
  {
    digitalWrite(LEDSwitch, LOW);
  }
  led_status_msg.data = led_switch_msg.data;
  led_status_pub.publish(&led_status_msg);
  delay(10);
  led_status_pub.publish(&led_status_msg);
}

// LED Switch Subscriber
ros::Subscriber<std_msgs::Bool> led_switch_sub("/TheBoatDoctor/LED_Switch", &ledSwitchCallback);

// PUMP SWITCH

// pump status Publisher
std_msgs::Bool pump_status_msg;
ros::Publisher pump_status_pub("/TheBoatDoctor/pump_status", &pump_status_msg);

// Pump Switch Callback
// Turns on the pump if the msg contains true, 
// otherwise turns off the pump if the msg contains false
void pumpSwitchCallback(const std_msgs::Bool& pump_switch_msg){
  // If the pump_switch_msg contains true, turn on the pump
  if(pump_switch_msg.data)
  {
    digitalWrite(PumpSwitch, HIGH);
  }
  // Otherwise turn off the pump
  else
  {
    digitalWrite(PumpSwitch, LOW);
  }
  pump_status_msg.data = pump_switch_msg.data;
  pump_status_pub.publish(&pump_status_msg);
  delay(10);
  pump_status_pub.publish(&pump_status_msg);
}

ros::Subscriber<std_msgs::Bool> pump_switch_sub("/TheBoatDoctor/Pump_Switch", &pumpSwitchCallback);

// STOP COMMAND
// Stop Command Globals
bool stop_flag = false;

// Stop Command Callback
// Immediately halts everything by setting all the motor control pins to LOW and sets the stop_flag to true.
// Remember to publish a false msg to restart the robot.
void stopCallback(const std_msgs::Bool& stop_msg){

  // If the stop_msg contains true, stop all of the motors
  if(stop_msg.data)
  {
    // set stop_flag to true
    stop_flag = true;

    // disable all base motors by setting all the motor control pins to LOW
    digitalWrite(FrontMotorEnable, LOW);
    digitalWrite(FrontMotorIn1, LOW);
    digitalWrite(FrontMotorIn2, LOW);

    digitalWrite(LeftMotorEnable, LOW);
    digitalWrite(LeftMotorIn1, LOW);
    digitalWrite(LeftMotorIn2, LOW);

    digitalWrite(BackMotorEnable, LOW); 
    digitalWrite(BackMotorIn1, LOW);
    digitalWrite(BackMotorIn2, LOW);

    digitalWrite(RightMotorEnable, LOW);
    digitalWrite(RightMotorIn1, LOW);
    digitalWrite(RightMotorIn2, LOW);

    // disable stepper motors by setting their enable lines to HIGH
    digitalWrite(TurntableStepperEnable, HIGH);
    digitalWrite(XGantryStepperEnable, HIGH);
    digitalWrite(ZGantryStepperEnable, HIGH);
    
    // disable pump and LEDs by setting their switches to LOW
    digitalWrite(PumpSwitch, LOW);
    digitalWrite(LEDSwitch, LOW);
  }
  // Otherwise if the stop_msg contains false, set the stop_flag back to false
  // and enable the stepper motors.
  else
  {
    stop_flag = false;

    // enable stepper motors by setting their enable lines to low
    digitalWrite(TurntableStepperEnable, LOW);
    digitalWrite(XGantryStepperEnable, LOW);
    digitalWrite(ZGantryStepperEnable, LOW);
  }
}

// Stop Command Subscriber
ros::Subscriber<std_msgs::Bool> stop_sub("/TheBoatDoctor/Stop", &stopCallback);

// HOME COMMAND
// Home Command Globals
bool home_x_gantry_flag = false;
bool home_z_gantry_flag = false;
bool home_turntable_flag = false;

// done homing Publisher
std_msgs::Bool done_homing_msg;
ros::Publisher done_homing_pub("/TheBoatDoctor/done_homing", &done_homing_msg);

// Home Command Callback
// Returns the robot to the home position by setting the homing flags to true
void homeCallback(const std_msgs::Empty& home_msg){
  home_x_gantry_flag = true;
  home_z_gantry_flag = true;
  home_turntable_flag = true;
}

// Home Command Subscriber
ros::Subscriber<std_msgs::Empty> home_sub("/TheBoatDoctor/Home", &homeCallback);

// MOVE ROBOT BASE
// Move Robot Base Globals
bool move_base_x_flag = false;
bool move_base_y_flag = false;
float desired_x_position;
float desired_y_position;
float desired_theta;

// done moving robot base Publisher
std_msgs::Bool done_moving_robot_base_msg;
ros::Publisher done_moving_robot_base_pub("/TheBoatDoctor/done_moving_robot_base", &done_moving_robot_base_msg);

// Move Robot Base Callback
// Moves the robot to a specified location in the testbed using the ultrasonic sensors and motor encoders
void moveRobotBaseCallback(const geometry_msgs::Pose2D& pose_2d_msg)
{
  desired_x_position = pose_2d_msg.x;
  desired_y_position = pose_2d_msg.y;
  desired_theta = pose_2d_msg.theta;

  if(abs(desired_x_position - current_x_position) > x_position_threshold)
  {
    move_base_x_flag = true;
  } 
  if(abs(desired_y_position - current_y_position) > y_position_threshold)
  {
    move_base_y_flag = true;
  } 
  if(!move_base_x_flag && !move_base_y_flag)
  {
    done_moving_robot_base_msg.data = true;
    done_moving_robot_base_pub.publish(&done_moving_robot_base_msg);
    delay(10);
    done_moving_robot_base_pub.publish(&done_moving_robot_base_msg);
  }
}

// Move Robot Base Subscriber
ros::Subscriber<geometry_msgs::Pose2D> move_robot_base_sub("/TheBoatDoctor/move_robot_base", &moveRobotBaseCallback);


// COMMAND VELOCITY
// Command Velocity Globals
float desired_x_vel;
float desired_y_vel;

// Command Velocity Callback
// Moves the robot at a given velocity
void cmdVelCallback(const geometry_msgs::Twist& twist_msg)
{
  if(!stop_flag)
  {
    desired_x_vel = twist_msg.linear.x;
    desired_y_vel = twist_msg.linear.y;

    if (desired_x_vel == 0.0)
    {
      digitalWrite(LeftMotorEnable, LOW);
      digitalWrite(RightMotorEnable, LOW);
      digitalWrite(LeftMotorIn1, LOW);
      digitalWrite(LeftMotorIn2, LOW);  
      digitalWrite(RightMotorIn1, LOW);
      digitalWrite(RightMotorIn2, LOW);
    }
    else if  (desired_x_vel < 0.0)
    {
      // Forward
      int x_speed = (int)(max((desired_x_vel / max_base_speed), 1.0) * 255.0);
      digitalWrite(LeftMotorIn1, HIGH);
      digitalWrite(LeftMotorIn2, LOW);  
      digitalWrite(RightMotorIn1, LOW);
      digitalWrite(RightMotorIn2, HIGH);
      analogWrite(LeftMotorEnable, x_speed);
      analogWrite(RightMotorEnable, x_speed);
    }
    else
    {
      // Backward
      int x_speed = (int)(max((-desired_x_vel / max_base_speed), 1.0) * 255.0);
      digitalWrite(LeftMotorIn1, LOW);
      digitalWrite(LeftMotorIn2, HIGH);  
      digitalWrite(RightMotorIn1, HIGH);
      digitalWrite(RightMotorIn2, LOW);
      analogWrite(LeftMotorEnable, x_speed);
      analogWrite(RightMotorEnable, x_speed);
    }

    if(desired_y_vel == 0.0)
    {
      digitalWrite(FrontMotorEnable, LOW);
      digitalWrite(BackMotorEnable, LOW);
      digitalWrite(FrontMotorIn1, LOW);
      digitalWrite(FrontMotorIn2, LOW);  
      digitalWrite(BackMotorIn1, LOW);
      digitalWrite(BackMotorIn2, LOW);
    }
    else if(desired_y_vel < 0.0)
    {
      // Right
      int y_speed = (int)(max((desired_y_vel / max_base_speed), 1.0) * 255.0);
      digitalWrite(FrontMotorIn1, HIGH);
      digitalWrite(FrontMotorIn2, LOW);  
      digitalWrite(BackMotorIn1, LOW);
      digitalWrite(BackMotorIn2, HIGH);
      analogWrite(FrontMotorEnable, y_speed);
      analogWrite(BackMotorEnable, y_speed);
    }
    else
    {
      // Left
      int y_speed = (int)(max((-desired_y_vel / max_base_speed), 1.0) * 255.0);
      digitalWrite(FrontMotorIn1, LOW);
      digitalWrite(FrontMotorIn2, HIGH);  
      digitalWrite(BackMotorIn1, HIGH);
      digitalWrite(BackMotorIn2, LOW);
      analogWrite(FrontMotorEnable, y_speed);
      analogWrite(BackMotorEnable, y_speed);
    }
  }
}

// Command Velocity Subscriber
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/TheBoatDoctor/cmd_vel", &cmdVelCallback);

// MOVE GANTRY
// Move Gantry Globals
bool move_x_gantry_flag = false;
bool move_z_gantry_flag = false;
float desired_x_gantry_position;
float desired_z_gantry_position;

// done moving gantry Publisher
std_msgs::Bool done_moving_gantry_msg;
ros::Publisher done_moving_gantry_pub("/TheBoatDoctor/done_moving_gantry", &done_moving_gantry_msg);

// Move Gantry Callback
void moveGantryCallback(const geometry_msgs::Pose2D& move_gantry_msg)
{
  desired_x_gantry_position = move_gantry_msg.x;
  desired_z_gantry_position = move_gantry_msg.y;
  
  if(abs(desired_x_gantry_position - current_x_gantry_position) > x_gantry_threshold)
  {
    move_x_gantry_flag = true;
  } 
  if(abs(desired_z_gantry_position - current_z_gantry_position) > z_gantry_threshold)
  {
    move_z_gantry_flag = true;
  }
  if(!move_x_gantry_flag && !move_z_gantry_flag)
  {
    done_moving_gantry_msg.data = true;
    done_moving_gantry_pub.publish(&done_moving_gantry_msg);
    delay(10);
    done_moving_gantry_pub.publish(&done_moving_gantry_msg);
  }
}

// Move Gantry Subscriber
ros::Subscriber<geometry_msgs::Pose2D> move_gantry_sub("/TheBoatDoctor/move_gantry", &moveGantryCallback);


// TURN TURNTABLE
// Turn Turntable Globals
bool turn_turntable_flag = false;
float desired_turntable_theta;

// done turning turntable Publisher
std_msgs::Bool done_turning_turntable_msg;
ros::Publisher done_turning_turntable_pub("/TheBoatDoctor/done_turning_turntable", &done_turning_turntable_msg);

// Turn Turntable Callback
void turnTurntableCallback(const geometry_msgs::Pose2D& turn_turntable_msg)
{
  desired_turntable_theta = turn_turntable_msg.theta;
  
  if(abs(desired_turntable_theta - current_turntable_theta) > turntable_threshold)
  {
    turn_turntable_flag = true;
  }
  else
  {
    done_turning_turntable_msg.data = true;
    done_turning_turntable_pub.publish(&done_turning_turntable_msg);
    delay(10);
    done_turning_turntable_pub.publish(&done_turning_turntable_msg);
  } 
}

// Turn Turntable Subscriber
ros::Subscriber<geometry_msgs::Pose2D> turn_turntable_sub("/TheBoatDoctor/turn_turntable", &turnTurntableCallback);

// RESET COMMAND
// Reset Command Callback
// Resets the robot's desired positions to the default positions.
void resetCallback(const std_msgs::Empty& reset_msg){
  digitalWrite(FrontMotorEnable, LOW);
  digitalWrite(LeftMotorEnable, LOW);
  digitalWrite(BackMotorEnable, LOW);
  digitalWrite(RightMotorEnable, LOW);
  digitalWrite(FrontMotorIn1, LOW);
  digitalWrite(FrontMotorIn2, LOW); 
  digitalWrite(LeftMotorIn1, LOW);
  digitalWrite(LeftMotorIn2, LOW); 
  digitalWrite(BackMotorIn1, LOW);
  digitalWrite(BackMotorIn2, LOW);
  digitalWrite(RightMotorIn1, LOW);
  digitalWrite(RightMotorIn2, LOW);

  desired_x_vel = 0.0;
  desired_y_vel = 0.0;
  desired_x_position = current_x_position;
  desired_y_position = current_y_position;
  desired_turntable_theta = 0.0;
  desired_x_gantry_position = 0.0;
  desired_z_gantry_position = 0.0;
}

// Reset Command Subscriber
ros::Subscriber<std_msgs::Empty> reset_sub("/TheBoatDoctor/Reset", &resetCallback);

// STAY COMMAND
// Stay Command Callback
// Sets the robot's desired positions to the current positions.
void stayCallback(const std_msgs::Empty& stay_msg){
  digitalWrite(FrontMotorEnable, LOW);
  digitalWrite(LeftMotorEnable, LOW);
  digitalWrite(BackMotorEnable, LOW);
  digitalWrite(RightMotorEnable, LOW);
  digitalWrite(FrontMotorIn1, LOW);
  digitalWrite(FrontMotorIn2, LOW); 
  digitalWrite(LeftMotorIn1, LOW);
  digitalWrite(LeftMotorIn2, LOW); 
  digitalWrite(BackMotorIn1, LOW);
  digitalWrite(BackMotorIn2, LOW);
  digitalWrite(RightMotorIn1, LOW);
  digitalWrite(RightMotorIn2, LOW);

  desired_x_vel = 0.0;
  desired_y_vel = 0.0;
  desired_x_position = current_x_position;
  desired_y_position = current_y_position;
  desired_turntable_theta = current_turntable_theta;
  desired_x_gantry_position = current_x_gantry_position;
  desired_z_gantry_position = current_z_gantry_position;
}

// Stay Command Subscriber
ros::Subscriber<std_msgs::Empty> stay_sub("/TheBoatDoctor/Stay", &stayCallback);


// INTERRUPT SERVICE ROUTINES
// Interrupt Service Routines for Motor Encoders
void front_motor_hallA_detect() 
{
  /* If pinA and pinB are both high or both low, it is spinning
     forward. If they're different, it's going backward.

     For more information on speeding up this process, see
     [Reference/PortManipulation], specifically the PIND register.
  */
  if (digitalRead(FrontMotorEncoderHallA) == digitalRead(FrontMotorEncoderHallB)) {
    front_motor_encoder_count++;
  } else {
    front_motor_encoder_count--;
  }

  #ifdef PrintMotorEncoderValues
    Serial.print("Front Motor Encoder Count:")
    Serial.println(front_motor_encoder_count);
  #endif
}

void left_motor_hallA_detect() 
{
  /* If pinA and pinB are both high or both low, it is spinning
     forward. If they're different, it's going backward.

     For more information on speeding up this process, see
     [Reference/PortManipulation], specifically the PIND register.
  */
  if (digitalRead(LeftMotorEncoderHallA) == digitalRead(LeftMotorEncoderHallB)) {
    left_motor_encoder_count++;
  } else {
    left_motor_encoder_count--;
  }

  #ifdef PrintMotorEncoderValues
    Serial.print("Left Motor Encoder Count:")
    Serial.println(left_motor_encoder_count);
  #endif
}

void back_motor_hallA_detect() 
{
  /* If pinA and pinB are both high or both low, it is spinning
     forward. If they're different, it's going backward.

     For more information on speeding up this process, see
     [Reference/PortManipulation], specifically the PIND register.
  */
  if (digitalRead(BackMotorEncoderHallA) == digitalRead(BackMotorEncoderHallB)) {
    back_motor_encoder_count++;
  } else {
    back_motor_encoder_count--;
  }

  #ifdef PrintMotorEncoderValues
    Serial.print("Back Motor Encoder Count:")
    Serial.println(back_motor_encoder_count);
  #endif
}

void right_motor_hallA_detect() 
{
  /* If pinA and pinB are both high or both low, it is spinning
     forward. If they're different, it's going backward.

     For more information on speeding up this process, see
     [Reference/PortManipulation], specifically the PIND register.
  */
  if (digitalRead(RightMotorEncoderHallA) == digitalRead(RightMotorEncoderHallB)) {
    right_motor_encoder_count++;
  } else {
    right_motor_encoder_count--;
  }

  #ifdef PrintMotorEncoderValues
    Serial.print("Right Motor Encoder Count:")
    Serial.println(right_motor_encoder_count);
  #endif
}

// SETUP CODE
void setup()
{ 
  // set all the base dc motor control pins to outputs
  pinMode(FrontMotorEnable, OUTPUT);
  pinMode(FrontMotorIn1, OUTPUT);
  pinMode(FrontMotorIn2, OUTPUT);
  pinMode(LeftMotorEnable, OUTPUT);
  pinMode(LeftMotorIn1, OUTPUT);
  pinMode(LeftMotorIn2, OUTPUT);
  pinMode(BackMotorEnable, OUTPUT);
  pinMode(BackMotorIn1, OUTPUT);
  pinMode(BackMotorIn2, OUTPUT);
  pinMode(RightMotorEnable, OUTPUT);
  pinMode(RightMotorIn1, OUTPUT);
  pinMode(RightMotorIn2, OUTPUT);

  // set all the stepper motor control pins to outputs
  pinMode(TurntableStepperEnable, OUTPUT);
  pinMode(TurntableStepperDirection, OUTPUT);
  pinMode(TurntableStepperPulse, OUTPUT);
  pinMode(XGantryStepperEnable, OUTPUT);
  pinMode(XGantryStepperDirection, OUTPUT);
  pinMode(XGantryStepperPulse, OUTPUT);
  pinMode(ZGantryStepperEnable, OUTPUT);
  pinMode(ZGantryStepperDirection, OUTPUT);
  pinMode(ZGantryStepperPulse, OUTPUT);

  // set the pump and LED switches to outputs
  pinMode(PumpSwitch, OUTPUT);
  pinMode(LEDSwitch, OUTPUT);

  // Not implemented currently
  // set all the motor encoder hall A and B pins to input pullup mode
  pinMode(FrontMotorEncoderHallA, INPUT_PULLUP);
  pinMode(FrontMotorEncoderHallB, INPUT_PULLUP);
  pinMode(LeftMotorEncoderHallA, INPUT_PULLUP);
  pinMode(LeftMotorEncoderHallB, INPUT_PULLUP);
  pinMode(BackMotorEncoderHallA, INPUT_PULLUP);
  pinMode(BackMotorEncoderHallB, INPUT_PULLUP);
  pinMode(RightMotorEncoderHallA, INPUT_PULLUP);
  pinMode(RightMotorEncoderHallB, INPUT_PULLUP);

  // According to documentation here: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  // digitalPin 2 = Interrupt 0
  // digitalPin 3 = Interrupt 1
  // digitalPin 21 = Interrupt 2
  // digitalPin 20 = Interrupt 3
  // digitalPin 19 = Interrupt 4
  // digitalPin 18 = Interrupt 5

  // Attach interrupts to all motor encoder hall A pins
  // FrontMotorEncoderHallA is on pin 3
  attachInterrupt(1, front_motor_hallA_detect, CHANGE); 
  // LeftMotorEncoderHallA is on pin 2
  attachInterrupt(0, left_motor_hallA_detect, CHANGE);
  // BackMotorEncoderHallA is on pin 18
  attachInterrupt(5, back_motor_hallA_detect, CHANGE);
  // RightMotorEncoderHallA is on pin 19
  attachInterrupt(4, right_motor_hallA_detect, CHANGE);

  // IMU setup and startup code
  // Taken from https://learn.sparkfun.com/tutorials/lsm9ds1-breakout-hookup-guide
  // Datasheet is here: https://cdn.sparkfun.com/assets/learn_tutorials/3/7/3/LSM9DS1_Datasheet.pdf
  imu.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C
  imu.settings.device.mAddress = LSM9DS1_M; // Set mag address to 0x1E
  imu.settings.device.agAddress = LSM9DS1_AG; // Set ag address to 0x6B
  imu.settings.accel.scale = 8; // Set accel range to +/-8g
  imu.settings.gyro.scale = 2000; // Set gyro range to +/-2000dps
  imu.settings.mag.scale = 4; // Set mag range to +/-4Gs
  imu.begin();

  // set ultrasonic sensor trigger pins to output and echo pins to input
  pinMode(FrontUltrasonicTrigger, OUTPUT);
  pinMode(FrontUltrasonicEcho, INPUT);
  pinMode(RightUltrasonicTrigger, OUTPUT);
  pinMode(RightUltrasonicEcho, INPUT);

  // set limit switches to input pullup mode
  pinMode(XAxisLimitSwitch, INPUT_PULLUP);
  pinMode(ZAxisLimitSwitch, INPUT_PULLUP); 

  // disable dc motos by setting their enable lines to low
  digitalWrite(RightMotorEnable, LOW);
  digitalWrite(BackMotorEnable, LOW);
  digitalWrite(LeftMotorEnable, LOW);
  digitalWrite(FrontMotorEnable, LOW);

  // enable stepper motors by setting their enable lines to low
  digitalWrite(TurntableStepperEnable, LOW);
  digitalWrite(XGantryStepperEnable, LOW);
  digitalWrite(ZGantryStepperEnable, LOW);

  // ROS Serial Initialization Code

  // Initialize Node
  nh.initNode();

  // Initialize tf broadcaster
  broadcaster.init(nh);
  
  // Advertise topics
  nh.advertise(front_ultrasonic_range_pub);
  nh.advertise(right_ultrasonic_range_pub);
  nh.advertise(ultrasonic_pose_pub);
  nh.advertise(imu_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(led_status_pub);
  nh.advertise(pump_status_pub);
  nh.advertise(done_homing_pub);
  nh.advertise(done_moving_robot_base_pub);
  nh.advertise(done_moving_gantry_pub);
  nh.advertise(done_turning_turntable_pub);

  // Subscribe to topics
  nh.subscribe(led_switch_sub);
  nh.subscribe(pump_switch_sub);
  nh.subscribe(stop_sub);
  nh.subscribe(home_sub);
  nh.subscribe(reset_sub);
  nh.subscribe(stay_sub);
  nh.subscribe(move_robot_base_sub);
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(move_gantry_sub);
  nh.subscribe(turn_turntable_sub);

  // Ultrasonic Sensor Msg Setup Code
  front_ultrasonic_range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  front_ultrasonic_range_msg.header.frame_id = front_ultrasonic_frameid;
  front_ultrasonic_range_msg.field_of_view = 0.2618;  // 15 degrees field of view
  front_ultrasonic_range_msg.min_range = 0.0;
  front_ultrasonic_range_msg.max_range = 4;

  right_ultrasonic_range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  right_ultrasonic_range_msg.header.frame_id = right_ultrasonic_frameid;
  right_ultrasonic_range_msg.field_of_view = 0.1;  // 15 degrees field of view
  right_ultrasonic_range_msg.min_range = 0.0;
  right_ultrasonic_range_msg.max_range = 4;

  // IMU Msg Setup Code

  // From datasheet, 1 gauss std deviation

  // From datasheet, 30 degrees per second std deviation
  // 30 degrees per second => 0.5236 radians per second
  // Variance = std deviation ^ 2 = 0.5236 * 0.5236 ~ 0.275
  imu_msg.angular_velocity_covariance[0] = 0.275;
  imu_msg.angular_velocity_covariance[4] = 0.275;
  imu_msg.angular_velocity_covariance[8] = 0.275;

  // From datasheet, 90 mg std deviation
  // 90 mg => 0.882598 m / s ^ 2
  // Variance = std deviation ^ 2 = 0.882598 * 0.882598 ~ 0.777
  imu_msg.linear_acceleration_covariance[0] = 0.777;
  imu_msg.linear_acceleration_covariance[4] = 0.777;
  imu_msg.linear_acceleration_covariance[8] = 0.777;

  // Joint States Msg Setup Code
  joint_states_msg.name_length = 3;
  joint_states_msg.velocity_length = 3;
  joint_states_msg.position_length = 3;
  joint_states_msg.effort_length = 3;
  joint_states_msg.name = joint_names;
  
  readUltrasonicSensors();
  desired_x_position = current_x_position;
  desired_y_position = current_y_position;
  current_avg_x_position = current_x_position;
  current_avg_y_position = current_y_position;
}

bool determineHoming()
{
  return (home_x_gantry_flag || home_z_gantry_flag || home_turntable_flag);
}

void home()
{
  if(home_x_gantry_flag)
  {
    homeXGantry();
  }
  else if(home_z_gantry_flag)
  {
    homeZGantry();
  }
  else if(home_turntable_flag)
  {
    homeTurntable();
  }
}

void homeXGantry()
{
  int num_steps = 0;

  while(num_steps < x_gantry_step_interval && digitalRead(XAxisLimitSwitch) != 0)
  {
    digitalWrite(XGantryStepperDirection, LOW);
    digitalWrite(XGantryStepperPulse, HIGH);
    digitalWrite(XGantryStepperPulse, LOW);
    x_gantry_step_count--;

    delayMicroseconds(x_gantry_step_time);
  }
  
  // Check to see if the X Axis Limit Switch was hit
  if(digitalRead(XAxisLimitSwitch) == 0)
  {         
    // Conduct the X Gantry Calibration Sequence
    XGantryCalibrationSequence();

    // Set the x_gantry_step_count to 0
    x_gantry_step_count = 0;

    // Set the home_x_gantry_flag to false because the x_gantry was successfully homed
    home_x_gantry_flag = false;
  }

  // Update current_x_gantry_position with the current x_gantry_step_count
  current_x_gantry_position = (x_gantry_step_count / x_gantry_steps_per_revolution) * x_gantry_distance_per_revolution;
}

// Conducts the X Gantry Calibration Sequence after the Limit Switch has been hit, which involves 
// moving the X Gantry out until the Limit Switch is no longer clicked.
void XGantryCalibrationSequence()
{
  digitalWrite(XGantryStepperDirection, HIGH);
  while (digitalRead(XAxisLimitSwitch) != 1)
  {
   digitalWrite(XGantryStepperPulse, HIGH);
   digitalWrite(XGantryStepperPulse, LOW);
   
   delayMicroseconds(x_gantry_step_time);
  }
  for (int i = 1; i < num_x_gantry_steps_from_limit_switch; i++)
  {
   digitalWrite(XGantryStepperPulse, HIGH);
   digitalWrite(XGantryStepperPulse, LOW);

   delayMicroseconds(x_gantry_step_time);
  }
}

void homeZGantry()
{
  int num_steps = 0;

  while(num_steps < z_gantry_step_interval && digitalRead(ZAxisLimitSwitch) != 0)
  {
    digitalWrite(ZGantryStepperDirection, HIGH);
    digitalWrite(ZGantryStepperPulse, HIGH);
    digitalWrite(ZGantryStepperPulse, LOW);
    z_gantry_step_count--;

    delayMicroseconds(z_gantry_step_time);
  }

  // Check to see if the Z Axis Limit Switch was hit
  if(digitalRead(ZAxisLimitSwitch) == 0)
  {         
    // Conduct the Z Gantry Calibration Sequence
    ZGantryCalibrationSequence();
    
    // Set the z_gantry_step_count to 0
    z_gantry_step_count = 0;

    // Set the home_z_gantry_flag to false because the z_gantry was successfully homed
    home_z_gantry_flag = false;
  }

  // Update current_z_gantry_position with the current z_gantry_step_count
  current_z_gantry_position = (z_gantry_step_count / z_gantry_steps_per_revolution) * z_gantry_distance_per_revolution;
}

// Conducts the Z Gantry Calibration Sequence after the Limit Switch has been hit, which involves 
// moving the Z Gantry out until the Limit Switch is no longer clicked.
void ZGantryCalibrationSequence()
{
  digitalWrite(ZGantryStepperDirection, LOW);
  while (digitalRead(ZAxisLimitSwitch) != 1)
  {
   digitalWrite(ZGantryStepperPulse, HIGH);
   digitalWrite(ZGantryStepperPulse, LOW);
   
   delayMicroseconds(z_gantry_step_time);
  }
  for (int i = 1; i < num_z_gantry_steps_from_limit_switch; i++)
  {
   digitalWrite(ZGantryStepperPulse, HIGH);
   digitalWrite(ZGantryStepperPulse, LOW);

   delayMicroseconds(z_gantry_step_time);
  }
}

void homeTurntable()
{
  if(current_turntable_step_count > 0)
  {
    digitalWrite(TurntableStepperDirection, HIGH);
    digitalWrite(TurntableStepperPulse, HIGH);
    digitalWrite(TurntableStepperPulse, LOW);
    current_turntable_step_count--;
  }
  else if(current_turntable_step_count < 0)
  {
    digitalWrite(TurntableStepperDirection, LOW);
    digitalWrite(TurntableStepperPulse, HIGH);
    digitalWrite(TurntableStepperPulse, LOW);
    current_turntable_step_count++;
  }
  else if(current_turntable_step_count == 0)
  {
    home_turntable_flag = false;
    done_homing_msg.data = true;
    done_homing_pub.publish(&done_homing_msg);
    delay(10);
    done_homing_pub.publish(&done_homing_msg);
  }
  current_turntable_theta = (((float)current_turntable_step_count) / turntable_steps_per_revolution) * 2 * PI;
}

void updateTransform()
{
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = 1.0; 
  t.transform.translation.y = 1.0;
  t.transform.translation.z = 1.0;  
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0; 
  t.transform.rotation.z = 0.0; 
  t.transform.rotation.w = 1.0;  
  t.header.stamp = nh.now();
}

void publishTransform()
{
  broadcaster.sendTransform(t);
}

void updateIMU()
{
  imu.readMag();
  imu.readGyro();
  imu.readAccel();
  determineHeading(imu.ax, imu.ay, imu.az, 
                  -imu.my, -imu.mx, imu.mz);
}

// Calculate heading.
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void determineHeading(float ax, float ay, float az, float mx, float my, float mz)
{
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
}

void publishIMU()
{
  imu_msg.header.stamp = nh.now();
  imu_msg.orientation.w = cos(heading / 2);
  imu_msg.orientation.z = sin(heading / 2);
  imu_msg.angular_velocity.x = imu.calcGyro(imu.gx) * 0.0174533;
  imu_msg.angular_velocity.y = imu.calcGyro(imu.gy) * 0.0174533;
  imu_msg.angular_velocity.z = (imu.calcGyro(imu.gz) + 6) * 0.0174533;
  imu_msg.linear_acceleration.x = imu.calcAccel(imu.ax) * 9.80665;
  imu_msg.linear_acceleration.y = imu.calcAccel(imu.ay) * 9.80665;
  imu_msg.linear_acceleration.z = imu.calcAccel(imu.az) * 9.80665;
  imu_pub.publish(&imu_msg);
}

// 
void readUltrasonicSensors(){
  // Set the trigger to Low for a little while initially
  digitalWrite(FrontUltrasonicTrigger, LOW);
  delayMicroseconds(3);
  // Set the trigger to High for 10 microseconds according to the datasheet
  digitalWrite(FrontUltrasonicTrigger, HIGH);
  delayMicroseconds(10);
  // Set the trigger back to low
  digitalWrite(FrontUltrasonicTrigger, LOW);

  // Wait for the echo with a timeout of 23.28ms 
  front_ultrasonic_range_duration = pulseIn(FrontUltrasonicEcho, HIGH, 23280);
  
  // Find the distance in cm based on the duration
  // Values taken from the datasheet at https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
  front_ultrasonic_range_distance = (front_ultrasonic_range_duration / 29) / 2;

  // Set the trigger to Low for a little while initially
  digitalWrite(RightUltrasonicTrigger, LOW);
  delayMicroseconds(3);
  // Set the trigger to High for 10 microseconds according to the datasheet
  digitalWrite(RightUltrasonicTrigger, HIGH);
  delayMicroseconds(10);
  // Set the trigger back to low
  digitalWrite(RightUltrasonicTrigger, LOW);

  // Wait for the echo with a timeout of 23.28ms
  right_ultrasonic_range_duration = pulseIn(RightUltrasonicEcho, HIGH, 23280);
  
  // Find the distance in cm based on the duration
  // Values taken from the datasheet at https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
  right_ultrasonic_range_distance = (right_ultrasonic_range_duration / 29) / 2;
}

// Publish the current ultrasonic info from the front and right ultrasonic sensors to ROS
void publishUltrasonicInfo()
{
  front_ultrasonic_range_msg.range = front_ultrasonic_range_distance / 100;
  front_ultrasonic_range_msg.header.stamp = nh.now();
  right_ultrasonic_range_msg.range = right_ultrasonic_range_distance / 100;
  right_ultrasonic_range_msg.header.stamp = nh.now();
  current_x_position = (front_ultrasonic_range_distance / 100) + ultrasonic_sensor_offset_from_center;
  current_y_position = (right_ultrasonic_range_distance / 100) + ultrasonic_sensor_offset_from_center;
  ultrasonic_pose_msg.x = current_x_position;
  current_avg_x_position = (((current_avg_x_position * (avg_filter_size - 1)) + current_x_position) / avg_filter_size);
  ultrasonic_pose_msg.y = current_y_position;
  current_avg_y_position = (((current_avg_y_position * (avg_filter_size - 1)) + current_y_position) / avg_filter_size);
  front_ultrasonic_range_pub.publish(&front_ultrasonic_range_msg);
  right_ultrasonic_range_pub.publish(&right_ultrasonic_range_msg);
  ultrasonic_pose_pub.publish(&ultrasonic_pose_msg);
}

// Publish the current joint state positions of the turntable, x-gantry, and z-gantry to ROS
void publishJointStates()
{
  joint_states_msg.header.stamp = nh.now();
  joint_state_positions[0] = current_turntable_theta;
  joint_state_positions[1] = current_x_gantry_position;
  joint_state_positions[2] =  current_z_gantry_position;
  joint_states_msg.position = joint_state_positions;
  joint_states_pub.publish(&joint_states_msg);
}

// LOOP CODE
void loop()
{
  // If the robot is not currently in the stop mode
  if(!stop_flag)
  {
    if(determineHoming())
    {
      home();
    }
    else
    {
      if(move_x_gantry_flag)
      {
        moveXGantry();
      } 
      else if(move_z_gantry_flag)
      {
        moveZGantry();
      }
      else if(turn_turntable_flag)
      {
        turnTurntable();
      }
      else if(move_base_x_flag)
      {
        moveBaseX();
      }
      else if(move_base_y_flag)
      {
        moveBaseY();
      }
      else
      {
        //checkBasePosition();
      }
    }
  }
  
  updateTransform();
  publishTransform();

  updateIMU();
  publishIMU();

  readUltrasonicSensors();
  publishUltrasonicInfo();

  publishJointStates();

  nh.spinOnce();
  delay(3);
}

void checkBasePosition()
{
  if(abs(desired_x_position - current_avg_x_position) > avg_x_position_threshold)
  {
    move_base_x_flag = true;
  } 
  if(abs(desired_y_position - current_avg_y_position) > avg_y_position_threshold)
  {
    move_base_y_flag = true;
  } 
}

void moveBaseX()
{
  float error_x_position = desired_x_position - current_x_position;
  
  if(abs(error_x_position) < x_position_threshold)
  {
    digitalWrite(LeftMotorIn1, LOW);
    digitalWrite(LeftMotorIn2, LOW);  
    digitalWrite(RightMotorIn1, LOW);
    digitalWrite(RightMotorIn2, LOW); 
    move_base_x_flag = false;
    if(!move_base_y_flag)
    {
      done_moving_robot_base_msg.data = true;
      done_moving_robot_base_pub.publish(&done_moving_robot_base_msg);
      delay(10);
      done_moving_robot_base_pub.publish(&done_moving_robot_base_msg);
    }
  }
  else if(current_x_position < min_x_position)
  {
    digitalWrite(LeftMotorIn1, LOW);
    digitalWrite(LeftMotorIn2, LOW);  
    digitalWrite(RightMotorIn1, LOW);
    digitalWrite(RightMotorIn2, LOW); 
    move_base_x_flag = false;
    done_moving_robot_base_msg.data = false;
    done_moving_robot_base_pub.publish(&done_moving_robot_base_msg);
    delay(10);
    done_moving_robot_base_pub.publish(&done_moving_robot_base_msg);
  }
  else
  {
    long current_left_motor_encoder_count = left_motor_encoder_count;
    long current_right_motor_encoder_count = right_motor_encoder_count;

    long num_x_encoder_counts = (long)((error_x_position / distance_traveled_per_wheel_revolution) * encoder_counts_per_revolution);
    
    long target_left_motor_encoder_count = left_motor_encoder_count - num_x_encoder_counts;
    long target_right_motor_encoder_count = right_motor_encoder_count + num_x_encoder_counts;

    if (num_x_encoder_counts < 0)
    {
      // Going Forward
      digitalWrite(LeftMotorIn1, HIGH);
      digitalWrite(LeftMotorIn2, LOW);  
      digitalWrite(RightMotorIn1, LOW);
      digitalWrite(RightMotorIn2, HIGH);
      analogWrite(LeftMotorEnable, min_base_motor_speed);
      analogWrite(RightMotorEnable, min_base_motor_speed);
    }
    else
    {
      // Going Backward
      digitalWrite(LeftMotorIn1, LOW);
      digitalWrite(LeftMotorIn2, HIGH);  
      digitalWrite(RightMotorIn1, HIGH);
      digitalWrite(RightMotorIn2, LOW); 
      analogWrite(LeftMotorEnable, min_base_motor_speed);
      analogWrite(RightMotorEnable, min_base_motor_speed);
    }
  }
}

void moveBaseY()
{
  float error_y_position = desired_y_position - current_y_position;

  if(abs(error_y_position) < y_position_threshold)
  {
    digitalWrite(FrontMotorIn1, LOW);
    digitalWrite(FrontMotorIn2, LOW);  
    digitalWrite(BackMotorIn1, LOW);
    digitalWrite(BackMotorIn2, LOW);
    move_base_y_flag = false;
    if(!move_base_x_flag)
    {
      done_moving_robot_base_msg.data = true;
      done_moving_robot_base_pub.publish(&done_moving_robot_base_msg);
      delay(10);
      done_moving_robot_base_pub.publish(&done_moving_robot_base_msg);
    }
  }
  else if(current_y_position < min_y_position)
  {
    digitalWrite(FrontMotorIn1, LOW);
    digitalWrite(FrontMotorIn2, LOW);  
    digitalWrite(BackMotorIn1, LOW);
    digitalWrite(BackMotorIn2, LOW);
    move_base_y_flag = false;
    done_moving_robot_base_msg.data = false;
    done_moving_robot_base_pub.publish(&done_moving_robot_base_msg);
    delay(10);
    done_moving_robot_base_pub.publish(&done_moving_robot_base_msg);
  }
  else
  {
    long current_front_motor_encoder_count = front_motor_encoder_count;
    long current_back_motor_encoder_count = back_motor_encoder_count;

    long num_y_encoder_counts = (long)((error_y_position / distance_traveled_per_wheel_revolution) * encoder_counts_per_revolution);

    long target_front_motor_encoder_count = front_motor_encoder_count - num_y_encoder_counts;
    long target_back_motor_encoder_count = back_motor_encoder_count + num_y_encoder_counts;

    if(num_y_encoder_counts < 0)
    {
      // Going Right
      digitalWrite(FrontMotorIn1, HIGH);
      digitalWrite(FrontMotorIn2, LOW);  
      digitalWrite(BackMotorIn1, LOW);
      digitalWrite(BackMotorIn2, HIGH);
      analogWrite(FrontMotorEnable, min_base_motor_speed);
      analogWrite(BackMotorEnable, min_base_motor_speed);
    }
    else
    {
      // Going Left
      digitalWrite(FrontMotorIn1, LOW);
      digitalWrite(FrontMotorIn2, HIGH);  
      digitalWrite(BackMotorIn1, HIGH);
      digitalWrite(BackMotorIn2, LOW);
      analogWrite(FrontMotorEnable, min_base_motor_speed);
      analogWrite(BackMotorEnable, min_base_motor_speed);
    }
  }
}

void moveXGantry()
{
  long num_x_gantry_steps = (long)((desired_x_gantry_position - current_x_gantry_position) / x_gantry_distance_per_revolution) * x_gantry_steps_per_revolution;

  if(num_x_gantry_steps > 0)
  {
    // Move X Gantry Forward
    digitalWrite(XGantryStepperDirection, HIGH);

    for (long i = 0; i < min(x_gantry_step_interval, num_x_gantry_steps); i++)
    {         
      if(x_gantry_step_count >= max_x_gantry_steps)
      {
        current_x_gantry_position = (x_gantry_step_count / x_gantry_steps_per_revolution) * x_gantry_distance_per_revolution;
        move_x_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        delay(10);
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        return;
      }
      digitalWrite(XGantryStepperPulse, HIGH);
      digitalWrite(XGantryStepperPulse, LOW);
      x_gantry_step_count++;

      delayMicroseconds(x_gantry_step_time);
    }
  }
  else
  {
    // Move X Gantry Back
    digitalWrite(XGantryStepperDirection, LOW);

    for (long i = 0; i < min(x_gantry_step_interval, -num_x_gantry_steps); i++)
    {         
      if(x_gantry_step_count == 0)
      {
        current_x_gantry_position = 0.0;
        move_x_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        delay(10);
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        return;
      }
      digitalWrite(XGantryStepperPulse, HIGH);
      digitalWrite(XGantryStepperPulse, LOW);
      x_gantry_step_count--;

      delayMicroseconds(x_gantry_step_time);
    }
  }
  current_x_gantry_position = (x_gantry_step_count / x_gantry_steps_per_revolution) * x_gantry_distance_per_revolution;

  if(abs(current_x_gantry_position - desired_x_gantry_position) <= x_gantry_threshold)
  {
    move_x_gantry_flag = false;
    if(!move_z_gantry_flag)
    {
      done_moving_gantry_msg.data = true;
      done_moving_gantry_pub.publish(&done_moving_gantry_msg);
      delay(10);
      done_moving_gantry_pub.publish(&done_moving_gantry_msg);
    }
  }
}

void moveZGantry()
{
  long num_z_gantry_steps = (long)((desired_z_gantry_position - current_z_gantry_position) / z_gantry_distance_per_revolution) * z_gantry_steps_per_revolution;

  if(num_z_gantry_steps > 0)
  {
    // Move Z Gantry Up
    digitalWrite(ZGantryStepperDirection, LOW);

    for (long i = 0; i < min(z_gantry_step_interval, num_z_gantry_steps); i++)
    {         
      if(z_gantry_step_count >= max_z_gantry_steps)
      {
        current_z_gantry_position = (z_gantry_step_count / z_gantry_steps_per_revolution) * z_gantry_distance_per_revolution;
        move_z_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        delay(10);
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        return;
      }
      digitalWrite(ZGantryStepperPulse, HIGH);
      digitalWrite(ZGantryStepperPulse, LOW);
      z_gantry_step_count++;

      delayMicroseconds(z_gantry_step_time);
    }
  }
  else
  {
    // Move Z Gantry Down
    digitalWrite(ZGantryStepperDirection, HIGH);

    for (long i = 0; i < min(z_gantry_step_interval, -num_z_gantry_steps); i++)
    {         
      if(z_gantry_step_count == 0)
      {
        current_z_gantry_position = 0.0;
        move_z_gantry_flag = false;
        done_moving_gantry_msg.data = false;
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        delay(10);
        done_moving_gantry_pub.publish(&done_moving_gantry_msg);
        return;
      }
      digitalWrite(ZGantryStepperPulse, HIGH);
      digitalWrite(ZGantryStepperPulse, LOW);
      z_gantry_step_count--;

      delayMicroseconds(z_gantry_step_time);
    }
  }
  current_z_gantry_position = (z_gantry_step_count / z_gantry_steps_per_revolution) * z_gantry_distance_per_revolution;

  if(abs(current_z_gantry_position - desired_z_gantry_position) <= z_gantry_threshold)
  {
    move_z_gantry_flag = false;
    if(!move_x_gantry_flag)
    {
      done_moving_gantry_msg.data = true;
      done_moving_gantry_pub.publish(&done_moving_gantry_msg);
      delay(10);
      done_moving_gantry_pub.publish(&done_moving_gantry_msg);
    }
  }
}

void turnTurntable()
{
  int num_turntable_steps = (int)(((desired_turntable_theta - current_turntable_theta) / (2 * PI)) * turntable_steps_per_revolution);

  if(num_turntable_steps == 0)
  {
    turn_turntable_flag = false;
    done_turning_turntable_msg.data = true;
    done_turning_turntable_pub.publish(&done_turning_turntable_msg);
    delay(10);
    done_turning_turntable_pub.publish(&done_turning_turntable_msg);
    return;
  }
  else if(num_turntable_steps > 0)
  {
    // Turn Clockwise
    digitalWrite(TurntableStepperDirection, HIGH);

    for (int i = 1; i < min(turntable_step_interval, num_turntable_steps); i++)
    {
      if(current_turntable_step_count >= max_turntable_steps)
      {
        current_turntable_theta = (((float)current_turntable_step_count) / turntable_steps_per_revolution) * 2 * PI;
        turn_turntable_flag = false;
        done_turning_turntable_msg.data = false;
        done_turning_turntable_pub.publish(&done_turning_turntable_msg);
        delay(10);
        done_turning_turntable_pub.publish(&done_turning_turntable_msg);
        return;
      }

      digitalWrite(TurntableStepperPulse, HIGH);
      digitalWrite(TurntableStepperPulse, LOW);
      current_turntable_step_count++;

      delayMicroseconds(turntable_step_time);
    }
  }
  else
  {
    // Turn Counter Clockwise
    digitalWrite(TurntableStepperDirection, LOW);

    for (int i = 1; i < min(turntable_step_interval, -num_turntable_steps); i++)
    {
      if(current_turntable_step_count <= min_turntable_steps)
      {
        current_turntable_theta = (((float)current_turntable_step_count) / turntable_steps_per_revolution) * 2 * PI;
        turn_turntable_flag = false;
        done_turning_turntable_msg.data = false;
        done_turning_turntable_pub.publish(&done_turning_turntable_msg);
        delay(10);
        done_turning_turntable_pub.publish(&done_turning_turntable_msg);
        return;
      }

      digitalWrite(TurntableStepperPulse, HIGH);
      digitalWrite(TurntableStepperPulse, LOW);
      current_turntable_step_count--;

      delayMicroseconds(turntable_step_time);
    }
  }

  current_turntable_theta = (((float)current_turntable_step_count) / turntable_steps_per_revolution) * 2 * PI;

  if(abs(current_turntable_theta - desired_turntable_theta) <= turntable_threshold)
  {
    turn_turntable_flag = false;
    done_turning_turntable_msg.data = true;
    done_turning_turntable_pub.publish(&done_turning_turntable_msg);
    delay(10);
    done_turning_turntable_pub.publish(&done_turning_turntable_msg);
  }
}
