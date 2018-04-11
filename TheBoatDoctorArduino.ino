// ARDUINO MEGA PIN DEFINITIONS
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

// ROS SERIAL INCLUDES
#include <ros.h> // ROS Serial
#include <ros/time.h> // ROS time for publishing messages
#include <tf/transform_broadcaster.h> // Transform Broadcaster for TF
#include <std_msgs/Empty.h> // Empty msg for stopping everything
#include <std_msgs/Bool.h> // Bool msg for Pump and LED switches
#include <geometry_msgs/Pose2D.h> // Pose2D msg for moving the robot base and turntable
#include <geometry_msgs/Twist.h> // Twist msg for cmd_vel
#include <sensor_msgs/Range.h> // Sensor_msgs Range message for Ultrasonic Sensors
#include <sensor_msgs/Imu.h> // Sensor_msgs Imu message for IMU

// IMU INCLUDES
#include <SPI.h> // SPI library included for SparkFunLSM9DS1
#include <Wire.h> // I2C library included for SparkFunLSM9DS1
#include <SparkFunLSM9DS1.h> // SparkFun LSM9DS1 library

// IMU DEFINES
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

// ROS Globals
ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

sensor_msgs::Range front_ultrasonic_range_msg;
sensor_msgs::Range right_ultrasonic_range_msg;
ros::Publisher front_ultrasonic_range_pub( "/TheBoatDoctor/front/ultrasonic", &front_ultrasonic_range_msg);
ros::Publisher right_ultrasonic_range_pub( "/TheBoatDoctor/right/ultrasonic", &right_ultrasonic_range_msg);

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub( "/TheBoatDoctor/imu", &imu_msg);

char base_link[] = "/base_link";
char odom[] = "/odom";

// Motor Encoder Globals
volatile unsigned int front_motor_encoder_count = 0;
volatile unsigned int left_motor_encoder_count = 0;
volatile unsigned int back_motor_encoder_count = 0;
volatile unsigned int right_motor_encoder_count = 0;
const float wheel_diameter = 4.0;
const int gear_ratio = 131;
const float motor_rpm = 80.0;
const float distance_between_wheels = 0.33; // Guessing that there is around 0.33m (13") between each pair of wheels
const float distance_traveled_per_wheel_revolution = wheel_diameter * PI; // m
const float max_motor_speed = distance_traveled_per_wheel_revolution * motor_rpm / 60.0; // m/s
const int encoder_counts_per_revolution = (64 / 2) * gear_ratio; // 64 CPR motor encoder, but only using an interrupt for channel A

// Stepper Motor Globals
float turntable_theta = 0.0;
int turntable_step_count = 0;
const int turntable_steps_per_revolution = 6400;
float x_gantry_position;
int x_gantry_step_count;
const int x_gantry_steps_per_revolution = 1600;
float z_gantry_position;
int z_gantry_step_count;
const int z_gantry_steps_per_revolution = 1600;

// IMU Globals
// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 9.20 // Declination (degrees) in Pittsburgh, PA.

LSM9DS1 imu;
float heading;

// Ultrasonic Sensor Globals
long front_ultrasonic_range_duration;
long right_ultrasonic_range_duration;
float front_ultrasonic_range_distance;
float right_ultrasonic_range_distance;
char front_ultrasonic_frameid[] = "/front_ultrasonic";
char right_ultrasonic_frameid[] = "/right_ultrasonic";

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
}

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
}

// Stop Command Callback
// Immediately halts everything by setting all the motor control pins to LOW
void stopCallback(const std_msgs::Empty& stop_msg){
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
  
  digitalWrite(PumpSwitch, LOW);
  digitalWrite(LEDSwitch, LOW);
}

// Home Command Callback
// Returns the robot to the home position
void homeCallback(const std_msgs::Empty& stop_msg){
  zeroZGantry();
  zeroXGantry();
  homeTurntable();
}

void cmdVelCallback(const geometry_msgs::Twist& twist_msg)
{

}

void pose2DCallback(const geometry_msgs::Pose2D& pose_2d_msg)
{
  float current_x = front_ultrasonic_range_distance;
  float current_y = right_ultrasonic_range_distance;
  float current_theta = turntable_theta;

  float desired_x = pose_2d_msg.x;
  float desired_y = pose_2d_msg.y;
  float desired_theta = pose_2d_msg.theta;

  moveBase(desired_x - current_x, desired_y - current_y);
  moveTurntable(desired_theta - current_theta);
}

ros::Subscriber<std_msgs::Bool> led_switch_sub("/TheBoatDoctor/LED_Switch", &ledSwitchCallback );
ros::Subscriber<std_msgs::Bool> pump_switch_sub("/TheBoatDoctor/Pump_Switch", &pumpSwitchCallback );
ros::Subscriber<std_msgs::Empty> stop_sub("/TheBoatDoctor/Stop", &stopCallback );
ros::Subscriber<std_msgs::Empty> home_sub("/TheBoatDoctor/Home", &homeCallback );
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/TheBoatDoctor/cmd_vel", cmdVelCallback);
ros::Subscriber<geometry_msgs::Pose2D> pose_2d_sub("/TheBoatDoctor/Pose2D", pose2DCallback);

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
  Serial.println(front_motor_encoder_count);
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
  Serial.println(left_motor_encoder_count);
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
  Serial.println(back_motor_encoder_count);
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
  Serial.println(right_motor_encoder_count);
}

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

  // attach interrupts to all motor encoder hall A pins
  attachInterrupt(1, front_motor_hallA_detect, CHANGE); //Initialize the interrupt pin digitalPinToInterrupt(FrontMotorEncoderHallA) = 1
  attachInterrupt(0, left_motor_hallA_detect, CHANGE); //Initialize the interrupt pin digitalPinToInterrupt(LeftMotorEncoderHallA) = 0
  attachInterrupt(5, back_motor_hallA_detect, CHANGE); //Initialize the interrupt pin digitalPinToInterrupt(BackMotorEncoderHallA) = 5
  attachInterrupt(4, right_motor_hallA_detect, CHANGE); //Initialize the interrupt pin digitalPinToInterrupt(RightMotorEncoderHallA) = 4

  // IMU setup and startup code
  imu.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C
  imu.settings.device.mAddress = LSM9DS1_M; // Set mag address to 0x1E
  imu.settings.device.agAddress = LSM9DS1_AG; // Set ag address to 0x6B
  imu.settings.accel.scale = 8; // Set accel range to +/-8g
  imu.settings.gyro.scale = 2000; // Set gyro range to +/-2000dps
  imu.settings.mag.scale = 4; // Set mag range to +/-4Gs
  if (!imu.begin())
  {
      Serial.println("Failed to communicate with LSM9DS1.");
      Serial.println("Looping to infinity.");
      while (1)
        ;
  }

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
  nh.advertise(imu_pub);

  // Subscribe to topics
  nh.subscribe(led_switch_sub);
  nh.subscribe(pump_switch_sub);
  nh.subscribe(stop_sub);

  // Ultrasonic Sensor Setup Code
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

  // IMU Setup Code

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

  zeroXGantry();
  zeroZGantry();
}

void homeTurntable()
{
  if(turntable_step_count > 0)
  {
    digitalWrite(TurntableStepperDirection, LOW);
    for (int i = 1; i < turntable_step_count; i++)
    {
     digitalWrite(TurntableStepperPulse, HIGH);
     digitalWrite(TurntableStepperPulse, LOW);

     delayMicroseconds(10000);
    }
  }
  else
  {
    digitalWrite(TurntableStepperDirection, HIGH);
    for (int i = 1; i < -turntable_step_count; i++)
    {
     digitalWrite(TurntableStepperPulse, HIGH);
     digitalWrite(TurntableStepperPulse, LOW);

     delayMicroseconds(10000);
    }
  }
  turntable_theta = 0.0;
  turntable_step_count = 0;
}

void zeroXGantry()
{
  digitalWrite(XGantryStepperDirection, LOW);
  while (digitalRead(XAxisLimitSwitch) != 0)
  {         
   digitalWrite(XGantryStepperPulse, HIGH);
   digitalWrite(XGantryStepperPulse, LOW);
   
   delayMicroseconds(1000);
  }
  digitalWrite(XGantryStepperDirection, HIGH);
  while (digitalRead(XAxisLimitSwitch) != 1)
  {
   digitalWrite(XGantryStepperPulse, HIGH);
   digitalWrite(XGantryStepperPulse, LOW);
   
   delayMicroseconds(1000);
  }
  for (int i = 1; i < 300; i++)
  {
   digitalWrite(XGantryStepperPulse, HIGH);
   digitalWrite(XGantryStepperPulse, LOW);

   delayMicroseconds(1000);
  }
  x_gantry_position = 0.0;
  x_gantry_step_count = 0;
}

void zeroZGantry()
{
  digitalWrite(ZGantryStepperDirection, LOW);
  while (digitalRead(ZAxisLimitSwitch) != 0)
  {         
   digitalWrite(ZGantryStepperPulse, HIGH);
   digitalWrite(ZGantryStepperPulse, LOW);
   
   delayMicroseconds(1000);
  }
  digitalWrite(ZGantryStepperDirection, HIGH);
  while (digitalRead(ZAxisLimitSwitch) != 1)
  {
   digitalWrite(ZGantryStepperPulse, HIGH);
   digitalWrite(ZGantryStepperPulse, LOW);
   
   delayMicroseconds(1000);
  }
  for (int i = 1; i < 100; i++)
  {
   digitalWrite(ZGantryStepperPulse, HIGH);
   digitalWrite(ZGantryStepperPulse, LOW);

   delayMicroseconds(1000);
  }
  z_gantry_position = 0.0;
  z_gantry_step_count = 0;
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

void moveBase(float x_dist, float y_dist)
{
  int current_front_motor_encoder_count = front_motor_encoder_count;
  int current_left_motor_encoder_count = left_motor_encoder_count;
  int current_back_motor_encoder_count = back_motor_encoder_count;
  int current_right_motor_encoder_count = right_motor_encoder_count;

  int num_x_encoder_counts = (x_dist / distance_traveled_per_wheel_revolution) * encoder_counts_per_revolution;
  int num_y_encoder_counts = (y_dist / distance_traveled_per_wheel_revolution) * encoder_counts_per_revolution;

  int target_front_motor_encoder_count = front_motor_encoder_count + num_y_encoder_counts;
  int target_left_motor_encoder_count = left_motor_encoder_count + num_x_encoder_counts;
  int target_back_motor_encoder_count = back_motor_encoder_count + num_y_encoder_counts;
  int target_right_motor_encoder_count = right_motor_encoder_count + num_x_encoder_counts;

  if(num_x_encoder_counts < 0)
  {
    // Going Forwards
    digitalWrite(LeftMotorIn1, LOW);
    digitalWrite(LeftMotorIn2, HIGH);  
    digitalWrite(RightMotorIn1, HIGH);
    digitalWrite(RightMotorIn2, LOW); 

    int motor_speed = 0;
    while(left_motor_encoder_count > target_left_motor_encoder_count && 
          right_motor_encoder_count > target_right_motor_encoder_count &&
          motor_speed < 128)
    {
      analogWrite(LeftMotorEnable, motor_speed);
      analogWrite(RightMotorEnable, motor_speed);
      motor_speed++;
      delay(20);
    }
    while(left_motor_encoder_count > target_left_motor_encoder_count && 
          right_motor_encoder_count > target_right_motor_encoder_count)
    {
      delay(20);
    }
    while(motor_speed > 0)
    {
      analogWrite(LeftMotorEnable, motor_speed);
      analogWrite(RightMotorEnable, motor_speed);
      motor_speed--;
      delay(20);
    }
  }
  else
  {
    // Going Backwards
    digitalWrite(LeftMotorIn1, HIGH);
    digitalWrite(LeftMotorIn2, LOW);  
    digitalWrite(RightMotorIn1, LOW);
    digitalWrite(RightMotorIn2, HIGH); 

    int motor_speed = 0;
    while(left_motor_encoder_count < target_left_motor_encoder_count && 
          right_motor_encoder_count < target_right_motor_encoder_count &&
          motor_speed < 128)
    {
      analogWrite(LeftMotorEnable, motor_speed);
      analogWrite(RightMotorEnable, motor_speed);
      motor_speed++;
      delay(20);
    }
    while(left_motor_encoder_count < target_left_motor_encoder_count && 
          right_motor_encoder_count < target_right_motor_encoder_count)
    {
      delay(20);
    }
    while(motor_speed > 0)
    {
      analogWrite(LeftMotorEnable, motor_speed);
      analogWrite(RightMotorEnable, motor_speed);
      motor_speed--;
      delay(20);
    } 
  }
  if(num_y_encoder_counts > 0)
  {
    // Going Left
    digitalWrite(FrontMotorIn1, HIGH);
    digitalWrite(FrontMotorIn2, LOW);  
    digitalWrite(BackMotorIn1, LOW);
    digitalWrite(BackMotorIn2, HIGH);

    int motor_speed = 0;
    while(front_motor_encoder_count > target_front_motor_encoder_count && 
          back_motor_encoder_count > target_back_motor_encoder_count &&
          motor_speed < 128)
    {
      analogWrite(FrontMotorEnable, motor_speed);
      analogWrite(BackMotorEnable, motor_speed);
      motor_speed++;
      delay(20);
    }
    while(front_motor_encoder_count > target_front_motor_encoder_count && 
          back_motor_encoder_count > target_back_motor_encoder_count)
    {
      delay(20);
    }
    while(motor_speed > 0)
    {
      analogWrite(FrontMotorEnable, motor_speed);
      analogWrite(BackMotorEnable, motor_speed);
      motor_speed--;
      delay(20);
    }
  }
  else
  {
    // Going Right
    digitalWrite(FrontMotorIn1, LOW);
    digitalWrite(FrontMotorIn2, HIGH);  
    digitalWrite(BackMotorIn1, HIGH);
    digitalWrite(BackMotorIn2, LOW);

    int motor_speed = 0;
    while(front_motor_encoder_count < target_front_motor_encoder_count && 
          back_motor_encoder_count < target_back_motor_encoder_count &&
          motor_speed < 128)
    {
      analogWrite(FrontMotorEnable, motor_speed);
      analogWrite(BackMotorEnable, motor_speed);
      motor_speed++;
      delay(20);
    }
    while(front_motor_encoder_count < target_front_motor_encoder_count && 
          back_motor_encoder_count < target_back_motor_encoder_count)
    {
      delay(20);
    }
    while(motor_speed > 0)
    {
      analogWrite(FrontMotorEnable, motor_speed);
      analogWrite(BackMotorEnable, motor_speed);
      motor_speed--;
      delay(20);
    }
  }
}

void moveTurntable(float theta)
{
  if(theta > 0)
  {
    int num_turntable_steps = (theta / (2 * PI)) * turntable_steps_per_revolution;
    digitalWrite(TurntableStepperDirection, HIGH);
    for (int i = 1; i < num_turntable_steps; i++)
    {
     digitalWrite(TurntableStepperPulse, HIGH);
     digitalWrite(TurntableStepperPulse, LOW);
     turntable_step_count++;

     delayMicroseconds(10000);
    }
  }
  else
  {
    int num_turntable_steps = (-theta / (2 * PI)) * turntable_steps_per_revolution;
    digitalWrite(TurntableStepperDirection, LOW);
    for (int i = 1; i < num_turntable_steps; i++)
    {
     digitalWrite(TurntableStepperPulse, HIGH);
     digitalWrite(TurntableStepperPulse, LOW);
     turntable_step_count--;

     delayMicroseconds(10000);
    }
  }
  turntable_theta = (turntable_step_count / turntable_steps_per_revolution) * 2 * PI;
}

//void serialCommand()
//{
//  if(Serial.available() > 0) {
//    int command = Serial.read();
//
//    switch (command) {
//      case 'f':
//        if(prevCommand != command)
//        {
//          stopPrevCommand();
//          digitalWrite(FrontMotorIn1, LOW);
//          digitalWrite(LeftMotorIn2, HIGH);  
//          digitalWrite(RightMotorIn1, HIGH);
//          digitalWrite(RightMotorIn2, LOW); 
//          // accelerate from zero to maximum speed
//          for (int i = 0; i < 128; i++)
//          {
//            analogWrite(LeftMotorEnable, i);
//            analogWrite(RightMotorEnable, i);
//            delay(20);
//          } 
//        }
//        break;
//      case 'b':
//        if(prevCommand != command)
//        {
//          stopPrevCommand();
//          digitalWrite(LeftMotorIn1, HIGH);
//          digitalWrite(LeftMotorIn2, LOW);  
//          digitalWrite(RightMotorIn1, LOW);
//          digitalWrite(RightMotorIn2, HIGH); 
//          // accelerate from zero to maximum speed
//          for (int i = 0; i < 128; i++)
//          {
//            analogWrite(LeftMotorEnable, i);
//            analogWrite(RightMotorEnable, i);
//            delay(20);
//          } 
//        }
//        break;
//      case 'l':
//        if(prevCommand != command)
//        {
//          stopPrevCommand();
//          digitalWrite(FrontMotorIn1, HIGH);
//          digitalWrite(FrontMotorIn2, LOW);  
//          digitalWrite(BackMotorIn1, LOW);
//          digitalWrite(BackMotorIn2, HIGH); 
//          // accelerate from zero to maximum speed
//          for (int i = 0; i < 128; i++)
//          {
//            analogWrite(FrontMotorEnable, i);
//            analogWrite(BackMotorEnable, i);
//            delay(20);
//          } 
//        }
//        break;
//      case 'r':
//        if(prevCommand != command)
//        {
//          stopPrevCommand();
//          digitalWrite(FrontMotorIn1, LOW);
//          digitalWrite(FrontMotorIn2, HIGH);  
//          digitalWrite(BackMotorIn1, HIGH);
//          digitalWrite(BackMotorIn2, LOW); 
//          // accelerate from zero to maximum speed
//          for (int i = 0; i < 128; i++)
//          {
//            analogWrite(FrontMotorEnable, i);
//            analogWrite(BackMotorEnable, i);
//            delay(20);
//          } 
//        break;
//        }
//      case 'p':
//        digitalWrite(PumpSwitch, HIGH);
//        break;
//      case 'o':
//        digitalWrite(PumpSwitch, LOW);
//        break;
//      case 'k':
//        digitalWrite(LEDSwitch, HIGH);
//        break;
//      case 'j':
//        digitalWrite(LEDSwitch, LOW);
//        break;
//      case 'w':
//        digitalWrite(ZGantryStepperDirection, LOW);
//        for (int i = 0; i < 3200; i++)
//        { 
//          digitalWrite(ZGantryStepperPulse, HIGH);
//          digitalWrite(ZGantryStepperPulse, LOW);
//          
//          delayMicroseconds(300);
//        }
//        break;
//      case 's':
//        digitalWrite(ZGantryStepperDirection, HIGH);
//        for (int i = 0; i < 3200; i++)
//        {         
//          digitalWrite(ZGantryStepperPulse, HIGH);
//          digitalWrite(ZGantryStepperPulse, LOW);
//          
//          delayMicroseconds(300);
//        }
//        break;
//      case 'a':
//        digitalWrite(XGantryStepperDirection, LOW);
//        for (int i = 0; i < 3200; i++)
//        {         
//          if(digitalRead(XAxisLimitSwitch) == 0)
//          {
//            digitalWrite(XGantryStepperDirection, HIGH);
//            while (digitalRead(XAxisLimitSwitch) != 1)
//            {
//              digitalWrite(XGantryStepperPulse, HIGH);
//              digitalWrite(XGantryStepperPulse, LOW);
//              
//              delayMicroseconds(300);
//            }
//            for (int i = 1; i < 300; i++)
//            {
//              digitalWrite(XGantryStepperPulse, HIGH);
//              digitalWrite(XGantryStepperPulse, LOW);
//    
//              delayMicroseconds(300);
//            }
//            break;
//          }
//          digitalWrite(XGantryStepperPulse, HIGH);
//          digitalWrite(XGantryStepperPulse, LOW);
//          
//          delayMicroseconds(300);
//        }
//        break;
//      case 'd':
//        digitalWrite(XGantryStepperDirection, HIGH);
//        for (int i = 0; i < 3200; i++)
//        {         
//          digitalWrite(XGantryStepperPulse, HIGH);
//          digitalWrite(XGantryStepperPulse, LOW);
//          
//          delayMicroseconds(300);
//        }
//        break;
//      case 'z':
//        digitalWrite(ZGantryStepperDirection, HIGH);
//        while (digitalRead(ZAxisLimitSwitch) != 0)
//        {         
//          digitalWrite(ZGantryStepperPulse, HIGH);
//          digitalWrite(ZGantryStepperPulse, LOW);
//          
//          delayMicroseconds(1000);
//        }
//        break;
//      case 'x':
//        digitalWrite(XGantryStepperDirection, LOW);
//        while (digitalRead(XAxisLimitSwitch) != 0)
//        {         
//          digitalWrite(XGantryStepperPulse, HIGH);
//          digitalWrite(XGantryStepperPulse, LOW);
//          
//          delayMicroseconds(1000);
//        }
//        digitalWrite(XGantryStepperDirection, HIGH);
//        while (digitalRead(XAxisLimitSwitch) != 1)
//        {
//          digitalWrite(XGantryStepperPulse, HIGH);
//          digitalWrite(XGantryStepperPulse, LOW);
//          
//          delayMicroseconds(1000);
//        }
//        for (int i = 1; i < 300; i++)
//        {
//          digitalWrite(XGantryStepperPulse, HIGH);
//          digitalWrite(XGantryStepperPulse, LOW);
//
//          delayMicroseconds(1000);
//        }
//        break;
//      case 'v':
//        digitalWrite(TurntableStepperDirection, LOW);
//        for (int i = 0; i < 12800; i++)
//        {         
//          digitalWrite(TurntableStepperPulse, HIGH);
//          digitalWrite(TurntableStepperPulse, LOW);
//          
//          delayMicroseconds(300);
//        }
//        break;
//      case 'c':
//        digitalWrite(TurntableStepperDirection, HIGH);
//        for (int i = 0; i < 12800; i++)
//        {         
//          digitalWrite(TurntableStepperPulse, HIGH);
//          digitalWrite(TurntableStepperPulse, LOW);
//          
//          delayMicroseconds(300);
//        }
//        break;
//
//      default:
//        stopPrevCommand();
//        digitalWrite(FrontMotorIn1, LOW);
//        digitalWrite(FrontMotorIn2, LOW);
//        digitalWrite(LeftMotorIn1, LOW);
//        digitalWrite(LeftMotorIn2, LOW); 
//        digitalWrite(RightMotorIn1, LOW);
//        digitalWrite(RightMotorIn2, LOW);
//        digitalWrite(BackMotorIn1, LOW);
//        digitalWrite(BackMotorIn2, LOW);
//        digitalWrite(PumpSwitch, LOW);
//        digitalWrite(LEDSwitch, LOW);
//    }
//    prevCommand = command;
//  }
//  readUltrasonicSensors();
//}
//
//void stopPrevCommand()
//{
//  switch (prevCommand) {
//    case 'f':
//    case 'b':
//      for (int i = 127; i >= 0; --i)
//      {
//        analogWrite(LeftMotorEnable, i);
//        analogWrite(RightMotorEnable, i);
//        delay(20);
//      } 
//      digitalWrite(LeftMotorIn1, LOW);
//      digitalWrite(LeftMotorIn2, LOW);  
//      digitalWrite(RightMotorIn1, LOW);
//      digitalWrite(RightMotorIn2, LOW);  
//      break;
//    case 'l':
//    case 'r':
//      for (int i = 127; i >= 0; --i)
//      {
//        analogWrite(FrontMotorEnable, i);
//        analogWrite(BackMotorEnable, i);
//        delay(20);
//      } 
//      digitalWrite(FrontMotorIn1, LOW);
//      digitalWrite(FrontMotorIn2, LOW);  
//      digitalWrite(BackMotorIn1, LOW);
//      digitalWrite(BackMotorIn2, LOW);  
//      break;
//    default:
//      digitalWrite(FrontMotorIn1, LOW);
//      digitalWrite(FrontMotorIn2, LOW);
//      digitalWrite(LeftMotorIn1, LOW);
//      digitalWrite(LeftMotorIn2, LOW); 
//      digitalWrite(RightMotorIn1, LOW);
//      digitalWrite(RightMotorIn2, LOW);
//      digitalWrite(BackMotorIn1, LOW);
//      digitalWrite(BackMotorIn2, LOW);
//    }
//}

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
  front_ultrasonic_range_distance = (front_ultrasonic_range_duration / 2) / 29.1;

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
  right_ultrasonic_range_distance = (right_ultrasonic_range_duration / 2) / 29.1;
}

void publishUltrasonicRangeMsgs()
{
  front_ultrasonic_range_msg.range = front_ultrasonic_range_distance / 100;
  front_ultrasonic_range_msg.header.stamp = nh.now();
  right_ultrasonic_range_msg.range = right_ultrasonic_range_distance / 100;
  right_ultrasonic_range_msg.header.stamp = nh.now();
  front_ultrasonic_range_pub.publish(&front_ultrasonic_range_msg);
  right_ultrasonic_range_pub.publish(&right_ultrasonic_range_msg);
}

void loop()
{
  updateTransform();
  publishTransform();

  updateIMU();
  publishIMU();

  readUltrasonicSensors();
  publishUltrasonicRangeMsgs();

  nh.spinOnce();
  delay(10);
}
