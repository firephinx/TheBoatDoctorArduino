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
#include <sensor_msgs/Range.h> // Sensor_msgs Range message for Ultrasonic Sensors

// IMU INCLUDES
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

char base_link[] = "/base_link";
char odom[] = "/odom";

// IMU Globals
LSM9DS1 imu;

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
void ledSwitchCb( const std_msgs::Bool& led_switch_msg){
  // If the led_switch_msg contains true, turn on the LEDs
  if(led_switch_msg->data)
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
void pumpSwitchCb( const std_msgs::Bool& pump_switch_msg){
  // If the pump_switch_msg contains true, turn on the pump
  if(pump_switch_msg->data)
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
void stopCb( const std_msgs::Empty& stop_msg){
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

ros::Subscriber<std_msgs::Bool> led_switch_sub("/TheBoatDoctor/LED_Switch", &ledSwitchCb );
ros::Subscriber<std_msgs::Bool> pump_switch_sub("/TheBoatDoctor/Pump_Switch", &pumpSwitchCb );
ros::Subscriber<std_msgs::Empty> stop_sub("/TheBoatDoctor/Stop", &stopCb );

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
  // attach interrupts to all motor encoder hall A pins
//  attachInterrupt(digitalPinToInterrupt(FrontMotorEncoderHallA), front_motor_hallA_detect, CHANGE); //Initialize the interrupt pin
//  attachInterrupt(digitalPinToInterrupt(LeftMotorEncoderHallA), left_motor_hallA_detect, CHANGE); //Initialize the interrupt pin
//  attachInterrupt(digitalPinToInterrupt(BackMotorEncoderHallA), back_motor_hallA_detect, CHANGE); //Initialize the interrupt pin
//  attachInterrupt(digitalPinToInterrupt(RightMotorEncoderHallA), right_motor_hallA_detect, CHANGE); //Initialize the interrupt pin

  // set all the motor encoder hall B pins to inputs
  pinMode(FrontMotorEncoderHallB, INPUT);
  pinMode(LeftMotorEncoderHallB, INPUT);
  pinMode(BackMotorEncoderHallB, INPUT);
  pinMode(RightMotorEncoderHallB, INPUT);

  // IMU setup and startup code
  imu.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C
  imu.settings.device.mAddress = LSM9DS1_M; // Set mag address to 0x1E
  imu.settings.device.agAddress = LSM9DS1_AG; // Set ag address to 0x6B
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

  // Subscribe to topics
  nh.subscribe(led_switch_sub);
  nh.subscribe(pump_switch_sub);
  nh.subscribe(stop_sub);

  // Ultrasonic Sensor Setup Code
  front_ultrasonic_range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  front_ultrasonic_range_msg.header.frame_id = front_ultrasonic_frameid;
  front_ultrasonic_range_msg.field_of_view = 0.1;  // fake
  front_ultrasonic_range_msg.min_range = 0.0;
  front_ultrasonic_range_msg.max_range = 6.47;

  right_ultrasonic_range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  right_ultrasonic_range_msg.header.frame_id = right_ultrasonic_frameid;
  right_ultrasonic_range_msg.field_of_view = 0.1;  // fake
  right_ultrasonic_range_msg.min_range = 0.0;
  right_ultrasonic_range_msg.max_range = 6.47;

}

void updateTransform()
{
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = 1.0; 
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

void serialCommand()
{
  if(Serial.available() > 0) {
    int command = Serial.read();

    switch (command) {
      case 'f':
        if(prevCommand != command)
        {
          stopPrevCommand();
          digitalWrite(LeftMotorIn1, LOW);
          digitalWrite(LeftMotorIn2, HIGH);  
          digitalWrite(RightMotorIn1, HIGH);
          digitalWrite(RightMotorIn2, LOW); 
          // accelerate from zero to maximum speed
          for (int i = 0; i < 128; i++)
          {
            analogWrite(LeftMotorEnable, i);
            analogWrite(RightMotorEnable, i);
            delay(20);
          } 
        }
        break;
      case 'b':
        if(prevCommand != command)
        {
          stopPrevCommand();
          digitalWrite(LeftMotorIn1, HIGH);
          digitalWrite(LeftMotorIn2, LOW);  
          digitalWrite(RightMotorIn1, LOW);
          digitalWrite(RightMotorIn2, HIGH); 
          // accelerate from zero to maximum speed
          for (int i = 0; i < 128; i++)
          {
            analogWrite(LeftMotorEnable, i);
            analogWrite(RightMotorEnable, i);
            delay(20);
          } 
        }
        break;
      case 'l':
        if(prevCommand != command)
        {
          stopPrevCommand();
          digitalWrite(FrontMotorIn1, HIGH);
          digitalWrite(FrontMotorIn2, LOW);  
          digitalWrite(BackMotorIn1, LOW);
          digitalWrite(BackMotorIn2, HIGH); 
          // accelerate from zero to maximum speed
          for (int i = 0; i < 128; i++)
          {
            analogWrite(FrontMotorEnable, i);
            analogWrite(BackMotorEnable, i);
            delay(20);
          } 
        }
        break;
      case 'r':
        if(prevCommand != command)
        {
          stopPrevCommand();
          digitalWrite(FrontMotorIn1, LOW);
          digitalWrite(FrontMotorIn2, HIGH);  
          digitalWrite(BackMotorIn1, HIGH);
          digitalWrite(BackMotorIn2, LOW); 
          // accelerate from zero to maximum speed
          for (int i = 0; i < 128; i++)
          {
            analogWrite(FrontMotorEnable, i);
            analogWrite(BackMotorEnable, i);
            delay(20);
          } 
        break;
        }
      case 'p':
        digitalWrite(PumpSwitch, HIGH);
        break;
      case 'o':
        digitalWrite(PumpSwitch, LOW);
        break;
      case 'k':
        digitalWrite(LEDSwitch, HIGH);
        break;
      case 'j':
        digitalWrite(LEDSwitch, LOW);
        break;
      case 'w':
        digitalWrite(ZGantryStepperDirection, LOW);
        for (int i = 0; i < 3200; i++)
        { 
          digitalWrite(ZGantryStepperPulse, HIGH);
          digitalWrite(ZGantryStepperPulse, LOW);
          
          delayMicroseconds(300);
        }
        break;
      case 's':
        digitalWrite(ZGantryStepperDirection, HIGH);
        for (int i = 0; i < 3200; i++)
        {         
          digitalWrite(ZGantryStepperPulse, HIGH);
          digitalWrite(ZGantryStepperPulse, LOW);
          
          delayMicroseconds(300);
        }
        break;
      case 'a':
        digitalWrite(XGantryStepperDirection, LOW);
        for (int i = 0; i < 3200; i++)
        {         
          if(digitalRead(XAxisLimitSwitch) == 0)
          {
            digitalWrite(XGantryStepperDirection, HIGH);
            while (digitalRead(XAxisLimitSwitch) != 1)
            {
              digitalWrite(XGantryStepperPulse, HIGH);
              digitalWrite(XGantryStepperPulse, LOW);
              
              delayMicroseconds(300);
            }
            for (int i = 1; i < 300; i++)
            {
              digitalWrite(XGantryStepperPulse, HIGH);
              digitalWrite(XGantryStepperPulse, LOW);
    
              delayMicroseconds(300);
            }
            break;
          }
          digitalWrite(XGantryStepperPulse, HIGH);
          digitalWrite(XGantryStepperPulse, LOW);
          
          delayMicroseconds(300);
        }
        break;
      case 'd':
        digitalWrite(XGantryStepperDirection, HIGH);
        for (int i = 0; i < 3200; i++)
        {         
          digitalWrite(XGantryStepperPulse, HIGH);
          digitalWrite(XGantryStepperPulse, LOW);
          
          delayMicroseconds(300);
        }
        break;
      case 'z':
        digitalWrite(ZGantryStepperDirection, HIGH);
        while (digitalRead(ZAxisLimitSwitch) != 0)
        {         
          digitalWrite(ZGantryStepperPulse, HIGH);
          digitalWrite(ZGantryStepperPulse, LOW);
          
          delayMicroseconds(1000);
        }
        break;
      case 'x':
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
        break;
      case 'v':
        digitalWrite(TurntableStepperDirection, LOW);
        for (int i = 0; i < 12800; i++)
        {         
          digitalWrite(TurntableStepperPulse, HIGH);
          digitalWrite(TurntableStepperPulse, LOW);
          
          delayMicroseconds(300);
        }
        break;
      case 'c':
        digitalWrite(TurntableStepperDirection, HIGH);
        for (int i = 0; i < 12800; i++)
        {         
          digitalWrite(TurntableStepperPulse, HIGH);
          digitalWrite(TurntableStepperPulse, LOW);
          
          delayMicroseconds(300);
        }
        break;

      default:
        stopPrevCommand();
        digitalWrite(FrontMotorIn1, LOW);
        digitalWrite(FrontMotorIn2, LOW);
        digitalWrite(LeftMotorIn1, LOW);
        digitalWrite(LeftMotorIn2, LOW); 
        digitalWrite(RightMotorIn1, LOW);
        digitalWrite(RightMotorIn2, LOW);
        digitalWrite(BackMotorIn1, LOW);
        digitalWrite(BackMotorIn2, LOW);
        digitalWrite(PumpSwitch, LOW);
        digitalWrite(LEDSwitch, LOW);
    }
    prevCommand = command;
  }
  readUltrasonicSensors();
}

void stopPrevCommand()
{
  switch (prevCommand) {
    case 'f':
    case 'b':
      for (int i = 127; i >= 0; --i)
      {
        analogWrite(LeftMotorEnable, i);
        analogWrite(RightMotorEnable, i);
        delay(20);
      } 
      digitalWrite(LeftMotorIn1, LOW);
      digitalWrite(LeftMotorIn2, LOW);  
      digitalWrite(RightMotorIn1, LOW);
      digitalWrite(RightMotorIn2, LOW);  
      break;
    case 'l':
    case 'r':
      for (int i = 127; i >= 0; --i)
      {
        analogWrite(FrontMotorEnable, i);
        analogWrite(BackMotorEnable, i);
        delay(20);
      } 
      digitalWrite(FrontMotorIn1, LOW);
      digitalWrite(FrontMotorIn2, LOW);  
      digitalWrite(BackMotorIn1, LOW);
      digitalWrite(BackMotorIn2, LOW);  
      break;
    default:
      digitalWrite(FrontMotorIn1, LOW);
      digitalWrite(FrontMotorIn2, LOW);
      digitalWrite(LeftMotorIn1, LOW);
      digitalWrite(LeftMotorIn2, LOW); 
      digitalWrite(RightMotorIn1, LOW);
      digitalWrite(RightMotorIn2, LOW);
      digitalWrite(BackMotorIn1, LOW);
      digitalWrite(BackMotorIn2, LOW);
    }
}

void readUltrasonicSensors(){
  // Set the trigger to Low for a little while initially
  digitalWrite(FrontUltrasonicTrigger, LOW);
  delayMicroseconds(3);
  // Set the trigger to High for 10 microseconds according to the datasheet
  digitalWrite(FrontUltrasonicTrigger, HIGH);
  delayMicroseconds(10);
  // Set the trigger back to low
  digitalWrite(FrontUltrasonicTrigger, LOW);

  // Wait for the echo with a timeout of 30ms 
  front_ultrasonic_range_duration = pulseIn(FrontUltrasonicEcho, HIGH, 30000);
  
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

  // Wait for the echo with a timeout of 30ms
  right_ultrasonic_range_duration = pulseIn(RightUltrasonicEcho, HIGH, 30000);
  
  // Find the distance in cm based on the duration
  // Values taken from the datasheet at https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
  right_ultrasonic_range_distance = right_ultrasonic_range_duration / 58;
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

  readUltrasonicSensors();
  publishUltrasonicRangeMsgs();

  nh.spinOnce();
  delay(1);
}
