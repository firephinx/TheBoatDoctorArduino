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

int prevCommand = 0;

void setup()
{
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  
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

  // TODO: set IMU pins here

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
  long durationForward;
  float distanceForward;

  // Set the trigger to Low for a little while initially
  digitalWrite(FrontUltrasonicTrigger, LOW);
  delayMicroseconds(3);
  // Set the trigger to High for 10 microseconds according to the datasheet
  digitalWrite(FrontUltrasonicTrigger, HIGH);
  delayMicroseconds(10);
  // Set the trigger back to low
  digitalWrite(FrontUltrasonicTrigger, LOW);

  // Wait for the echo 
  durationForward = pulseIn(FrontUltrasonicEcho, HIGH, 30000);
  
  // Find the distance in cm based on the duration
  // Values taken from the datasheet at https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
  distanceForward = durationForward / 58;
  
  long durationRight;
  float distanceRight;

  // Set the trigger to Low for a little while initially
  digitalWrite(RightUltrasonicTrigger, LOW);
  delayMicroseconds(3);
  // Set the trigger to High for 10 microseconds according to the datasheet
  digitalWrite(RightUltrasonicTrigger, HIGH);
  delayMicroseconds(10);
  // Set the trigger back to low
  digitalWrite(RightUltrasonicTrigger, LOW);

  // Wait for the echo 
  durationRight = pulseIn(RightUltrasonicEcho, HIGH, 30000);
  
  // Find the distance in cm based on the duration
  // Values taken from the datasheet at https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
  distanceRight = durationRight / 58;
  
  if (distanceRight >= 200 || distanceRight <= 0){
    Serial.println("Out of range in X direction");
    if (distanceForward >= 200 || distanceForward <= 0){
      Serial.println("Out of range in Y direction");
    }
    else
    {
      Serial.print(distanceForward);
      Serial.println(" cm in y");
    }
  }
  else if (distanceForward >= 200 || distanceForward <= 0){
    Serial.print(distanceRight);
    Serial.println(" cm in x");
    Serial.println("Out of range in Y direction");
  }
  else {
    Serial.print("(");
    Serial.print(distanceRight);
    Serial.print(",");
    Serial.print(distanceForward);
    Serial.println(")");
  }
  delay(100);
}

void loop()
{
  serialCommand();
}
