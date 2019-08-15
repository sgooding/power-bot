/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>



#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100
#define PIN_INPUT A8
int statpin = 13;

#define CENTER_OFFSET 100

/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)

//Define Variables we'll be connecting to
double Setpoint, error, Output;

//Specify the links and initial tuning parameters
//double Kp=2, Ki=.05, Kd=.01;
double Kp=2, Ki=.05, Kd=.01;
PID myPID(&error, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);



void setup()
{
  Serial.begin(9600);
  myPID.SetOutputLimits(-100.0,100.0);
  
  pinMode(statpin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  
  //initialize the variables we're linked to
  
  Setpoint = CENTER_OFFSET-map(analogRead(A9),0,1024,0,255);
  error = CENTER_OFFSET-map(analogRead(PIN_INPUT),0,1024,0,255);
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(33);
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled
 
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 
 pwm: should be a value between ? and 1023, higher the number, the faster
 it'll go
 */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}
void loop()
{
  error = CENTER_OFFSET-map(analogRead(PIN_INPUT),0,1024,0,255);
  Setpoint = CENTER_OFFSET-map(analogRead(A9),0,1024,0,255);

  myPID.Compute();
  
  //double motor_pwm(Output-Setpoint);
  motorGo(0, Output>0?CW:CCW, abs(Output));
  
  //Serial.print("Setpoint: ");
  //Serial.print(Setpoint);
  //Serial.print(" error: ");
  //Serial.print(error);
  //Serial.print(" Output: ");
  //Serial.println(Output);
  //Serial.print(" MotorPWM: ");
  //Serial.println(motor_pwm);
  Serial.print(Setpoint);
  Serial.print(",");
  Serial.println(error);
  
  
  
  if ((analogRead(cspin[0]) < CS_THRESHOLD) && (analogRead(cspin[1]) < CS_THRESHOLD))
    digitalWrite(statpin, HIGH);
}
