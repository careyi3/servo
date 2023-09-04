#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>

double input, output, setpoint, kp, ki, kd;

PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);
Encoder encoder(2, 3);

void readEncoder();
void calculatePID();
void driveMotor();

void setupPID()
{
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);
  kp = 50.0;
  ki = 0.0;
  kd = 0.0;
  setpoint = 50.0;
  pid.SetTunings(kp, ki, kd);
}

void setupMotor()
{
  pinMode(DD5, OUTPUT);
  pinMode(DD6, OUTPUT);
  analogWrite(DD5, 0);
  analogWrite(DD6, 0);
}

void setup()
{
  Serial.begin(9600);
  setupPID();
  setupMotor();
}

void loop()
{
  readEncoder();
  calculatePID();
  driveMotor();
}

void readEncoder()
{
  input = encoder.read();
}

void calculatePID()
{
  pid.Compute();
}

void driveMotor()
{
  int throttle = (int)output;
  if (throttle > 0)
  {
    analogWrite(DD5, throttle);
    analogWrite(DD6, 0);
  }
  else
  {
    analogWrite(DD6, -1 * throttle);
    analogWrite(DD5, 0);
  }
}