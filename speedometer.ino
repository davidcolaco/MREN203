/**
 * @file motor-angular-rate.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca)
 * @brief Arduino program to estimate motor speed from encoder.
 * @version 2.1
 * @date 2022-12-09
 *
 * @copyright Copyright (c) 2021-2022
 *
 *
 */

#include <Arduino_LSM6DS3.h>


float omega_x, omega_y, omega_z = 0;

float a_f, g_f;

const float lengthRobot = 0.2775;

double dOmega = -10;
double dSpeed = 0.05;

bool firstVal;
float fox, foy, foz;
double eint;


// Wheel PWM pin (must be a PWM pin)
int EA = 6;
int EB = 12;

// Wheel direction digital pins
int I1 = 7;
int I2 = 4;
int I3 = 11;
int I4 = 10;
// Motor PWM command variable [0-255]
double uR = 0;
double uL = 0;

// Left wheel encoder digital pins
const byte SIGNAL_A = 2;
const byte SIGNAL_B = 3;
const byte SIGNAL_C = 8;
const byte SIGNAL_D = 9;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticksL = 0;
volatile long encoder_ticksR = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 1000;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicksL()
{
    if (digitalRead(SIGNAL_B) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticksL--;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticksL++;
    }

}
void decodeEncoderTicksR()
{

     if (digitalRead(SIGNAL_D) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticksR--;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticksR++;
    }
}

double vehicleSpeed(double v_L, double v_R){
  double v;
  v = 0.5 * (v_L + v_R);
  return v;
}

double ENowR(double vRDesired, double vR){
  return vRDesired - vR;
}
double ENowL(double vLDesired, double vL){
  return vLDesired - vL;
}
double GetVLDesired(double vDesired, double ODesired){
  return vDesired - ODesired * 0.5 * lengthRobot;
}
double GetVRDesired(double vDesired, double ODesired){
  return vDesired + ODesired * 0.5 * lengthRobot;
}

double turningRate(float vr, float vl){
  double turningRate = (1/0.2775) * (vr - vl);
  return turningRate;
}

short PIControllerR(double e_now, double k_P, double e_int, double k_I){
  short u;
  u = (short)(k_P * e_now + k_I * e_int);
  if(u > 255){
    u = 255;
  } else if (u < 255){
    u = -255;
  }
  Serial.println("Calling it");
  return u;
}
short PIControllerL(double e_now, double k_P, double e_int, double k_I){
  short u;
  u = (short)(k_P * e_now + k_I * e_int);
  if(u > 255){
    u = 255;
  } else if (u < -255){
    u = -255;
  }
  Serial.println("Calling it");
  return u;
}

void setup()
{
    // Open the serial port at 9600 bps
    Serial.begin(9600);

    // Set the pin modes for the motor driver
    pinMode(EA, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    // Set the pin modes for the encoders
    pinMode(SIGNAL_A, INPUT);
    pinMode(SIGNAL_B, INPUT);
    pinMode(SIGNAL_C, INPUT);
    pinMode(SIGNAL_D, INPUT);

    // Every time the pin goes high, this is a pulse
    attachInterrupt(digitalPinToInterrupt(SIGNAL_A), decodeEncoderTicksL, RISING);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_C), decodeEncoderTicksR, RISING);


    // Print a message
    Serial.print("Program initialized.");
    Serial.print("\n");

      Serial.begin(115200);

  while(!Serial){
    delay(10);
  }

  Serial.println();

  // Check board init
  if (!IMU.begin()){
    // Print err
    Serial.println("IMU init  failed");
    while(1){
      delay(10);
    }
  }

  a_f = IMU.accelerationSampleRate();
  g_f = IMU.gyroscopeSampleRate();

  // Print sample rate
  Serial.println("Accel Sample Rate: ");
  Serial.print(a_f);
  Serial.println("Gyro Sample Rate: ");
  Serial.print(g_f);

  
    
}

void loop()
{
    // Get the elapsed time [ms]
    t_now = millis();

    if (t_now - t_last >= T)
    {
        // Estimate the rotational speed [rad/s]
        omega_L = 2.0 * PI * ((double)encoder_ticksL / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        omega_R = 2.0 * PI * ((double)encoder_ticksR / (double)TPR) * 1000.0 / (double)(t_now - t_last);

        // Print some stuff to the serial monitor
        /*
        Serial.print("Estimated left wheel speed: ");
        Serial.print(omega_L);
        Serial.print(" rad/s");
        Serial.print("    Estimated right wheel speed: ");
        Serial.print(-omega_R);
        Serial.print(" rad/s");
        Serial.print("\n");
        
        Serial.print("Translation left wheel speed: ");
        Serial.print(omega_L*RHO);
        Serial.print(" m/s");
        Serial.print("    Translation right wheel speed: ");
        Serial.print(-omega_R*RHO);
        Serial.print(" m/s");
        Serial.print("\n");
        */
        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        encoder_ticksL = 0;
        encoder_ticksR = 0;

          if (IMU.gyroscopeAvailable()){
    if(firstVal){
          IMU.readGyroscope(fox, foy, foz);
          firstVal = false;
    }

    IMU.readGyroscope(omega_x, omega_y, omega_z);
    omega_x = omega_x - fox;
    omega_y = omega_y - foy;
    omega_z = omega_z - foz;


    /*
    Serial.println("Gyroscope Measurements: ");
    Serial.print(omega_x);
    Serial.print("\t");
    Serial.print(omega_y);
    Serial.print("\t");
    Serial.print(omega_z);
    Serial.print(" deg/s\n");

    Serial.println(turningRate(-omega_R*RHO, omega_L*RHO));
    */

    }

    double enowR = ENowR(GetVRDesired(dSpeed, dOmega), omega_R);
    double enowL = ENowL(GetVLDesired(dSpeed, dOmega), omega_L);

    double eintR = eintR + enowR;
    double eintL = eintL + enowL;
    // Set the wheel motor PWM command [0-255]
    uR = PIControllerR(enowR, 200, eintR, 100); // Using R for now but they are diff (L and R)
    uL = PIControllerL(enowL, 200, eintL, 100);

    // Select a direction
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);

    // PWM command to the motor driver
    analogWrite(EA, (-1)*uR);
    analogWrite(EB, (-1)*uL);
  }

      
    
}
