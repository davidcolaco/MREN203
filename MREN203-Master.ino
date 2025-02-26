// ╔═══════════════════════════════════════════════════════════════════════╗
// ║ Start of File                                                         ║
// ╚═══════════════════════════════════════════════════════════════════════╝
/********************************************
 *  Project: MREN 203 
 *  Authors: David Colaco, Jared Commanda, Mariana Siqueira 
 *  Description: Centralized File for MREN 203
 *  Board:Arduino Uno WiFi Rev2
 ********************************************/
// ╔═══════════════════════════════════════════════════════════════════════╗
// ║ Includes, Global Constants & Variables                                ║
// ╚═══════════════════════════════════════════════════════════════════════╝
//Includes
#include <Arduino.h>
#include <Arduino_LSM6DS3.h>

//Robot Constants 
const double RHO = 0.0625;// Wheel radius [m]
const float lengthRobot = 0.2775;

//Variables
double uR = 0;// Motor PWM command variable [0-255]
double uL = 0;// Motor PWM command variable [0-255]

volatile long encoder_ticksL = 0;// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticksR = 0;// Counter to keep track of encoder ticks [integer]

double omega_L = 0.0;// Variable to store estimated angular rate of left wheel [rad/s]
double omega_R = 0.0;// Variable to store estimated angular rate of left wheel [rad/s]

const int T = 1000;// Sampling interval for measurements in milliseconds

long t_now = 0;// Counters for milliseconds during interval
long t_last = 0;// Counters for milliseconds during interval

const int TPR = 3000;// Encoder ticks per (motor) revolution (TPR)
float omega_x, omega_y, omega_z = 0;
float a_f, g_f;
double dOmega = -10;
double dSpeed = 0.05;
bool firstVal;
float fox, foy, foz;
double eint;

// ╔═══════════════════════════════════════════════════════════════════════╗
// ║ Pin Config                                                            ║
// ╚═══════════════════════════════════════════════════════════════════════╝

int EA = 6;// Wheel PWM pin (must be a PWM pin)
int EB = 5;// Wheel PWM pin (must be a PWM pin)
int I1 = 7;// Wheel direction digital pins
int I2 = 4;// Wheel direction digital pins
int I3 = 11;// Wheel direction digital pins
int I4 = 10;// Wheel direction digital pins
// Left wheel encoder digital pins
const byte SIGNAL_A = 2;
const byte SIGNAL_B = 3;
// Right wheel encoder digital pins
const byte SIGNAL_C = 8;
const byte SIGNAL_D = 9;

// ╔═══════════════════════════════════════════════════════════════════════╗
// ║ System Interrupts                                                     ║
// ╚═══════════════════════════════════════════════════════════════════════╝
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
// This function is called when SIGNAL_C goes HIGH
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
// ╔═══════════════════════════════════════════════════════════════════════╗
// ║ Setup Functions                                                       ║
// ╚═══════════════════════════════════════════════════════════════════════╝

void setup() {
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
    //Serial.begin(115200);

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

// ╔═══════════════════════════════════════════════════════════════════════╗
// ║ Additional Functions                                                  ║
// ╚═══════════════════════════════════════════════════════════════════════╝
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
  } else if (u < -255){
    u = -255;
  }
  //Serial.println("Calling it");
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
  //Serial.println("Calling it");
  return u;
}
// ╔═══════════════════════════════════════════════════════════════════════╗
// ║ Main Loop                                                             ║
// ╚═══════════════════════════════════════════════════════════════════════╝

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
      
        /*Serial.print("Estimated left wheel speed: ");
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
        Serial.print("\n");*/
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


          
          /*Serial.println("Gyroscope Measurements: ");
          Serial.print(omega_x);
          Serial.print("\t");
          Serial.print(omega_y);
          Serial.print("\t");
          Serial.print(omega_z);
          Serial.print(" deg/s\n");
          Serial.print("turingingRate:  ");
          Serial.println(turningRate(-omega_R*RHO, omega_L*RHO));*/
      
    }

    // Set the wheel motor PWM command [0-255]
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
 
    analogWrite(EA, 255);
    analogWrite(EB, 255);
    delay(1000);
  
    analogWrite(EA, 0);
    analogWrite(EB, 0);
    delay(1000);
}
}



// ╔═══════════════════════════════════════════════════════════════════════╗
// ║ Unused/Trash                                                          ║
// ╚═══════════════════════════════════════════════════════════════════════╝

// ╔═══════════════════════════════════════════════════════════════════════╗
// ║ End of File                                                           ║
// ╚═══════════════════════════════════════════════════════════════════════╝

