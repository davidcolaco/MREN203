/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 10
  #define LEFT_MOTOR_BACKWARD  7
  #define RIGHT_MOTOR_FORWARD  11
  #define LEFT_MOTOR_FORWARD   4
  #define RIGHT_MOTOR_ENABLE 5
  #define LEFT_MOTOR_ENABLE 6
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
