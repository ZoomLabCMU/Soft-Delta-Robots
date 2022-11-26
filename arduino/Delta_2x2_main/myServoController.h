#ifndef myServoController_h
#define myServoController_h
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

#define NUM_MOTORS 12


class myServoController {
public:
  myServoController(Adafruit_DCMotor* motors[NUM_MOTORS], 
                    Adafruit_ADS1015* adcs[NUM_MOTORS], 
                    int channels[NUM_MOTORS]);
  void init_controller();
  void cmdPos(float qCmd[]);
  void cmdVel(float q_dotCmd[]);
  void driveController();
  void update_state();
  void print_state();

  // State estimation
  float _joint_positions[NUM_MOTORS];
  float _prev_joint_positions[NUM_MOTORS];
  float _joint_velocities[NUM_MOTORS];

  // Control
  float _joint_positionsCmd[NUM_MOTORS];
  float _joint_velocitiesCmd[NUM_MOTORS];
  int _motorCmd[NUM_MOTORS];
  
private:
  // Position
  Adafruit_ADS1015* _adcs[NUM_MOTORS];
  int _channels[NUM_MOTORS];
  void read_joint_positions();

  // Actuator
  Adafruit_DCMotor *_motors[NUM_MOTORS];

  long _millis;
  long _delta_millis;

  // cmdPos
  void positional_PID(float PID_Cmd[NUM_MOTORS]);

  // Note these constants were determined with error in meters
  const float _cmdPos_Kp = 10000;
  const float _cmdPos_Ki = 10;
  const float _cmdPos_Kd = 100;
  float _cmdPos_error[NUM_MOTORS];
  float _cmdPos_integral_error[NUM_MOTORS];
  float _cmdPos_derivative_error[NUM_MOTORS];

  // driveController
  void send_motorCmd();

  
};
#endif
