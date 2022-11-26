#include "myServoController.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

// 100mm per 1650 bits (3.3V/4.096V)*2^11 (range is +- 4.096V with 1 gain)
#define ADC2M 0.0000606060606 

myServoController::myServoController(Adafruit_DCMotor* motors[NUM_MOTORS], 
                                     Adafruit_ADS1015* adcs[NUM_MOTORS], 
                                     int channels[NUM_MOTORS]) {
  for (int i=0; i<NUM_MOTORS; i++){
    _motors[i] = motors[i];
    _adcs[i] = adcs[i];
    _channels[i] = channels[i];
  }
}


void myServoController::init_controller() {
  // Initialize actuator and encoder
  _millis = millis();
  for (int i=0; i<NUM_MOTORS; i++) {
    _motors[i]->setSpeed(0);
    _motors[i]->run(RELEASE);
    _joint_velocities[i] = 0;
  }
  read_joint_positions();
  // IMPLEMENT: Set all joints to 0
  read_joint_positions();
}

void myServoController::read_joint_positions() {
  for (int i=0; i<NUM_MOTORS; i++) {
    int ADC_val = _adcs[i]->readADC_SingleEnded(_channels[i]);
    _joint_positions[i] = ADC2M*ADC_val;
  }
}

void myServoController::update_state() {
  // Implement me!
  for (int i=0; i<NUM_MOTORS; i++) {
    _prev_joint_positions[i] = _joint_positions[i];
  }
  long curr_millis = millis();
  _delta_millis = curr_millis - _millis;
  _millis = curr_millis;
  read_joint_positions();
  for (int i=0; i<NUM_MOTORS; i++) {
    _joint_velocities[i] = (_joint_positions[i] - _prev_joint_positions[i])/(_delta_millis * 0.001);
  }
  
}

void myServoController::print_state() {
  //Serial.println("Joint Positions [mm]");
  //for (int i=0; i<NUM_MOTORS; i++) {
  //  Serial.print(_joint_positions[i]*1000); Serial.print('\t');
  //}
  //Serial.println();
  
  //Serial.println("Motor Commands (x/255)");
  //for (int i=0; i<NUM_MOTORS; i++) {
  //  Serial.print(_motorCmd[i]); Serial.print('\t');
  //}
  //Serial.println();
  Serial.print(_joint_positions[0]*1000); Serial.print('\t');
  Serial.println(_joint_positionsCmd[0]*1000);
}

void myServoController::cmdPos(float qCmd[]) {
  for(int i=0; i<NUM_MOTORS; i++) {
    _joint_positionsCmd[i] = qCmd[i];
  }
}

void myServoController::cmdVel(float q_dotCmd[]) {
  for (int i=0; i<NUM_MOTORS; i++){
    _joint_velocitiesCmd[i] = q_dotCmd[i];
  }
}

void myServoController::positional_PID(float PID_Cmd[NUM_MOTORS]) {
  for (int i=0; i<NUM_MOTORS; i++) {
    float error = _joint_positionsCmd[i] - _joint_positions[i];
    _cmdPos_integral_error[i] = _cmdPos_integral_error[i] + error * (0.001*_delta_millis);
    _cmdPos_derivative_error[i] = (error - _cmdPos_error[i]) / (0.001*_delta_millis);
    _cmdPos_error[i] = error;
  
    PID_Cmd[i] = (_cmdPos_Kp*_cmdPos_error[i] + 
                  _cmdPos_Ki*_cmdPos_integral_error[i] + 
                  _cmdPos_Kd*_cmdPos_derivative_error[i]);
  }
}

void myServoController::send_motorCmd() {
  for (int i=0; i<NUM_MOTORS; i++) {
    int motor_cmd_val = _motorCmd[i];
    if (motor_cmd_val > 0) {
      _motors[i]->run(FORWARD);
    } else{
      _motors[i]->run(BACKWARD);
    }
    _motors[i]->setSpeed((uint8_t)(abs(motor_cmd_val)));
  }
}

void myServoController::driveController() {
  update_state();
  
  float PID_Cmd[NUM_MOTORS];
  positional_PID(PID_Cmd);

  for (int i=0; i<NUM_MOTORS; i++) {
    _motorCmd[i] = (int)(1.0 * PID_Cmd[i]);
  }

  send_motorCmd();
}
