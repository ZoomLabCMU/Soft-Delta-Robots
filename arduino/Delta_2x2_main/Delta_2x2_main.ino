#include "delta_trajectory.pb.h"
#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>
#include "myServoController.h"

#define NUM_MOTORS 12 
#define MY_ID 15


//################################## Feather MC and ADC Libraries INIT #####################3
Adafruit_MotorShield MC0 = Adafruit_MotorShield(0x62);
Adafruit_MotorShield MC1 = Adafruit_MotorShield(0x60);
Adafruit_MotorShield MC2 = Adafruit_MotorShield(0x61);
//
Adafruit_DCMotor *MC0_M1 = MC0.getMotor(1);
Adafruit_DCMotor *MC0_M2 = MC0.getMotor(2);
Adafruit_DCMotor *MC0_M3 = MC0.getMotor(3);
Adafruit_DCMotor *MC0_M4 = MC0.getMotor(4);
Adafruit_DCMotor *MC1_M1 = MC1.getMotor(1);
Adafruit_DCMotor *MC1_M2 = MC1.getMotor(2);
Adafruit_DCMotor *MC1_M3 = MC1.getMotor(3);
Adafruit_DCMotor *MC1_M4 = MC1.getMotor(4);
Adafruit_DCMotor *MC2_M1 = MC2.getMotor(1);
Adafruit_DCMotor *MC2_M2 = MC2.getMotor(2);
Adafruit_DCMotor *MC2_M3 = MC2.getMotor(3);
Adafruit_DCMotor *MC2_M4 = MC2.getMotor(4);
//
Adafruit_DCMotor* motors[NUM_MOTORS] = {MC0_M1,MC0_M2,MC1_M1,// 1st robot
                                        MC1_M2,MC2_M1,MC2_M2,// 2nd robot
                                        MC2_M3,MC2_M4,MC1_M3,// 3rd robot
                                        MC1_M4,MC0_M3,MC0_M4,// 4th robot
                                        };
//Adafruit_DCMotor* motors[NUM_MOTORS] = {MC1_M4};


Adafruit_ADS1015 ADC2;
Adafruit_ADS1015 ADC1;
Adafruit_ADS1015 ADC0;

Adafruit_ADS1015* adcs[NUM_MOTORS] = {&ADC2, &ADC2, &ADC1,//1st robot
                                      &ADC1, &ADC0, &ADC0,//2nd robot
                                      &ADC0, &ADC0, &ADC1,//3rd robot
                                      &ADC1, &ADC2, &ADC2,//4th robot
                                      };

int channels[NUM_MOTORS] = {0,1,0,//1st robot
                            1,0,1,//2nd robot
                            2,3,2,//3rd robot
                            3,2,3,//4th robot
                            };

myServoController controller(motors, adcs, channels);


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial1) {
    delay(10);
  }
  // set all the base dc motor control pins to outputs
  MC0.begin();
  MC1.begin();
  MC2.begin();
  // start all the ADCs
  ADC2.begin(0x4A);
  ADC1.begin(0x49);
  ADC0.begin(0x48);

  ADC2.setGain(GAIN_ONE);
  ADC1.setGain(GAIN_ONE);
  ADC0.setGain(GAIN_ONE);
  
  controller.init_controller();
}

float qCmd[NUM_MOTORS] = {0.000, 0.000, 0.000,
                          0.000, 0.000, 0.000,
                          0.000, 0.000, 0.000,
                          0.000, 0.000, 0.000};
long t_start = millis();
int i=0;

void loop() {
  controller.cmdPos(qCmd);
  
  controller.driveController();
  i++;
  if (i == 10) {
    i = 0;
    controller.print_state();
  }

  float val = 0.050 + 0.040*sin((millis()-t_start)*0.001*2*3.14/20);

  for (int i=0; i<9; i++) {
    qCmd[i] = val;
  }
  delay(50);
}
