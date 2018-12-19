#include <stdlib.h>

#include "serial_inputs.h"
#include "store_arm_data"

// arm0(left) setup
store_arm_data *arm0 = malloc(sizeof(store_arm_data));
arm0->motor0_step_pin = 2;
arm0->motor1_step_pin = 3;
arm0->motor0_dir_pin = 4;
arm0->motor1_dir_pin = 5;

// arm1(right) setup
store_arm_data *arm1 = malloc(sizeof(store_arm_data));
arm1->motor0_step_pin = 6;
arm1->motor1_step_pin = 7;
arm1->motor0_dir_pin = 8;
arm1->motor1_dir_pin = 9;

void calibrate_motors()
{
  rec_arm_data *pregame_arm_data = read_arm_pregame();
  arm0->arm_x = pregame_arm_data->arm_x;
  arm0->arm_y = pregame_arm_data->arm_y;
  free(pregame_arm_data);
}

void move_arm(store_arm_data *arm, float goal_x, float goal_y)
{

}


void setup() {
  // set motor control pin modes
  pinMode(arm0->motor0_step_pin, OUTPUT);
  pinMode(arm0->motor1_step_pin, OUTPUT);
  pinMode(arm0->motor0_dir_pin, OUTPUT);
  pinMode(arm0->motor1_dir_pin, OUTPUT);

  pinMode(arm1->motor0_step_pin, OUTPUT);
  pinMode(arm1->motor1_step_pin, OUTPUT);
  pinMode(arm1->motor0_dir_pin, OUTPUT);
  pinMode(arm1->motor1_dir_pin, OUTPUT);
  // digitalWrite(dirPin,HIGH); //Enables the motor to move in a particular direction
}


void loop() {

  customDelayMapped = speedUp(); // Gets custom delay values from the custom speedUp function
  // Makes pules with custom delay, depending on the Potentiometer, from which the speed of the motor depends
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(customDelayMapped);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(customDelayMapped);
}
// Function for reading the Potentiometer
int speedUp() {
  int customDelay = analogRead(A0); // Reads the potentiometer
  int newCustom = map(customDelay, 0, 1023, 300,4000); // Convrests the read values of the potentiometer from 0 to 1023 into desireded delay values (300 to 4000)
  return newCustom;
}
