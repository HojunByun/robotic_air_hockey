#include <AccelStepper.h>

AccelStepper stepper(AccelStepper::DRIVER, 2, 3);

#define MAX_ERROR 2

int goal_acc = 5000;
int goal_vel = 4000;
int goal_theta = 3200;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  stepper.setMaxSpeed(goal_vel);
  stepper.setAcceleration(goal_acc);
  stepper.setCurrentPosition(0);
}

void loop() {
  // how to get ard
  if (stepper.currentPosition() != goal_theta) {
    stepper.moveTo(1000000);
    stepper.run();
  }
}
