#ifndef _ARM_DATA_H_
#define _ARM_DATA_H

typedef struct rec_arm_data_struct rec_arm_data;
typedef struct store_arm_data_struct store_arm_data;
typedef struct pregame_arm_data_struct pregame_arm_data;

struct pregame_arm_data_struct {
  float arm0_x;
  float arm0_y;
  float arm1_x;
  float arm1_y;
}

struct rec_arm_data_struct {
  float arm_x;
  float arm_y;
  float arm_vx;
  float arm_vy;
}

struct store_arm_data_struct {
  float arm_x;
  float arm_y;

  int motor0_step_pin;
  int motor1_step_pin;

  int motor0_dir_pin;
  int motor1_dir_pin;

  float motor0_speed;
  float motor1_speed;
}

#endif
