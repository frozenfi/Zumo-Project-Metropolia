/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>

#define SPEED 255 // 0 < SPEED <= 255
#define DEBUGMODE 0

//main functions
void line(void);
void maze(void);
void zumo(void);

/********************/
/* USEFUL FUNCTIONS */
/********************/

// start everything
void start_IR_refl_motor(void);

// TURNS //
//turn backwards
void motor_turn_backwards(uint8 l_speed, uint8 r_speed, uint32 delay);
// tank turns
void motor_tank_turn_left(uint8 speed, uint32 delay);
void motor_tank_turn_right(uint8 speed, uint32 delay);
// 90 deg
void motor_tank_turn_90_left(void);
void motor_tank_turn_90_right(void);

// setting up speed using percents
int percent_speed(uint8 percent);

// Tools for line following
bool is_on_black_line(struct sensors_ dig); // checks if robot is standing on black line
bool is_on_track(struct sensors_ dig);      // checks if robot is standing on track
bool is_on_line(struct sensors_ dig);       // checks if robot is only on track ( only two middle sensors are on line )
void go_to_the_first_line(struct sensors_ dig);

//button press and LED
void btn_press(void);

//handle the line movement
void line_movement(struct sensors_ dig);

//Motor halt
void halt_motor(void);

//line miss and found
void line_miss(struct sensors_ dig, bool *lineMiss);

/* [] END OF FILE */
