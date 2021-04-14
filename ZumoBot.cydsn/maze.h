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

#include "main_includes.h"



// avoiding functions
static void avoid_obstacle_and_turn_north(uint8 *dist, int8 *x, int8 *y, int8 *robotdir);

// point functions
static int go_to_next_point(int8 *x, int8 *y, int8 *robotdir);
static bool is_on_point(struct sensors_ dig);
static void count_coordinate(int8 *x, int8 *y, int8 *robotdir);
static void allign_on_point(uint8 init_dist);
static void allign_on_point_backwards( uint8 dist );

// line following functions
static int follow_line_forward(struct sensors_ dig);
static int follow_line_backward(struct sensors_ dig, uint8 speed);

// turn functions
static void maze_motor_tank_turn_left( uint8 speed, uint32 delay, int8 *robotdir );
static void maze_motor_tank_turn_right( uint8 speed, uint32 delay, int8 *robotdir );

static void maze_motor_turn_left( uint8 speed, uint32 delay, int8 *robotdir );
static void maze_motor_turn_right( uint8 speed, uint32 delay, int8 *robotdir );

/* [] END OF FILE */
