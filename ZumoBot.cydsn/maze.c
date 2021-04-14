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

#include "maze.h"
#include <stdlib.h>

// directions
#define EAST 0
#define NORTH 1
#define WEST 2
#define SOUTH 3

void maze(void)
{
    int8 x = 0;
    int8 y = -1;
    int8 robotdir = NORTH;

    TickType_t start, stop;
    uint8 dist;
    struct sensors_ dig;

    if(DEBUGMODE) printf("Executing Maze solving code.\n");
    if(DEBUGMODE) printf("Starting IR, reflectance sensors. Starting motor and ultrasonic sensor.\n");
    start_IR_refl_motor();
    Ultra_Start();
    
    printf("Please press the button, robot should be placed on the track.\n");
    // button
    btn_press();
    printf("Robot started\n");

    go_to_the_first_line(dig);
    printf("Please send the IR signal.\n");
    print_mqtt("Zumo05/ready", "maze");
    IR_wait();

    start = xTaskGetTickCount();
    if(DEBUGMODE) printf("IR signal received.\n");
    print_mqtt("Zumo05/start", "%d", start);

    // Maze solving
    dist = Ultra_GetDistance();
    while( y < 11 )
    {
        go_to_next_point(&x, &y, &robotdir);
        dist = Ultra_GetDistance();
        if( dist < 15 )
        {
            motor_forward( 0, 0 );
            allign_on_point(dist); // Robot should be alligned before the avoiding, this way we can start from alligning backwards as well
            avoid_obstacle_and_turn_north(&dist, &x, &y, &robotdir);
        }
    }
    if(DEBUGMODE) printf( "Finished solving at %d %d point.\n", x, y );

    // Finishing part ( is very adjustable and written separately )
    if ( x < 0 ) // to the left from the centre - must go right
    {
        maze_motor_turn_right( SPEED, 0, &robotdir );
        while ( x < 0 ) go_to_next_point(&x, &y, &robotdir);
        maze_motor_turn_left( SPEED, 0, &robotdir );
    }
    else if ( x > 0 ) // to the right from the centre - must go left
    {
        maze_motor_turn_left( SPEED, 0, &robotdir );
        while ( x > 0 ) go_to_next_point(&x, &y, &robotdir);
        maze_motor_turn_right( SPEED, 0, &robotdir );
    }
    go_to_next_point(&x, &y, &robotdir); // robot is centred - so go forward 2 times
    go_to_next_point(&x, &y, &robotdir);
    motor_forward( SPEED, 500 ); // we finished, just have to run forward for some time
    motor_forward( 0, 0 );
    stop = xTaskGetTickCount();
    print_mqtt("Zumo05/stop", "%d", stop);
    print_mqtt("Zumo05/time", "%d", stop - start);
    if(DEBUGMODE) printf("Solved!\n");
    motor_stop();

    while(true)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
}

///////////////////////////////////////////////////////////////////////////
// Avoiding functions //
///////////////////////////////////////////////////////////////////////////

static void avoid_obstacle_and_turn_north(uint8 *dist, int8 *x, int8 *y, int8 *robotdir) // robot must be alligned on the point centre before using this function
{
    bool avoided_to_left = false;
    bool avoided = false;
    while( !avoided ) // we can exit this loop only by avoiding an obstacle
    {
        if( *x > -3 && !avoided_to_left && *robotdir != SOUTH ) // If possible, first we are avoiding to the left ( if we are going south - we have different rules )
        {
            if (DEBUGMODE) printf("Avoiding to the left\n");
            maze_motor_tank_turn_left( SPEED, 0, robotdir ); // Robot is alligned, so just turn left
            *dist = Ultra_GetDistance();
            if ( *dist >= 15 ) // if there is no obstacle - move there
            {
                go_to_next_point(x, y, robotdir);
                maze_motor_turn_right( SPEED, 0, robotdir ); // instantly turning back to starting direction to check for an obstacle.
                *dist = Ultra_GetDistance();
                if( *dist < 15 ) // there is one - allign on point using it
                {
                    allign_on_point_backwards(*dist);
                }
                else // there is none - avoided
                {
                    avoided = true;
                }
                
            }
        }

        if( *x == -3 && !avoided) avoided_to_left = true; // We are on x= -3 and still haven't avoided an obstacle - looks like we have to avoid it to the right.

        if( *x < 3 && avoided_to_left && *robotdir != SOUTH ) // Then we are avoiding to the right ( south - different rules )
        {
            if (DEBUGMODE) printf("Avoiding to the right\n");
            maze_motor_tank_turn_right( SPEED, 0, robotdir ); // Robot is alligned, so just turn left
            *dist = Ultra_GetDistance();
            if ( *dist >= 15 ) // if there is no obstacle - move there
            {
                go_to_next_point(x,y,robotdir);
                maze_motor_turn_left( SPEED, 0, robotdir ); // instantly turning back to starting direction to check for an obstacle.
                *dist = Ultra_GetDistance();
                if( *dist < 15 ) // there is one - allign on point using it
                {
                    allign_on_point_backwards(*dist);
                }
                else // there is none - avoided
                {
                    avoided = true;
                }
            }
        }
        // We ended up looking backwards. So we must behave in a different way
        if( *x > -3 && !avoided_to_left && *robotdir == SOUTH ) // We are still trying to avoid it to the left, so we must use different algorithm
        {
            if (DEBUGMODE) printf("Avoiding to the left backwards\n");
            *dist = Ultra_GetDistance();
            if( *dist >= 15 ) // if there is no obstacle - move there
            {
                go_to_next_point(x,y,robotdir);
                maze_motor_turn_right( SPEED, 0, robotdir ); // instantly turning right to check for an obstacle
                *dist = Ultra_GetDistance();
                if( *dist < 15 ) // there is one - allign on point using it
                {
                    allign_on_point_backwards(*dist);
                    maze_motor_tank_turn_left( SPEED, 0, robotdir ); // turn back South
                }
                else // there is none, go there
                {
                    go_to_next_point(x,y,robotdir);
                    maze_motor_turn_right( SPEED, 0, robotdir ); // instantly turning right, in order to end up facing North
                    *dist = Ultra_GetDistance();
                    if ( *dist >= 15 ) // there is no obstacle - avoided
                    {
                        avoided = true;
                    }
                    else // there is obstacle - alligning on point, continue avoiding
                    {
                        allign_on_point_backwards(*dist);
                    }
                }
            }
            else // there is an obstacle to the South - it is dead end
            {
                maze_motor_tank_turn_left( SPEED, 0, robotdir );
                maze_motor_tank_turn_left( SPEED, 0, robotdir );
                avoided_to_left = true;
            }
        }
        if( *x < 3 && avoided_to_left && *robotdir == SOUTH ) // the same as above, but to the right
        {
            if (DEBUGMODE) printf("Avoiding to the right backwards\n");
            *dist = Ultra_GetDistance();
            if( *dist >= 15 ) // if there is no obstacle - move there
            {
                go_to_next_point(x,y,robotdir);
                maze_motor_turn_left( SPEED, 0, robotdir ); // instantly turning right to check for an obstacle
                *dist = Ultra_GetDistance();
                if( *dist < 15 ) // there is one - allign on point using it
                {
                    allign_on_point_backwards(*dist);
                    maze_motor_tank_turn_right( SPEED, 0, robotdir ); // turn back South
                }
                else
                {
                    go_to_next_point(x,y,robotdir);
                    maze_motor_turn_left( SPEED, 0, robotdir ); // instantly turning left, in order to end up facing North
                    *dist = Ultra_GetDistance();
                    if ( *dist >= 15 ) // there is no obstacle - avoided
                    {
                        avoided = true;
                    }
                    else // there is obstacle - alligning on point, continue avoiding
                    {
                        allign_on_point_backwards(*dist);
                    }
                }
            }
            else // Out of ideas here. Hey robot! Just try again with the left side
            {
                maze_motor_tank_turn_right( SPEED, 0, robotdir );
                maze_motor_tank_turn_right( SPEED, 0, robotdir );
                avoided_to_left = false;
            }
        }
    }
    if (DEBUGMODE) printf ("AVOIDED an obstacle\n");
}

///////////////////////////////////////////////////////////////////////////
// Point functions //
///////////////////////////////////////////////////////////////////////////

static int go_to_next_point(int8 *x, int8 *y, int8 *robotdir)
{
    struct sensors_ dig;
    bool left_the_point = true;
    reflectance_digital(&dig);
    if(is_on_point(dig)) left_the_point = false; // if we are still on the point, then we need to handle that too
    while ( !is_on_point(dig) || !left_the_point )
    {
        reflectance_digital(&dig);
        if( !is_on_point(dig) ) left_the_point = true;
        if(follow_line_forward(dig)) // returns 1 in case of mistake
        {
            printf("Out of bounds!\n");
            vTaskDelay(10000);
        }
    }
    motor_forward( 0, 0 );
    count_coordinate(x, y, robotdir); // Successfully reached a point, so counting our coordinates
    print_mqtt("Zumo05/position", "%d %d", *x, *y);
    return 0;
}

static bool is_on_point(struct sensors_ dig) // true if robot is standing on one of the maze points ( not alligned on it, of course )
{
    if( ( dig.L3 == 1 && dig.L2 == 1 && dig.L1 == 1 && dig.R1 == 1 ) || ( dig.R3 == 1 && dig.R2 == 1 && dig.R1 == 1 && dig.L1 == 1 ) ) return true;
    return false;
}

static void count_coordinate(int8 *x, int8 *y, int8 *robotdir) // Used only in one place, when robot goes to the next point
{
    if(*robotdir == EAST) // right
    {
        *x = *x + 1;
    }
    if(*robotdir == NORTH) // up
    {
        *y = *y + 1;
    }
    if(*robotdir == WEST) // left
    {
        *x = *x - 1;
    }
    if(*robotdir == SOUTH) // down
    {
        *y = *y - 1;
    }
}

// ! Allignment can be done only having obstacle ahead ! //
static void allign_on_point(uint8 init_dist)
{
    struct sensors_ dig;
    uint8 dist = init_dist - 6;
    if(DEBUGMODE) printf("Initial distance = %d\n", init_dist);
    if(DEBUGMODE) printf("Needed distance = %d\n", dist);
    while(Ultra_GetDistance() > dist)
    {
        reflectance_digital(&dig);
        if(follow_line_forward(dig)) // returns 1 in case of mistake
        {
            if(DEBUGMODE) printf("Out of bounds!\n");
            vTaskDelay(10000);
        }
    }
    if(DEBUGMODE) printf("Alligned\n");
}

// ! Allignment can be done only having obstacle ahead ! //
static void allign_on_point_backwards( uint8 dist ) // Alligning robot bacwards, is very unstable, but does the job
{
    struct sensors_ dig;
    uint8 needed_dist = 7;
    if(DEBUGMODE) printf("Initial distance = %d\n", dist);
    if(DEBUGMODE) printf("Needed distance = %d\n", needed_dist);
    while ( dist < needed_dist || !is_on_line(dig) )
    {
        reflectance_digital(&dig);
        if(follow_line_backward(dig, SPEED)) // returns 1 in case of mistake
        {
            if(DEBUGMODE) printf("Out of bounds!\n");
            vTaskDelay(10000);
        }
        dist = Ultra_GetDistance();
    }
    motor_forward( 0, 0 );
    if(DEBUGMODE) printf("Alligned\n");
}

///////////////////////////////////////////////////////////////////////////
// Maze line following functions //
///////////////////////////////////////////////////////////////////////////

static int follow_line_forward(struct sensors_ dig) // while loop in the go_to_next_point and allign_on_point will stop us when we will reach next point.
{
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 0 && dig.R3 == 0 ) // We are on the line, so go forward
    {
        motor_forward( SPEED, 0 );
        return 0;
    }
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 1 && dig.R3 == 1 ) // it is a point/start, so go forward
    {
        motor_forward( SPEED, 0 );
        return 0;
    }
    if( dig.L3 == 1 && dig.L2 == 1 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 0 && dig.R3 == 0 ) // the same as above
    {
        motor_forward( SPEED, 0 );
        return 0;
    }
    if( dig.L3 == 1 && dig.L2 == 1 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 1 && dig.R3 == 1 ) // a point, so go forward
    {
        motor_forward( SPEED, 0 );
        return 0;
    }
    if( dig.L3 == 0 && dig.L2 == 1 && dig.L1 == 1 && dig.R1 == 0 && dig.R2 == 0 && dig.R3 == 0 ) // turn left
    {
        motor_turn( 0, SPEED, 0 ); // turning left
        return 0;
    }
    if( dig.L3 == 0 && dig.L2 == 1 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 0 && dig.R3 == 0 ) // turn left
    {
        motor_turn( 0, SPEED, 0 ); // turning left
        return 0;
    }
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 0 && dig.R1 == 1 && dig.R2 == 1 && dig.R3 == 0 ) // turn right
    {
        motor_turn( SPEED, 0, 0 ); // turning right
        return 0;
    }
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 1 && dig.R3 == 0 ) // turn right
    {
        motor_turn( SPEED, 0, 0 ); // turning right
        return 0;
    }

    // Something went wrong
    if( dig.L3 == 1 && dig.L2 == 1 && dig.L1 == 1 && dig.R1 == 0 && dig.R2 == 0 && dig.R3 == 0 ) // well, it can be turn left
    {
        motor_turn( 0, SPEED, 0 ); // turning left
        return 0;
    }
    if( dig.L3 == 1 && dig.L2 == 1 && dig.L1 == 0 && dig.R1 == 0 && dig.R2 == 0 && dig.R3 == 0 ) // well, it can be turn left
    {
        motor_turn( 0, SPEED, 0 ); // turning left
        return 0;
    }
    if( dig.L3 == 1 && dig.L2 == 0 && dig.L1 == 0 && dig.R1 == 0 && dig.R2 == 0 && dig.R3 == 0 ) // Turn left so messed up?
    {
        motor_turn( 0, SPEED, 0 ); // turning left
        return 0;
    }
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 0 && dig.R1 == 1 && dig.R2 == 1 && dig.R3 == 1 ) // well, it can be turn right
    {
        motor_turn( SPEED, 0, 0 ); // turning right
        return 0;
    }
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 0 && dig.R1 == 0 && dig.R2 == 1 && dig.R3 == 1 ) // well, it can be turn right
    {
        motor_turn( SPEED, 0, 0 ); // turning right
        return 0;
    }
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 0 && dig.R1 == 0 && dig.R2 == 0 && dig.R3 == 1 ) // Turn right so messed up?
    {
        motor_turn( SPEED, 0, 0 ); // turning right
        return 0;
    }
    // Allignment messed up, must fix it:
    if( dig.L3 == 0 && dig.L2 == 1 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 1 && dig.R3 == 1 ) // Just small fix to the left
    {
        motor_turn( SPEED/4, SPEED, 0 ); // turning left
        return 0;
    }
    if( dig.L3 == 1 && dig.L2 == 1 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 1 && dig.R3 == 0 ) // Just small fix to the right
    {
        motor_turn( SPEED, SPEED/4, 0 ); // turning right
        return 0;
    }
    // OUT
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 0 && dig.R1 == 0 && dig.R2 == 0 && dig.R3 == 0 ) // lost the line. There is no point in searching for it
    {
        motor_forward( 0, 0 );
        return 1;
    }
    return 0; // I haven't considered everything, it is not wise to stop him at that point.
}

static int follow_line_backward(struct sensors_ dig, uint8 speed) // Line following, but backwards
{
    if(speed > 110) speed = 110; // High speeds are too devastating for backtracking, so speed here is not bigger then 110
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 0 && dig.R3 == 0 ) // We are on the line, so go backwards
    {
        motor_backward( speed, 0 );
        return 0;
    }
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 1 && dig.R3 == 1 ) // it is a point/start, so go backwards
    {
        motor_backward( speed, 0 );
        return 0;
    }
    if( dig.L3 == 1 && dig.L2 == 1 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 0 && dig.R3 == 0 ) // the same as above
    {
        motor_backward( speed, 0 );
        return 0;
    }
    if( dig.L3 == 1 && dig.L2 == 1 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 1 && dig.R3 == 1 ) // a point, so go backwards
    {
        motor_backward( speed, 0 );
        return 0;
    }
    // turns
    if( dig.L3 == 0 && dig.L2 == 1 && dig.L1 == 1 && dig.R1 == 0 && dig.R2 == 0 && dig.R3 == 0 ) // turn left
    {
        motor_turn_backwards( 0, speed, 0 ); // turning left
        return 0;
    }
    if( dig.L3 == 0 && dig.L2 == 1 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 0 && dig.R3 == 0 ) // turn right
    {
        motor_turn_backwards( speed, 0, 0 ); // turning right
        return 0;
    }
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 0 && dig.R1 == 1 && dig.R2 == 1 && dig.R3 == 0 ) // turn right
    {
        motor_turn_backwards( speed, 0, 0 ); // turning right
        return 0;
    }
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 1 && dig.R3 == 0 ) // turn left
    {
        motor_turn_backwards( 0, speed, 0 ); // turning left
        return 0;
    }

    // Something went wrong
    if( dig.L3 == 1 && dig.L2 == 1 && dig.L1 == 1 && dig.R1 == 0 && dig.R2 == 0 && dig.R3 == 0 ) // well, it can be turn left
    {
        motor_turn_backwards( 0, speed, 0 ); // turning left
        return 0;
    }
    if( dig.L3 == 1 && dig.L2 == 1 && dig.L1 == 0 && dig.R1 == 0 && dig.R2 == 0 && dig.R3 == 0 ) // well, it can be turn left
    {
        motor_turn_backwards( 0, speed, 0 ); // turning left
        return 0;
    }
    if( dig.L3 == 1 && dig.L2 == 0 && dig.L1 == 0 && dig.R1 == 0 && dig.R2 == 0 && dig.R3 == 0 ) // Turn left so messed up?
    {
        motor_turn_backwards( 0, speed, 0 ); // turning left
        return 0;
    }
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 0 && dig.R1 == 1 && dig.R2 == 1 && dig.R3 == 1 ) // well, it can be turn right
    {
        motor_turn_backwards( speed, 0, 0 ); // turning right
        return 0;
    }
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 0 && dig.R1 == 0 && dig.R2 == 1 && dig.R3 == 1 ) // well, it can be turn right
    {
        motor_turn_backwards( speed, 0, 0 ); // turning right
        return 0;
    }
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 0 && dig.R1 == 0 && dig.R2 == 0 && dig.R3 == 1 ) // Turn right so messed up?
    {
        motor_turn_backwards( speed, 0, 0 ); // turning right
        return 0;
    }
    // Allignment messed up, must fix it:
    if( dig.L3 == 0 && dig.L2 == 1 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 1 && dig.R3 == 1 ) // Just small fix to the left
    {
        motor_turn_backwards( 0, speed, 0 ); // turning left
        return 0;
    }
    if( dig.L3 == 1 && dig.L2 == 1 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 1 && dig.R3 == 0 ) // Just small fix to the right
    {
        motor_turn_backwards( speed, 0, 0 ); // turning right
        return 0;
    }
    // OUT
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 0 && dig.R1 == 0 && dig.R2 == 0 && dig.R3 == 0 ) // lost the line. There is no point in searching for it
    {
        motor_forward( 0, 0 );
        return 1;
    }
    return 0;
}

///////////////////////////////////////////////////////////////////////////
// Turn functions //
///////////////////////////////////////////////////////////////////////////

static void maze_motor_tank_turn_left( uint8 speed, uint32 delay, int8 *robotdir )
{
    if ( speed > 110 ) speed = 110; // High speeds are too devastating for the tank turns, so speed here is not bigger then 110
    struct sensors_ dig;
    bool left_the_line = false;
    reflectance_digital(&dig);
    while( ( !is_on_track(dig) ) || !left_the_line ) // robot sensors must leave the track, get 2 middle sensors on the line on his left
    {
        motor_tank_turn_left( speed, 0 );
        reflectance_digital(&dig);
        if( !is_on_track(dig) ) left_the_line = true;
    }
    motor_forward( 0, delay );
    if( *robotdir == 3 ) *robotdir = 0; // direction adjustment
    else *robotdir = *robotdir + 1;
}

static void maze_motor_tank_turn_right( uint8 speed, uint32 delay, int8 *robotdir )
{
    if ( speed > 110 ) speed = 110; // High speeds are too devastating for the tank turns, so speed here is not bigger then 110
    struct sensors_ dig;
    bool left_the_line = false;
    reflectance_digital(&dig);
    while( ( !is_on_track(dig) ) || !left_the_line ) // robot sensors must leave the track, get 2 middle sensors on the line on his right
    {
        motor_tank_turn_right( speed, 0 );
        reflectance_digital(&dig);
        if( !is_on_track(dig) ) left_the_line = true;
    }
    motor_forward( 0, delay );
    if( *robotdir == 0 ) *robotdir = 3; // direction adjustment
    else *robotdir = *robotdir - 1;
}

static void maze_motor_turn_left( uint8 speed, uint32 delay, int8 *robotdir )
{
    struct sensors_ dig;
    bool turning = true;
    reflectance_digital(&dig);
    motor_turn( speed/8 , speed, 0 );
    while(turning || dig.R1 == 0) // while we started turning, or we alrady met the needed line
    {
        reflectance_digital(&dig);
        if( dig.R1 == 0 ) turning = false; // This can be the issue if our turn was too sharp - Sensor won't leave the track, means we won't stop turning
    }
    if(DEBUGMODE) printf("Stop turning left!\n");
    motor_forward( 0, delay );
    if( *robotdir == 3 ) *robotdir = 0; // direction adjustment
    else *robotdir = *robotdir + 1;
}

static void maze_motor_turn_right( uint8 speed, uint32 delay, int8 *robotdir )
{
    struct sensors_ dig;
    bool turning = true;
    reflectance_digital(&dig);
    motor_turn( speed , speed/8, 0 );
    while(turning || dig.L1 == 0) // while we started turning, or we alrady met the needed line
    {
        reflectance_digital(&dig);
        if( dig.L1 == 0 ) turning = false; // This can be the issue if our turn was too sharp - Sensor won't leave the track, means we won't stop turning
    }
    if(DEBUGMODE) printf("Stopped turning right!\n");
    motor_forward( 0, delay );
    if( *robotdir == 0 ) *robotdir = 3; // direction adjustment
    else *robotdir = *robotdir - 1;
}
/********************//********************//********************/
/********************//********************//********************/
/********************//********************//********************/
/********************//* USEFUL FUNCTIONS *//********************/
/********************//********************//********************/
/********************//********************//********************/
/********************//********************//********************/

// setting up speed using % ( 0 to 100)
int percent_speed( uint8 percent )
{
    return (percent*SPEED/100);
}
// counts delay for the 90 deg tank turn
int count_delay( uint32 for_max_speed )
{
    return ((for_max_speed*255)/SPEED);
}

// tank turns
void motor_tank_turn_left(uint8 speed,uint32 delay)
{
    SetMotors(1, 0, speed, speed, delay);
}

void motor_tank_turn_right(uint8 speed,uint32 delay)
{
    SetMotors(0, 1, speed, speed, delay);
}

// turning backwards
void motor_turn_backwards(uint8 l_speed, uint8 r_speed, uint32 delay)
{
    SetMotors( 1, 1, l_speed, r_speed, delay );
}

// Counted almost* 90 degree turns
void motor_tank_turn_90_left(void)
{ 
    motor_tank_turn_left(SPEED, count_delay(102));
    motor_tank_turn_left( 15 + ( SPEED / 43 ), 10 ); // Some overcomplicated formulas in order to get rid of flawes during calculations
    motor_forward( 0, 0 );
}

void motor_tank_turn_90_right(void)
{   
    motor_tank_turn_right(SPEED, count_delay(102));
    motor_tank_turn_right( 15 + ( SPEED / 43 ), 10 );
    motor_forward( 0, 0 );
}

// ----- Line following ----- //

// checks if robot is standing on black line
bool is_on_black_line(struct sensors_ dig)
{
    if ( dig.L1 == 1 && dig.R1 == 1 && dig.L2 == 1 && dig.L3 == 1 && dig.R2 == 1 && dig.R3 == 1 )
    {
        return true;
    }
    return false;
}
// checks if robot is standing on track
bool is_on_track(struct sensors_ dig)
{
    if( dig.L1 == 1 && dig.R1 == 1 )
    {
        return true;
    }
    return false;
}
// checks if robot is only on track ( only two middle sensors are on line )
bool is_on_line(struct sensors_ dig)
{
    if( dig.L3 == 0 && dig.L2 == 0 && dig.L1 == 1 && dig.R1 == 1 && dig.R2 == 0 && dig.R3 == 0 )
    {
        return true;
    }
    return false;
}

void go_to_the_first_line(struct sensors_ dig)
{   
    reflectance_digital(&dig);
    while (!is_on_track(dig))
    {
        reflectance_digital(&dig);
        printf("Put robot on the track, please. ( Two middle sensors should be on the track )\n");
        vTaskDelay(5000);
    }
    motor_forward( SPEED, 0 );
    do
    {
        reflectance_digital(&dig);
        if ( is_on_black_line(dig) )
        {
            motor_forward( 0, 0 );
        }
    }while( !is_on_black_line(dig) );
}

void start_IR_refl_motor(void)
{
    // Starting IR sensor
    IR_Start();
    if(DEBUGMODE) printf("IR test\n");
    IR_flush(); // clear IR receive buffer
    if(DEBUGMODE) printf("Buffer cleared\n");
    // Starting reflectance sensors
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000);
    
    motor_start();
    motor_forward( 0, 0 );
    if(DEBUGMODE) printf("Motor started.\n");
}

/* [] END OF FILE */
