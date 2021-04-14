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
#include "line.h"
#define LINE_COUNT 3

void line(void)
{

    struct sensors_ dig;
    start_IR_refl_motor();
    TickType_t start, end;
    int time_elapsed;
    bool lineMiss = true;
    bool left_line = true; 
    int count = 0;
   

    btn_press(); // This function is the switch function.;
    reflectance_digital(&dig);
    go_to_the_first_line(dig); // This function leads the robot to the first crossing

    motor_forward(0, 0);
    print_mqtt("Zumo05/ready", "line"); // Printing MQTT
    IR_wait();
    vTaskDelay(100);
    start = xTaskGetTickCount();
    print_mqtt("Zumo05/start", "%d", start); // Printing MQTT
    motor_forward(SPEED, 0);

    while (count != LINE_COUNT)
    {

        reflectance_digital(&dig);
        if (is_on_black_line(dig) && left_line)
        {
            left_line = false;
            motor_forward(SPEED , 0);
            vTaskDelay(50);
            count++; //It counts and add the lines detected by the sensors.

            if (count == LINE_COUNT) //The robot stops on the third line
            {
                halt_motor(); // This function stops the motor.
                end = xTaskGetTickCount();
                print_mqtt("Zumo05/stop", "%d", end); // Printing MQTT
                time_elapsed = end - start;
                print_mqtt("Zumo05/time", "%d", time_elapsed); // Printing MQTT
            }
        }
        else if (! is_on_black_line(dig))
        {
            left_line = true;
        }
        reflectance_digital(&dig);
        line_movement(dig);        // This function is line following algorithm.
        line_miss(dig, &lineMiss); //  This function detect the line miss and found by the middle sensors (L1 & R1)
    }
}

//**********************************//
//--- Function for line movement ---//
//**********************************//

void line_movement(struct sensors_ dig)
{
    // If the robot is out of track, turn back and search the line.

    if (dig.R1 == 0 && dig.R2 == 0 && dig.R3 == 0 && dig.L1 == 0 && dig.L2 == 0 && dig.L3 == 0)
    {
        motor_backward(SPEED * 0.8, 0);
        // printf("You missed the line turn back and search.");
    }

    //Forward movement, central sensors are active.

    if (dig.L1 == 1 && dig.R1 == 1 && dig.L3 == 0 && dig.R3 == 0)
    {
        motor_forward(SPEED, 0);
    }
    //---- Left Oriented Movements ----//

    //-- Slight left --//
    if (dig.L1 == 1 && dig.L3 == 0 && dig.R3 == 0 && dig.R1 == 0)
    {
        motor_turn(0, SPEED, 0);
        reflectance_digital(&dig);
    }
    //-- Normal left --//
    if (dig.L2 == 1 && dig.L1 == 1 && dig.L3 == 0 && dig.R3 == 0)
    {
        motor_turn(0, SPEED, 0);
    }
    //-- Hard left --//

    if (dig.L3 == 1 && dig.L2 == 1 && dig.R2 == 0 && dig.R3 == 0)
    {
        motor_turn(0, SPEED, 0);
    }

    //-- Super Hard Left --//

    if (dig.L3 == 1 && dig.R3 == 0) // Very sharp left turn ahead
    {
        motor_turn(0, SPEED, 0);
    }
    //---- Right Oriented Movements ----//

    //-- Slight right --//
    if (dig.R1 == 1 && dig.L3 == 0 && dig.R3 == 0 && dig.L1 == 0)
    {
        motor_turn(SPEED, 0, 0);
    }
    //-- Normal right --//
    if (dig.R1 == 1 && dig.R2 == 1 && dig.L3 == 0 && dig.L2 == 0)
    {
        motor_turn(SPEED, 0, 0);
    }
    //-- Hard right --//
    if (dig.R2 == 1 && dig.R3 == 1 && dig.L2 == 0 && dig.L3 == 0)
    {
        motor_turn(SPEED,0, 0);
    }
    //-- Super Hard right --//
    if (dig.R3 == 1 && dig.L3 == 0) // Very sharp right turn ahead
    {
        motor_turn(SPEED, 0, 0);
    }
}

//*********************************//
//--- Function for button press ---//
//*********************************//

void btn_press(void)
{

    printf("\nPlease press the button.\n");
    while (SW1_Read())
        ;                //Call for the button to be pressed.
    BatteryLed_Write(1); // The led turns to blue after the button is pressed.
    vTaskDelay(1000);
    BatteryLed_Write(0); // The Led turn off after 1 sec.
    vTaskDelay(500);
}

//***********************************//
//--- Function for stopping motor ---//
//***********************************//

void halt_motor(void)
{
    motor_forward(0, 0);
    motor_stop();
}

//****************************************//
//--- Function for line miss and found ---//
//****************************************//

void line_miss(struct sensors_ dig, bool *lineMiss)
{

    TickType_t lineTime, missTime;

    if (*lineMiss == false && dig.L1 == 1 && dig.R1 == 1)
    {
        lineTime = xTaskGetTickCount();
        print_mqtt("Zumo05/line", "%d", lineTime); //Printing MQTT for the line found
        *lineMiss = true;
    }
    else if (*lineMiss == true && dig.L1 == 0 && dig.R1 == 0)
    {
        missTime = xTaskGetTickCount();
        print_mqtt("Zumo05/miss", "%d", missTime); //Printing MQTT for the line miss
        *lineMiss = false;
    }
}

/* [] END OF FILE */
