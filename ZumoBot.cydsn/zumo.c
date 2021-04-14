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
#include "zumo.h"
#include <stdlib.h>

void zumo(void)
{
    struct sensors_ dig;
    TickType_t zumo_start,zumo_stop,zumo_obstacle;
    start_IR_refl_motor();
    Ultra_Start();
    printf("Please press the button.\n");
    while(SW1_Read());
    reflectance_digital(&dig);
    go_to_the_first_line(dig);
    print_mqtt("Zumo05/ready", "zumo");

    printf("Waiting for IR signal\n");
    IR_wait();
    zumo_start =xTaskGetTickCount();
    print_mqtt("Zumo05/start","%d", zumo_start);
    bool left_line = false;
    while (!left_line)
    {
        motor_forward(SPEED,0);
        reflectance_digital(&dig);
        if( dig.L1 == 0 && dig.R1 == 0 && dig.L2 == 0 && dig.L3 == 0 && dig.R2 == 0 && dig.R3 == 0 ){
            left_line = true;

        }
    }
     
    while(SW1_Read()){
        reflectance_digital(&dig);
        motor_forward(SPEED,0);
        if(Ultra_GetDistance() < 10){
            srand(xTaskGetTickCount());
            zumo_obstacle = xTaskGetTickCount();
            print_mqtt("Zumo05/obstacle","%d",zumo_obstacle); 
            if(rand()%2 == 0){
                motor_rand_tank_turn_left(SPEED);
            }
            else{
                motor_rand_tank_turn_right(SPEED);
            }  
        }
        if( dig.L1 == 1 || dig.R1 == 1 || dig.L2 == 1 || dig.L3 == 1 || dig.R2 == 1 || dig.R3 == 1 ){
            if(dig.L1 == 1 || dig.L2 == 1 || dig.L3 == 1){
                motor_rand_tank_turn_right(SPEED);
            }
            else if(dig.R1 == 1 || dig.R2 == 1 || dig.R3 == 1){
                motor_rand_tank_turn_left(SPEED);
            }
        }
    }
    printf("Stop execution!\n");
    motor_forward(0,0);
    zumo_stop =xTaskGetTickCount();
    print_mqtt("Zumo05/stop","%d",zumo_stop);
    print_mqtt("Zumo05/time","%d",zumo_stop-zumo_start);
    motor_stop();


    while(true)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
}

void motor_rand_tank_turn_left(uint8 speed){
    srand(xTaskGetTickCount());
    motor_tank_turn_left( speed, (100*(260 + rand()%261))/speed ); // 260 + 0 to 260
}
void motor_rand_tank_turn_right(uint8 speed){
    srand(xTaskGetTickCount());
    motor_tank_turn_right( speed, (100*(260 + rand()%261))/speed ); // 260 + 0 to 260
}

/* [] END OF FILE */
