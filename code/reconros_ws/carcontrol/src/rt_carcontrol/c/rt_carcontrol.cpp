extern "C" {
#include "reconos_thread.h"
#include "reconos_calls.h"

#include "../application/axi_modelcar.h"
}
#define DEBUG_LANE_CONTROL 0
#define NO_SPEED 150000
#define MAX_SPEED_F 180000
#define MAX_SPEED_B 120000
#define FULL_LEFT 100000
#define FULL_RIGHT 200000
#define NEUTRAL 150000
#define BREAK_VALUE 130000

extern "C" THREAD_ENTRY(); // this is required because of the mixture of c and c++

THREAD_ENTRY() {
	
        t_axi_modelcar *axi_modelcar = axi_modelcar_init(0xA0000000);

        axi_modelcar->Servo_1_Ctrl_Reg = 1;
        axi_modelcar->Servo_0_Ctrl_Reg = 1;
        float speed = NO_SPEED;
        float angle = NEUTRAL;
        while(1)
        {
            ROS_SUBSCRIBE_TAKE(rcarcontrol_subdrivecmd, rcarcontrol_drive_cmd);
            if(DEBUG_LANE_CONTROL)
                printf("[Car Control] subscribe done: %f\n", rcarcontrol_drive_cmd->linear.x);
            // Apply different mapping for speed depending on the sign of the speed
            if (rcarcontrol_drive_cmd->linear.x < 0){
                speed = rcarcontrol_drive_cmd->linear.x * (NO_SPEED-MAX_SPEED_B) + NO_SPEED;
            }else{
                speed = rcarcontrol_drive_cmd->linear.x * (MAX_SPEED_F-NO_SPEED) + NO_SPEED;
            }
            if (speed > MAX_SPEED_F){
                speed = MAX_SPEED_F;
            }
            else if (speed < MAX_SPEED_B){
                speed = MAX_SPEED_B;
            }
            // Map the angle to the steering
            angle = rcarcontrol_drive_cmd->angular.z / -1.2;
            if(angle > 0){
                angle = angle * (FULL_RIGHT-NEUTRAL) + NEUTRAL;
            }else{
                angle = angle * (NEUTRAL-FULL_LEFT) + NEUTRAL;
            }
            if (angle < FULL_LEFT){
                angle = FULL_LEFT;
            }
            else if (angle > FULL_RIGHT){
                angle = FULL_RIGHT;
            }
            axi_modelcar->Servo_1_PWM_Reg = (int) speed;
            axi_modelcar->Servo_0_PWM_Reg = (int) angle;
        }
    	return;
}
