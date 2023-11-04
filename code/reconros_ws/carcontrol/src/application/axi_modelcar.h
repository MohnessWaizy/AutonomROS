/********************************************************************          
* axi_modelcar.h         -- user space driver for the axi modelcar  *
*                        	ip-core			                        *
*                                                                   *  
* Author(s):  Christian Lienen                                      *   
*                                                                   *   
********************************************************************/

#ifndef AXI_MODELCAR_H
#define AXI_MODELCAR_H

#include <stdint.h>

typedef struct __attribute__((__packed__)) 
{
	volatile uint32_t	Servo_0_Ctrl_Reg;
	volatile uint32_t 	Servo_0_PWM_Reg;
	volatile uint32_t	Servo_1_Ctrl_Reg;
	volatile uint32_t 	Servo_1_PWM_Reg;
	volatile uint32_t 	Trigger_Ctrl_Reg;
	volatile uint32_t   Echo;
	volatile uint32_t   Hall_Sensor_Interval;
}t_axi_modelcar;


t_axi_modelcar * axi_modelcar_init( uint64_t base_addr);

#endif
