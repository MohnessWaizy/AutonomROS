/********************************************************************          
* axi_servo.c            -- user space driver for the axi servo     *
*                        	ip-core			                        *
*                                                                   *  
* Author(s):  Christian Lienen                                      *   
*                                                                   *   
********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>

#include "axi_modelcar.h"
#include "memory.h"

t_axi_modelcar * axi_modelcar_init( uint64_t base_addr)
{
	t_axi_modelcar * axi_modelcar;

	axi_modelcar = (t_axi_modelcar *)mmap(0, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_POPULATE, memfd, base_addr);
	if (axi_modelcar == MAP_FAILED) {
		printf("Error: %s\n", strerror(errno));
		printf("[AXI Modelcar] Init could not map memory\n");
		return NULL;
	}

	return axi_modelcar;
}
