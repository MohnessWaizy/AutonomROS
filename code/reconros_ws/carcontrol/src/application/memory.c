/********************************************************************          
* memory.c           -- memory initialization; used by all other    *
*                       userspace drivers which need to mmap areas  *
*                                                                   *  
* Author(s):  Christian Lienen                                      *   
*                                                                   *   
********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>

#include "memory.h"

int memory_init()
{
	memfd = open("/dev/mem", O_RDWR | O_SYNC);
	if (memfd < 0) {
		printf("[MEMORY] ERROR: Could not open /dev/mem\n");
		close(memfd);
		return -1;
	}
	return 0;
}

void memory_deinit()
{
	close(memfd);
}
