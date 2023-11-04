/********************************************************************          
* memory.h           -- memory initialization; used by all other    *
*                       userspace drivers which need to mmap areas  *
*                                                                   *  
* Author(s):  Christian Lienen                                      *   
*                                                                   *   
********************************************************************/

#ifndef MEMORY_H
#define MEMORY_H

int memfd;

int     memory_init();
void    memory_deinit();

#else 

extern int memfd;

#endif