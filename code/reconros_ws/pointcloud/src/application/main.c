#include "reconos.h"
#include "reconos_app.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "pointcloud_init.h"

int main(int argc, char ** argv)
{
  reconos_init();
  reconos_app_init();
  pointcloud_init_msg();

  // reconos_thread_create_swt_pointcloud(NULL, 0);
  reconos_thread_create_hwt_pointcloud(NULL);

  while (1) {sleep(10000);}

  reconos_app_cleanup();
  reconos_cleanup();
}
