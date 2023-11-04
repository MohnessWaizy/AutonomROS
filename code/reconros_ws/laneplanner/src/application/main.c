#define  _GNU_SOURCE

#include "reconos.h"
#include "reconos_app.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sensor_msgs/msg/point_field.h>

#include "main.h"


#define DEFAULT_IMAGE_HEIGHT 480
#define DEFAULT_IMAGE_WIDTH  640
#define NUM_POINTS 30


void pointcloud_init_msg(void)
{
  rpolyfit_poly_image_msg->header.frame_id.data = malloc(sizeof(char) * 10);
  strncpy(rpolyfit_poly_image_msg->header.frame_id.data, "base_link\0", 10);
  rpolyfit_poly_image_msg->header.frame_id.size = 9;
  rpolyfit_poly_image_msg->header.frame_id.capacity = 10;
  rpolyfit_poly_image_msg->width = NUM_POINTS;
  rpolyfit_poly_image_msg->height = 1;

  size_t fields_size = sizeof(sensor_msgs__msg__PointField) * 3;
  rpolyfit_poly_image_msg->fields.data = malloc(fields_size);
  rpolyfit_poly_image_msg->fields.size = 3;
  rpolyfit_poly_image_msg->fields.capacity = 3;

  sensor_msgs__msg__PointField * x_field =
    (sensor_msgs__msg__PointField *) rpolyfit_poly_image_msg->fields.data;
  x_field->name.data = malloc(sizeof(char) * 2);
  strncpy(x_field->name.data, "x\0", 2);
  x_field->name.size = 1;
  x_field->name.capacity = 2;
  x_field->count = 1;
  x_field->datatype = sensor_msgs__msg__PointField__FLOAT32;
  x_field->offset = 0;

  sensor_msgs__msg__PointField * y_field = x_field + 1;
  y_field->name.data = malloc(sizeof(char) * 2);
  strncpy(y_field->name.data, "y\0", 2);
  y_field->name.size = 1;
  y_field->name.capacity = 2;
  y_field->count = 1;
  y_field->datatype = sensor_msgs__msg__PointField__FLOAT32;
  y_field->offset = 4;

  sensor_msgs__msg__PointField * z_field = x_field + 2;
  z_field->name.data = malloc(sizeof(char) * 2);
  strncpy(z_field->name.data, "z\0", 2);
  z_field->name.size = 1;
  z_field->name.capacity = 2;
  z_field->count = 1;
  z_field->datatype = sensor_msgs__msg__PointField__FLOAT32;
  z_field->offset = 8;

  rpolyfit_poly_image_msg->point_step = 12;
  rpolyfit_poly_image_msg->row_step = rpolyfit_poly_image_msg->width *
    rpolyfit_poly_image_msg->point_step;
  size_t pointcloud_size = rpolyfit_poly_image_msg->height *
    rpolyfit_poly_image_msg->row_step;
  rpolyfit_poly_image_msg->data.data = malloc(pointcloud_size);
  rpolyfit_poly_image_msg->data.size = pointcloud_size;
  rpolyfit_poly_image_msg->data.capacity = pointcloud_size;
}


void init_msg(void)
{
	rlaneplanner_output_image_msg->height = 480;
	rlaneplanner_output_image_msg->width = 640;
	rlaneplanner_output_image_msg->encoding.data = "mono8";
	rlaneplanner_output_image_msg->encoding.size = 5; 
	rlaneplanner_output_image_msg->encoding.capacity = 6;
	rlaneplanner_output_image_msg->is_bigendian = 0;
	rlaneplanner_output_image_msg->step = 640;
	rlaneplanner_output_image_msg->data.data = malloc(DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH);
	rlaneplanner_output_image_msg->data.size = 480*640;
	rlaneplanner_output_image_msg->data.capacity = DEFAULT_IMAGE_HEIGHT*DEFAULT_IMAGE_WIDTH;

	pointcloud_init_msg();
	
}



static void exit_signal(int sig) 
{
	reconos_cleanup();
	printf("[ReconROS] aborted\n");
	exit(0);
}

int main(int argc, char **argv) 
{
	reconos_init();
	reconos_app_init();

	init_msg();

	signal(SIGINT, exit_signal);
	signal(SIGTERM, exit_signal);
	signal(SIGABRT, exit_signal);
	

	reconos_thread_create_hwt_laneplanner(0);
	reconos_thread_create_hwt_polyfit(0);
	//reconos_thread_create_swt_polyfit(rpolyfit_poly_image_msg->data.data, 0);


	while(1)
	{
		sleep(1);
	}

	reconos_app_cleanup();
	reconos_cleanup();
	return 0;
}
