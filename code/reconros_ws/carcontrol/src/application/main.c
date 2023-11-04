#define  _GNU_SOURCE

#define RECONOS_DEBUG

#define DEFAULT_IMAGE_HEIGHT 480
#define DEFAULT_IMAGE_WIDTH  640
#define NUM_POINTS 30

#define SETUP 2

#include "reconos.h"
#include "reconos_app.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "memory.h"
#include "axi_modelcar.h"

#include "main.h"

#include "pointcloud_init.h"

static bool running_ = true;

/*
void polyfit_init_msg(void)
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
*/
void init_msg(void)
{
	rultrasound_range_msg->header.frame_id.data = malloc(16 * sizeof(char));
	strncpy(rultrasound_range_msg->header.frame_id.data, "ultrasound_link\0", 16);
	rultrasound_range_msg->header.frame_id.size = 15;
	rultrasound_range_msg->header.frame_id.capacity = 16;
	rultrasound_range_msg->radiation_type = 0;
	rultrasound_range_msg->field_of_view = 0.366;
	rultrasound_range_msg->min_range = 0.05;
	rultrasound_range_msg->max_range = 4;

	rhallsensor_speed_msg->header.frame_id.data = malloc(19 * sizeof(char));
	strncpy(rhallsensor_speed_msg->header.frame_id.data, "wheel_encoder_link\0", 19);
	rhallsensor_speed_msg->header.frame_id.size = 18;
	rhallsensor_speed_msg->header.frame_id.capacity = 19;
	rhallsensor_speed_msg->twist.covariance[0] = 0.00;
	/*
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

	polyfit_init_msg();*/
}

static void exit_signal(int sig) 
{
	running_ = false;
	printf("[ReconROS] aborted\n");
}

int main(int argc, char **argv) 
{
	memory_init();
	reconos_init();
	reconos_app_init();
	init_msg();
	pointcloud_init_msg();

	signal(SIGINT, exit_signal);
	signal(SIGTERM, exit_signal);
	signal(SIGABRT, exit_signal);

	reconos_thread_create_swt_carcontrol(0,0);
	reconos_thread_create_swt_ultrasound(0,0);
	reconos_thread_create_swt_hallsensor(0,0);
	reconos_thread_create_hwt_laneplanner(0);
	reconos_thread_create_hwt_polyfit(0);
    reconos_thread_create_hwt_pointcloud(NULL);

	while(running_)
	{
		sleep(100);
	}

	reconos_app_cleanup();
	reconos_cleanup();
	memory_deinit();
	return 0;
}
