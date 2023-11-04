#include "reconos_thread.h"
#include "reconos_calls.h"

#include <stdint.h>
#include <sensor_msgs/msg/camera_info.h>

#define INPUT_WIDTH 640
#define INPUT_HEIGHT 480
#define DEPTH_PIXEL_SIZE 2
#define POINT_STEP 16

#define POINTS_PER_CYCLE 160
#define DEPTH_INPUT_LINESIZE DEPTH_PIXEL_SIZE * POINTS_PER_CYCLE
#define OUTPUT_LINESIZE POINT_STEP * POINTS_PER_CYCLE
#define LOAD_CYCLES (INPUT_WIDTH * INPUT_HEIGHT) / POINTS_PER_CYCLE

struct p_matrix
{
  float fx;
  float cx;
  float fy;
  float cy;
};

union float_raw {
  float f;
  uint32_t i;
};

union double_raw {
  double d;
  uint64_t i;
};

struct point_cloud_point
{
  union float_raw x;
  union float_raw y;
  union float_raw z;
  uint32_t _padding;
};

void calculate_point(
  int pixel, struct p_matrix p_matrix, uint16_t depth_pixel,
  struct point_cloud_point * point)
{
  float depthX = (pixel % INPUT_WIDTH);
  float depthY = (pixel / INPUT_WIDTH);
  float depthZ = depth_pixel;

  float u = depthX * depthZ;
  float v = depthY * depthZ;

  float x = (u - (p_matrix.cx * depthZ)) / p_matrix.fx;
  float y = (v - (p_matrix.cy * depthZ)) / p_matrix.fy;

  point->x.f = depthZ * 0.001f;
  point->y.f = -x * 0.001f;
  point->z.f = -y * 0.001f;
}

THREAD_ENTRY()
{
  iceoryx_msg__msg__Image640mono * mono_image = NULL;
  iceoryx_msg__msg__PointCloud640 * point_cloud = NULL;

  uint64_t depth_message;

  uint64_t info_message;
  uint64_t info_payload_addr[1];
  uint64_t p_matrix_buffer[7];
  struct p_matrix p_matrix;

  uint64_t output_buffer_addr;

  THREAD_INIT();

  // We know it will not change, so just load it once and save time
  ROS_SUBSCRIBE_TAKE(rpointcloud_depth_info_sub, rpointcloud_depth_info_msg);
  MEM_READ(rpointcloud_depth_info_msg->p, p_matrix_buffer, 7 * 8);

  double p_matrix_double[7];
  matrix_loop: for (int i = 0; i < 7; ++i) {
    union double_raw matrix_element;
    matrix_element.i = p_matrix_buffer[i];
    p_matrix_double[i] = matrix_element.d;
  }

  p_matrix.fx = p_matrix_double[0];
  p_matrix.cx = p_matrix_double[2];
  p_matrix.fy = p_matrix_double[5];
  p_matrix.cy = p_matrix_double[6];

  run_loop: while (true) {
    uint64_t depth_input_linebuffer[DEPTH_INPUT_LINESIZE / 8];
    uint64_t output_linebuffer[OUTPUT_LINESIZE / 8];

    depth_message = ROS_SUBSCRIBE_TAKE_LOANED(rpointcloud_depth_image_sub, (void **) &mono_image);
    ROS_BORROW(rpointcloud_point_cloud_pub, (void **) &point_cloud);
    output_buffer_addr = &point_cloud->data;

    MEM_READ(
      mono_image->data, depth_input_linebuffer,
      DEPTH_INPUT_LINESIZE);

    read_loop: for (int i = 0; i < LOAD_CYCLES; ++i) {
      if (i > 0) {
        MEM_WRITE(
          output_linebuffer, (output_buffer_addr + (i - 1) * OUTPUT_LINESIZE),
          OUTPUT_LINESIZE);
      }

      /*
       * Extract 16 bit (2 byte) mono16 pixels out of 64 bit (8 byte) integers
       */
      uint16_t depth_pixels[POINTS_PER_CYCLE];
      depth_loop: for (int j = 0; j < DEPTH_INPUT_LINESIZE / 8; ++j) {
        depth_pixels[4 * j] = (uint16_t) (depth_input_linebuffer[j]);
        depth_pixels[4 * j + 1] = (uint16_t) (depth_input_linebuffer[j] >> 16);
        depth_pixels[4 * j + 2] = (uint16_t) (depth_input_linebuffer[j] >> 32);
        depth_pixels[4 * j + 3] = (uint16_t) (depth_input_linebuffer[j] >> 48);
      }

      if (i + 1 < LOAD_CYCLES) {
        MEM_READ(
          mono_image->data + (i + 1) * DEPTH_INPUT_LINESIZE, depth_input_linebuffer,
          DEPTH_INPUT_LINESIZE);
      }

      calc_loop: for (int j = 0; j < POINTS_PER_CYCLE; ++j) {
        int pixel = i * POINTS_PER_CYCLE + j;
        struct point_cloud_point point;
        calculate_point(
          pixel,
          p_matrix,
          depth_pixels[j],
          &point
        );

        /*
         * Put the point into the following structure:
         *   float x;
         *   float y;
         *   float z;
         *   uint8_t b;   // little endian argb
         *   uint8_t g;
         *   uint8_t r;
         *   uint8_t a;
         */
        output_linebuffer[j * POINT_STEP / 8] =
          (((uint64_t) point.y.i) << 32) |
          (((uint64_t) point.x.i));
        output_linebuffer[j * POINT_STEP / 8 + 1] = ((uint64_t) point.z.i);
      }
    }
    MEM_WRITE(
      output_linebuffer, (output_buffer_addr + (LOAD_CYCLES - 1) * OUTPUT_LINESIZE),
      OUTPUT_LINESIZE);
    ROS_PUBLISH_LOANED(rpointcloud_point_cloud_pub, point_cloud);
    ROS_SUBSCRIBE_RETURN_LOANED(rpointcloud_depth_image_sub, mono_image);
    mono_image = NULL;
  }
}
