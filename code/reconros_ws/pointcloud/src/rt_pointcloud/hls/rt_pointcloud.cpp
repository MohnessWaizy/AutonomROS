#include "reconos_thread.h"
#include "reconos_calls.h"

#include <stdint.h>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/camera_info.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <iceoryx_msg/msg/image640rgb.h>
#include <iceoryx_msg/msg/image640mono.h>

#include "ap_int.h"
#include "ap_fixed.h"

#define INPUT_WIDTH 640
#define INPUT_HEIGHT 480
#define DEPTH_PIXEL_SIZE 2
#define POINT_STEP 16

#define POINTS_PER_CYCLE 320
#define DEPTH_INPUT_LINESIZE DEPTH_PIXEL_SIZE * POINTS_PER_CYCLE
#define LOAD_CYCLES (INPUT_WIDTH * INPUT_HEIGHT) / POINTS_PER_CYCLE

#define GRID_HEIGHT 18
#define GRID_WIDTH 13

const float transformation_matrix[4][4] = {
  {0.98875, 0.0, 0.14955, -0.15500},
  {0.0, 1.0, 0.0, 0.00000},
  {-0.14955, 0.0, 0.98875, 0.26400},
  {0.00000, 0.00000, 0.00000, 1.00000}
};

const float obstacle_box[3][2] = {
  {0.3, 1.2}, // x_min, x_max
  {-0.325, 0.325}, // y_min, y_max
  {0.02, 0.5} // z_min, z_max
};

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

  float tmp_x = p_matrix.cx * depthZ;
  float tmp_y = p_matrix.cy * depthZ;
  float tmp_x_1 = u - tmp_x;
  float tmp_y_1 = v - tmp_y;

  float x = tmp_x_1 / p_matrix.fx;
  float y = tmp_y_1 / p_matrix.fy;

  float scale = 1.0 / 1024;
  float scaled_x = depthZ * scale;
  float scaled_y = -x * scale;
  float scaled_z = -y * scale;

  point->x.f = scaled_x;
  point->y.f = scaled_y;
  point->z.f = scaled_z;
}

void depth_camera_link_to_base_link(struct point_cloud_point * in, struct point_cloud_point * out)
{
  out->x.f = transformation_matrix[0][0] * in->x.f + transformation_matrix[0][1] * in->y.f +
    transformation_matrix[0][2] * in->z.f + transformation_matrix[0][3] * 1;

  out->y.f = transformation_matrix[1][0] * in->x.f + transformation_matrix[1][1] * in->y.f +
    transformation_matrix[1][2] * in->z.f + transformation_matrix[1][3] * 1;

  out->z.f = transformation_matrix[2][0] * in->x.f + transformation_matrix[2][1] * in->y.f +
    transformation_matrix[2][2] * in->z.f + transformation_matrix[2][3] * 1;
}

THREAD_ENTRY()
{

  uint64_t depth_message;
  uint64_t depth_payload_addr[1];

  uint8_t grid[GRID_WIDTH * GRID_HEIGHT + 6]; // 13 x 18 = 234 plus 6 to get to 30 * 64

  uint64_t info_message;
  uint64_t info_payload_addr[1];
  uint64_t p_matrix_buffer[7];
  struct p_matrix p_matrix;

  uint64_t output_buffer_addr;

  THREAD_INIT();

  // We know it will not change, so just load it once and save time
  info_message = ROS_SUBSCRIBE_TAKE(rpointcloud_depth_info_sub, rpointcloud_depth_info_msg);
  MEM_READ(OFFSETOF(sensor_msgs__msg__CameraInfo, p) + info_message, p_matrix_buffer, 7 * 8);

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
    grid_loop: for (int i = 0; i < GRID_WIDTH * GRID_HEIGHT + 6; ++i) {
      grid[i] = 0;
    }

    uint64_t depth_input_linebuffer[DEPTH_INPUT_LINESIZE / 8];

    ROS_SUBSCRIBE_TAKE_LOANED(rpointcloud_depth_image_sub, depth_message);
    depth_payload_addr[0] = OFFSETOF(iceoryx_msg__msg__Image640mono, data) + depth_message;

    MEM_READ(
      depth_payload_addr[0], depth_input_linebuffer,
      DEPTH_INPUT_LINESIZE);

    read_loop: for (int i = 0; i < LOAD_CYCLES; ++i) {
      /*
       * Extract 16 bit (2 byte) mono16 pixels out of 64 bit (8 byte) integers
       */
      uint16_t depth_pixels[POINTS_PER_CYCLE];
#pragma HLS ARRAY_PARTITION variable=depth_pixels type=cyclic factor=4
      depth_loop: for (int j = 0; j < DEPTH_INPUT_LINESIZE / 8; ++j) {
        depth_pixels[4 * j] = (uint16_t) (depth_input_linebuffer[j]);
        depth_pixels[4 * j + 1] = (uint16_t) (depth_input_linebuffer[j] >> 16);
        depth_pixels[4 * j + 2] = (uint16_t) (depth_input_linebuffer[j] >> 32);
        depth_pixels[4 * j + 3] = (uint16_t) (depth_input_linebuffer[j] >> 48);
      }

      if (i + 1 < LOAD_CYCLES) {
        MEM_READ(
          depth_payload_addr[0] + (i + 1) * DEPTH_INPUT_LINESIZE, depth_input_linebuffer,
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

        struct point_cloud_point point_base_link;
        depth_camera_link_to_base_link(
          &point,
          &point_base_link
        );

        if (
          point_base_link.x.f >= obstacle_box[0][0] &&
          point_base_link.x.f <= obstacle_box[0][1] &&
          point_base_link.y.f >= obstacle_box[1][0] &&
          point_base_link.y.f <= obstacle_box[1][1] &&
          point_base_link.z.f >= obstacle_box[2][0] &&
          point_base_link.z.f <= obstacle_box[2][1])
        {
          float x = point_base_link.x.f - obstacle_box[0][0];
          float y = point_base_link.y.f - obstacle_box[1][0];

          uint8_t gx = x / 0.05;
          uint8_t gy = y / 0.05;
          uint8_t index = gx * GRID_WIDTH + gy;

          if (index < 0 || index >= GRID_WIDTH * GRID_HEIGHT) {
            continue;
          }

          if (grid[index] < 255) {
            grid[index] += 1;
          }
        }
      }
    }

    ROS_SUBSCRIBE_RETURN_LOANED(rpointcloud_depth_image_sub, depth_message);

    ROS_BORROW(rpointcloud_obstacle_pub, output_buffer_addr);
    uint64_t out[30];
    for (int i = 0; i < 30; ++i) {
      out[i] =
        ((uint64_t) grid[i * 8 + 7]) << 56 |
        ((uint64_t) grid[i * 8 + 6]) << 48 |
        ((uint64_t) grid[i * 8 + 5]) << 40 |
        ((uint64_t) grid[i * 8 + 4]) << 32 |
        ((uint64_t) grid[i * 8 + 3]) << 24 |
        ((uint64_t) grid[i * 8 + 2]) << 16 |
        ((uint64_t) grid[i * 8 + 1]) << 8 |
        ((uint64_t) grid[i * 8 + 0]);
    }

    MEM_WRITE(out, output_buffer_addr, 30 * 8);
    ROS_PUBLISH_LOANED(rpointcloud_obstacle_pub, output_buffer_addr);
  }
}
