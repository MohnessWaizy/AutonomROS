extern "C" {
#include "reconos_thread.h"
#include "reconos_calls.h"

#include "../application/axi_modelcar.h"

#include <stdint.h>
#include <iceoryx_msg/msg/point_cloud640.h>
}
extern "C" THREAD_ENTRY(); // this is required because of the mixture of c and c++

#define INPUT_WIDTH 640
#define INPUT_HEIGHT 480
#define POINT_STEP 16

#define POINTS_PER_CYCLE 160
#define POINT_CLOUD_LINESIZE POINT_STEP * POINTS_PER_CYCLE
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

union float_raw {
  float f;
  uint32_t i;
};

struct point_cloud_point
{
  float_raw x;
  float_raw y;
  float_raw z;
  uint32_t _padding;
};

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
  iceoryx_msg__msg__PointCloud640 * point_cloud_message = NULL;

  uint64_t output_buffer_addr;

  uint64_t point_cloud_input_linebuffer[POINT_CLOUD_LINESIZE / 8];
  uint8_t grid[GRID_WIDTH * GRID_HEIGHT + 6]; // 13 x 18 = 234 plus 6 to get to 30 * 64

  THREAD_INIT();

  while (true) {
    for (int i = 0; i < GRID_WIDTH * GRID_HEIGHT + 6; ++i) {
      grid[i] = 0;
    }

    ROS_SUBSCRIBE_TAKE_LOANED(robstacle_point_cloud_sub, (void **) &point_cloud_message);

    for (int i = 0; i < LOAD_CYCLES; ++i) {
      MEM_READ(
        point_cloud_message->data + i * POINT_CLOUD_LINESIZE, point_cloud_input_linebuffer,
        POINT_CLOUD_LINESIZE);

      for (int j = 0; j < POINTS_PER_CYCLE; ++j) {
        struct point_cloud_point * source =
          (struct point_cloud_point *) (point_cloud_input_linebuffer + j * 2);
        struct point_cloud_point out;
        depth_camera_link_to_base_link(source, &out);

        if (
          out.x.f >= obstacle_box[0][0] &&
          out.x.f <= obstacle_box[0][1] &&
          out.y.f >= obstacle_box[1][0] &&
          out.y.f <= obstacle_box[1][1] &&
          out.z.f >= obstacle_box[2][0] &&
          out.z.f <= obstacle_box[2][1])
        {
          float x = out.x.f - obstacle_box[0][0];
          float y = out.y.f - obstacle_box[1][0];

          uint8_t gx = x / 0.05;
          uint8_t gy = y / 0.05;
          uint8_t index = gx * GRID_WIDTH + gy;

          if (index < 0 || index >= GRID_WIDTH * GRID_HEIGHT) {
            printf("Index out of range\n");
            continue;
          }

          if (grid[index] < 255) {
            grid[index] += 1;
          }
        }
      }
    }

    ROS_SUBSCRIBE_RETURN_LOANED(robstacle_point_cloud_sub, point_cloud_message);
    point_cloud_message = NULL;

    ROS_BORROW(robstacle_obstacle_pub, (void **) &output_buffer_addr);

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

    ROS_PUBLISH_LOANED(robstacle_obstacle_pub, (void *) output_buffer_addr);
  }
}
