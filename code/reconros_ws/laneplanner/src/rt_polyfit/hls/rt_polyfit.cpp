
#include "reconos_thread.h"
#include "reconos_calls.h"

#include "hls_stream.h"
#include "hls_math.h"
#include <ap_int.h>

#include <MatrixDecomposition/getrf.hpp>
#include <LinearSolver/gelinearsolver.hpp>

#include <iceoryx_msg/msg/image640mono.h>
#include <iceoryx_msg/msg/lane_points.h>

using namespace std;

#define HEIGHT 480
#define WIDTH 640
#define NUM_POINTS 30
#define INCREMENT HEIGHT/NUM_POINTS
#define TYPE_SOLVER double

#define ROWS 3
#define COLS 3


union float_raw {
  float f;
  uint32_t i;
};


void proc(hls::stream<uint64_t> & memif_hwt2mem, hls::stream<uint64_t> & memif_mem2hwt,uint64_t input_message, uint64_t output_message)
{
    uint64_t line_buffer[WIDTH/8];
    
    double A_line[9] = {0};
    double B_line[3] = {0};
    uint64_t n_white=0;
    uint64_t n_yellow=0;
    uint64_t a_mat_vals_white[5] = {0};
    uint64_t b_mat_vals_white[3] = {0};
    uint64_t a_mat_vals_yellow[5] = {0};
    uint64_t b_mat_vals_yellow[3] = {0};

    //Calculates the matrix values with the image coordinates
    generate_matrix: for(uint64_t x = 0; x < HEIGHT; x+=1){
        MEM_READ(input_message + x*WIDTH, line_buffer, WIDTH);
        for(uint64_t y = 0; y < WIDTH/8; y+=1){
        	for(uint64_t g=0; g < 8; g += 1){
                // Get the 8 bit pixel of an 64 bit type
        		uint8_t tmp = (line_buffer[y] >> (g*8)) & 0xff;
				if(tmp == 255){
                    n_white++;
					a_mat_vals_white[0]++;
					a_mat_vals_white[1] += x;
					a_mat_vals_white[2] += x*x;
					a_mat_vals_white[3] += x*x*x;
					a_mat_vals_white[4] += x*x*x*x;
					b_mat_vals_white[0] += y*8+g;
					b_mat_vals_white[1] += x*(y*8+g);
					b_mat_vals_white[2] += x*x*(y*8+g);
                }
                else if(tmp == 127){
                    n_yellow++;
					a_mat_vals_yellow[0]++;
					a_mat_vals_yellow[1] += x;
					a_mat_vals_yellow[2] += x*x;
					a_mat_vals_yellow[3] += x*x*x;
					a_mat_vals_yellow[4] += x*x*x*x;
					b_mat_vals_yellow[0] += y*8+g;
					b_mat_vals_yellow[1] += x*(y*8+g);
					b_mat_vals_yellow[2] += x*x*(y*8+g);
                }
            }
        }
    }
    double factor = 1.0;
    // Matrix values are casted to the actual matrix
    if (n_yellow >= 2000){
        factor = -1.0;
        for(int index=0;index < 9; ++index){
            A_line[index] = (double) a_mat_vals_yellow[index%3+index/3];
        }
        B_line[0] = (double) b_mat_vals_yellow[0];
        B_line[1] = (double) b_mat_vals_yellow[1];
        B_line[2] = (double) b_mat_vals_yellow[2];
    }
    else{
        for(int index=0;index < 9; ++index){
            A_line[index] = (double) a_mat_vals_white[index%3+index/3];
        }
        B_line[0] = (double) b_mat_vals_white[0];
        B_line[1] = (double) b_mat_vals_white[1];
        B_line[2] = (double) b_mat_vals_white[2];
    }


    int info;
    // Solve the equation for the line fit
    xf::solver::gelinearsolver<TYPE_SOLVER, 3, 1>(3, A_line, 1, B_line, 3, 1, info);

    int64_t points_x[NUM_POINTS];
    int64_t points_y[NUM_POINTS];
    // Shift the points towards the middle
    shift_points: for(int i = 0; i < NUM_POINTS; i++){
        int64_t x =  i*INCREMENT;
        int64_t y = B_line[2]*x*x + B_line[1]*x + B_line[0];
        double slope = (double) (2.0 * B_line[2] * x + B_line[1]);

        double x_s =  factor * (slope * 150.0) / hls::sqrt(slope*slope+1);
        double y_s = -(1.0 / slope) * x_s;
        points_x[i] = (int64_t) x + x_s;
        points_y[i] = (int64_t) y + y_s;
    }

    uint64_t a_mat_vals_trajectory[7] = {180, 84600, 39774000, 18705060000, 8799331800000, 4140671706000000, 1949046161340000000};
    uint64_t b_mat_vals_trajectory[4] = {57600, 27072000, 12727680000, 5985619200000};

    for (int i = 0; i < NUM_POINTS; i++){
        a_mat_vals_trajectory[0]++;
        a_mat_vals_trajectory[1] += points_x[i];
        a_mat_vals_trajectory[2] += points_x[i]*points_x[i];
        a_mat_vals_trajectory[3] += points_x[i]*points_x[i]*points_x[i];
        a_mat_vals_trajectory[4] += points_x[i]*points_x[i]*points_x[i]*points_x[i];
        a_mat_vals_trajectory[5] += points_x[i]*points_x[i]*points_x[i]*points_x[i]*points_x[i];
        a_mat_vals_trajectory[6] += points_x[i]*points_x[i]*points_x[i]*points_x[i]*points_x[i]*points_x[i];
        b_mat_vals_trajectory[0] += points_y[i];
        b_mat_vals_trajectory[1] += points_y[i]*points_x[i];
        b_mat_vals_trajectory[2] += points_y[i]*points_x[i]*points_x[i];
        b_mat_vals_trajectory[3] += points_y[i]*points_x[i]*points_x[i]*points_x[i];
    }
    
    double A_trajectory[16];
    double B_trajectory[4];
    int val_index;
    // Matrix values are casted to the actual matrix
    for(int index=0; index < 16; ++index){
        val_index = (index % 4) + (index >> 2);
        A_trajectory[index] = (double) a_mat_vals_trajectory[val_index];
    }
    B_trajectory[0] = (double) b_mat_vals_trajectory[0];
    B_trajectory[1] = (double) b_mat_vals_trajectory[1];
    B_trajectory[2] = (double) b_mat_vals_trajectory[2];
    B_trajectory[3] = (double) b_mat_vals_trajectory[3];

    xf::solver::gelinearsolver<TYPE_SOLVER, 4, 1>(4, A_trajectory, 1, B_trajectory, 4, 1, info);

    double transformationMatrix[3][3] = {{0.625, -0.5650116678948661, 118},
                                         {2.368475785867001e-16, -0.3130327315155981, 220.9999999999999},
                                         {6.143906059317771e-19, -0.001776283468435274, 1}};
    int64_t third_order_points_x[NUM_POINTS];
    int64_t third_order_points_y[NUM_POINTS];
    third_order_points: for(int i = 0; i < NUM_POINTS; ++i){
        int64_t x = i*INCREMENT;
        third_order_points_x[i] = x;
        third_order_points_y[i] = B_trajectory[3]*x*x*x+B_trajectory[2]*x*x + B_trajectory[1]*x + B_trajectory[0];
    }
    int64_t corrected_points_x[NUM_POINTS];
    int64_t corrected_points_y[NUM_POINTS];
    transform_points: for(int i = 0; i < NUM_POINTS; i++){
        double d = transformationMatrix[2][0]*third_order_points_y[i] + transformationMatrix[2][1] * third_order_points_x[i] + transformationMatrix[2][2];
        double tmp_x = (transformationMatrix[0][0] * third_order_points_y[i] + transformationMatrix[0][1] * third_order_points_x[i] + transformationMatrix[0][2]) / d;
        double tmp_y = (transformationMatrix[1][0] * third_order_points_y[i] + transformationMatrix[1][1] * third_order_points_x[i] + transformationMatrix[1][2]) / d;
        corrected_points_x[i] = (int64_t) tmp_y;
        corrected_points_y[i] = (int64_t) tmp_x; 

    }
    float screen_coordinates[NUM_POINTS][3]; 
    float rotationMatrix[3][3] = {{-0.2673452550386417, 0.04594220078070565, 0.962504975984929},
                                    {0.01289720370866832, -0.9986026996828118, 0.05124753967457311},
                                    {0.9635144922339365, 0.02611440931028619, 0.2663791675066841}};
    float cameraMatrix[3][3] = {{0.002345255454288494, 0, -0.6144776305934803},
                                {0, 0.002333306533752256, -0.7051919484003447},
                                {0, 0, 1}};
    float translationVector[3] = {0.1649449070508159, -0.009324807078981687, -0.09606275043777565};
    float invR_x_tvec[3] = {0};
    float wcPoint[NUM_POINTS][3];
    float tmp1[3][3];

    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
            tmp1[i][j] = 0;
        }
    }

    for(int i = 0; i < NUM_POINTS; i++){
        screen_coordinates[i][0] = corrected_points_x[i];
        screen_coordinates[i][1] = corrected_points_y[i];
        screen_coordinates[i][2] = 1.0;
        wcPoint[i][0] = 0;
        wcPoint[i][1] = 0;
        wcPoint[i][2] = 0;
    }
    // Calculate tmp1
    for(int i = 0; i < ROWS; ++i){
        for(int j = 0; j < COLS; ++j){
            for(int k = 0; k < COLS; ++k)
            {
                tmp1[i][j] += rotationMatrix[i][k] * cameraMatrix[k][j];
            }
        }
    }
    // Calculate invR_x_tvec
    for(int i = 0; i < ROWS; ++i){
        for(int k = 0; k < COLS; ++k)
        {
            invR_x_tvec[i] += rotationMatrix[i][k] * translationVector[k];
        }
    }

    float tmp2 = invR_x_tvec[2];
    // Main Loop for the point cloud transform
    for(int j=0; j < NUM_POINTS; ++j){
        float invR_x_invM_x_uv1[3] = {0};
        float tmp3[3] = {0};
        float tmp4[3] = {0};
        for(int i = 0; i < ROWS; ++i){
            for(int k = 0; k < COLS; ++k)
            {
                invR_x_invM_x_uv1[i] += tmp1[i][k] * screen_coordinates[j][k];
            }
        }
        float s = tmp2 / invR_x_invM_x_uv1[2];
        for(int i = 0; i < ROWS; ++i){
            for(int k = 0; k < COLS; ++k)
            {
                tmp3[i] += cameraMatrix[i][k] * screen_coordinates[j][k];
            }
        }
        for(int i = 0; i < ROWS; ++i){
            tmp4[i] = tmp3[i] * s - translationVector[i];
        }
        for(int i = 0; i < ROWS; ++i){
            for(int k = 0; k < COLS; ++k)
            {
                wcPoint[j][i] += rotationMatrix[i][k] * tmp4[k];
            }
        }

    }
    
    uint64_t line_buffer_out[NUM_POINTS] = {0};
    uint64_t valid[1];
    valid[0] = 1;
    if ((n_white < 1000) && (n_yellow < 1000)){
        valid[0] = 0;
    }
    MEM_WRITE(valid, output_message, 8);
    write_loop: for (int i = 0; i < NUM_POINTS; i++)
    {
        float_raw lsb;
        float_raw msb;
        lsb.f = wcPoint[i][0];
        msb.f = wcPoint[i][1];
        line_buffer_out[i] = (((uint64_t) msb.i) << 32) | ((uint64_t) lsb.i);
    }
    MEM_WRITE(line_buffer_out, output_message, NUM_POINTS*8);
}



THREAD_ENTRY() {
    uint64_t output_buffer_addr;

    THREAD_INIT();

    while(1)
    {
        uint64_t mono_message;
        ROS_SUBSCRIBE_TAKE_LOANED(rpolyfit_bird_image_sub, mono_message);
        ROS_BORROW(rpolyfit_poly_image_pub, output_buffer_addr);

        proc(memif_hwt2mem, memif_mem2hwt, mono_message, output_buffer_addr);

        ROS_SUBSCRIBE_RETURN_LOANED(rpolyfit_bird_image_sub, mono_message);
        ROS_PUBLISH_LOANED(rpolyfit_poly_image_pub, output_buffer_addr);
    }

    return;
}
