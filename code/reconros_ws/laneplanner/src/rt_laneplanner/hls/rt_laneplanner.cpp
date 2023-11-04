#include "reconos_thread.h"
#include "reconos_calls.h"

#include "hls_stream.h"
#include <ap_int.h>

#include <common/xf_common.hpp>
#include <common/xf_utility.hpp>

#include <imgproc/xf_rgb2hsv.hpp>
#include <imgproc/xf_inrange.hpp>
#include <imgproc/xf_colorthresholding.hpp>
#include <imgproc/xf_warp_transform.hpp>
#include <iceoryx_msg/msg/image640rgb.h>
#include <iceoryx_msg/msg/image640mono.h>

using namespace std;

#define HEIGHT 480
#define WIDTH 640
#define INTYPE uint64_t

#define TRANSFORMATION_TYPE 1
#define INTERPOLATION_TYPE 1
#define STORE_LINES 480
#define START_ROW 0
#define MAXCOLORS 1
#define USE_URAM true

#if NO
#define NPC1 XF_NPPC1
#endif
#if RO
#define NPC1 XF_NPPC8
#endif


void proc(hls::stream<uint64_t> & memif_hwt2mem, hls::stream<uint64_t> & memif_mem2hwt, uint64_t rgb_message, ap_uint<64> ram_out[640*480/8])
{
    ap_uint<64> ram_in[640*480*3/8];
    

    #pragma HLS stream variable=ram_in depth=32

    float transformationMatrix[] = {-0.4064797422227889, -1.816603481169413, 449.4339789207294,
                                    -2.282868227341687e-16, -3.194554113106124, 705.9964589964535,
                                    -1.128170170017353e-18, -0.005674433660132317, 1};
    float inverseTransformationMatrix[] = {0.625, -0.5650116678948661, 118,
                                           2.368475785867001e-16, -0.3130327315155981, 220.9999999999999,
                                           6.143906059317771e-19, -0.001776283468435274, 1};
   
    xf::cv::Mat<XF_8UC3, HEIGHT, WIDTH, XF_NPPC1> in_mat_white(HEIGHT,WIDTH);
    xf::cv::Mat<XF_8UC3, HEIGHT, WIDTH, XF_NPPC1> in_mat_yellow(HEIGHT,WIDTH);
    xf::cv::Mat<XF_8UC3, HEIGHT, WIDTH, XF_NPPC1> hsv_mat_white(HEIGHT,WIDTH);
    xf::cv::Mat<XF_8UC3, HEIGHT, WIDTH, XF_NPPC1> hsv_mat_yellow(HEIGHT,WIDTH);
    xf::cv::Mat<XF_8UC1, HEIGHT, WIDTH, XF_NPPC1> color_thresh_mat_white(HEIGHT,WIDTH);
    xf::cv::Mat<XF_8UC1, HEIGHT, WIDTH, XF_NPPC1> color_thresh_mat_yellow(HEIGHT,WIDTH);
    xf::cv::Mat<XF_8UC1, HEIGHT, WIDTH, XF_NPPC1> color_thresh_mat_combined(HEIGHT,WIDTH);
    xf::cv::Mat<XF_8UC1, HEIGHT, WIDTH, XF_NPPC1> out_mat(HEIGHT,WIDTH);

    #pragma HLS DATAFLOW

    MEM_READ(rgb_message, ram_in, WIDTH*HEIGHT*3);
    int ramptr = 0;
    loop_read: for(int i = 0; i < WIDTH * HEIGHT / 8; i+=1)
    {
        ap_uint<24> pix;
        uint64_t tmp_ram = ram_in[ramptr++];

        pix.range(23,16) =  (tmp_ram >> 0) & 0xff;
        pix.range(15,8) = (tmp_ram >> 8) & 0xff;
        pix.range(7,0 )= (tmp_ram >> 16) & 0xff;
        in_mat_white.write(i*8 + 0, pix);
        in_mat_yellow.write(i*8 + 0, pix);

        pix.range(23,16) =  (tmp_ram >> 24 )& 0xff;
        pix.range(15,8) = (tmp_ram >> 32) & 0xff;
        pix.range(7,0 )= (tmp_ram >> 40) & 0xff;
        in_mat_white.write(i*8 + 1, pix);
        in_mat_yellow.write(i*8 + 0, pix);

        pix.range(23,16) =  (tmp_ram >> 48 )& 0xff;
        pix.range(15,8) = (tmp_ram >> 56) & 0xff;
        tmp_ram = ram_in[ramptr++];
        pix.range(7,0 )= (tmp_ram >> 0) & 0xff;    
        in_mat_white.write(i*8 + 2, pix);
        in_mat_yellow.write(i*8 + 0, pix);

        pix.range(23,16) =  (tmp_ram >> 8) & 0xff;
        pix.range(15,8) = (tmp_ram >> 16) & 0xff;
        pix.range(7,0 )= (tmp_ram >> 24) & 0xff;
        in_mat_white.write(i*8 + 3, pix);
        in_mat_yellow.write(i*8 + 0, pix);

        pix.range(23,16) =  (tmp_ram >> 32) & 0xff;
        pix.range(15,8) = (tmp_ram >> 40) & 0xff;
        pix.range(7,0 )= (tmp_ram >> 48) & 0xff;
        in_mat_white.write(i*8 + 4, pix);
        in_mat_yellow.write(i*8 + 0, pix);

        pix.range(23,16) =  (tmp_ram >> 56) & 0xff;
        tmp_ram = ram_in[ramptr++];
        pix.range(15,8) = (tmp_ram >> 0) & 0xff;
        pix.range(7,0 )= (tmp_ram >> 8) & 0xff;
        in_mat_white.write(i*8 + 5, pix);
        in_mat_yellow.write(i*8 + 0, pix);

        pix.range(23,16) =  (tmp_ram >> 16) & 0xff;
        pix.range(15,8) = (tmp_ram >> 24) & 0xff;
        pix.range(7,0 )= (tmp_ram >> 32) & 0xff;
        in_mat_white.write(i*8 + 6, pix);
        in_mat_yellow.write(i*8 + 0, pix);

        pix.range(23,16) =  (tmp_ram >> 40) & 0xff;
        pix.range(15,8) = (tmp_ram >> 48) & 0xff;
        pix.range(7,0 )= (tmp_ram >> 56) & 0xff;
        in_mat_white.write(i*8 + 7, pix);
        in_mat_yellow.write(i*8 + 0, pix);
    }
    
    xf::cv::rgb2hsv<XF_8UC3, HEIGHT, WIDTH, XF_NPPC1>(in_mat_white, hsv_mat_white);
    xf::cv::rgb2hsv<XF_8UC3, HEIGHT, WIDTH, XF_NPPC1>(in_mat_yellow, hsv_mat_yellow);

    unsigned char low_thresh_white[] = {70, 0, 195};
    unsigned char high_thresh_white[] = {90, 87, 255};
    unsigned char low_thresh_yellow[] = {30, 95, 50};
    unsigned char high_thresh_yellow[] = {34, 255, 255};

    xf::cv::inRange<XF_8UC3, XF_8UC1, HEIGHT, WIDTH, XF_NPPC1>(hsv_mat_white, low_thresh_white, high_thresh_white, color_thresh_mat_white);
    xf::cv::inRange<XF_8UC3, XF_8UC1, HEIGHT, WIDTH, XF_NPPC1>(hsv_mat_yellow, low_thresh_yellow, high_thresh_yellow, color_thresh_mat_yellow);
    ap_uint<8> white_pixel;
    ap_uint<8> yellow_pixel;
    ap_uint<8> combined;
    for (int i = 0; i < WIDTH; i+=1){
        for(int j = 0; j < HEIGHT; j+=1){
            white_pixel = color_thresh_mat_white.read(i*WIDTH+j);
            yellow_pixel = color_thresh_mat_yellow.read(i*WIDTH+j);
            combined = white_pixel + (yellow_pixel >> 1);
            color_thresh_mat_combined.write(i*WIDTH+j, combined);
        } 
    }
    xf::cv::warpTransform<STORE_LINES, START_ROW, TRANSFORMATION_TYPE, INTERPOLATION_TYPE, XF_8UC1, HEIGHT, WIDTH, XF_NPPC1, USE_URAM>(color_thresh_mat_combined, out_mat, inverseTransformationMatrix);

    ramptr = 0;
    loop_write: for(int i = 0; i < WIDTH * HEIGHT / 8; i+=1)
    {
        ap_uint<64> tmp_ram;
        ap_uint<8> pix;

        pix = out_mat.read (i*8 + 0);
        tmp_ram.range( 7, 0) = pix;

        pix = out_mat.read (i*8 + 1);
        tmp_ram.range(15, 8) = pix;
        
        pix = out_mat.read (i*8 + 2);
        tmp_ram.range(23, 16) = pix;

        pix = out_mat.read (i*8 + 3);
        tmp_ram.range(31, 24) = pix;

        pix = out_mat.read (i*8 + 4);
        tmp_ram.range(39, 32) = pix;

        pix = out_mat.read (i*8 + 5);
        tmp_ram.range(47, 40) = pix;
        
        pix = out_mat.read (i*8 + 6);
        tmp_ram.range(55, 48) = pix;

        pix = out_mat.read (i*8 + 7);
        tmp_ram.range(63, 56) = pix;
        
        ram_out[ramptr++] = tmp_ram;
    }
}

THREAD_ENTRY() {
    ap_uint<64> ram_out[640*480/8];
    uint64_t output_buffer_addr;

    THREAD_INIT();

    while(1)
    {
        uint64_t rgb_message;
        ROS_SUBSCRIBE_TAKE_LOANED(rlaneplanner_camera_image_sub, rgb_message);

        uint64_t rgb_payloar_address = OFFSETOF(iceoryx_msg__msg__Image640rgb, data) + rgb_message;

        proc(memif_hwt2mem, memif_mem2hwt, rgb_payloar_address, ram_out);

        ROS_SUBSCRIBE_RETURN_LOANED(rlaneplanner_camera_image_sub, rgb_message);

        ROS_BORROW(rlaneplanner_output_image_pub, output_buffer_addr);
        MEM_WRITE(ram_out, output_buffer_addr, WIDTH*HEIGHT);
        ROS_PUBLISH_LOANED(rlaneplanner_output_image_pub, output_buffer_addr);
    }

    return;
}
