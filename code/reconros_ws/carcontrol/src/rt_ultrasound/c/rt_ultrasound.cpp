#include <chrono>
#include <thread>
using namespace std;

extern "C" {
#include "reconos_thread.h"
#include "reconos_calls.h"

#include "../application/axi_modelcar.h"
}

extern "C" THREAD_ENTRY(); // this is required because of the mixture of c and c++

THREAD_ENTRY() {
	
        t_axi_modelcar *axi_modelcar = axi_modelcar_init(0xA0000000);

        axi_modelcar->Trigger_Ctrl_Reg = 1;
        int echo = 0;
        float echo_time = 0;
        float range = 0;
        while(1)
        {
            echo = axi_modelcar->Echo;
            echo_time = (float) echo / 100000000;
            range = echo_time * 170;

            rultrasound_range_msg->range = range;
            ROS_PUBLISH(rultrasound_range_pub, rultrasound_range_msg);
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    	return;
}
