#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

extern "C" {
#include "reconos_thread.h"
#include "reconos_calls.h"

#include "../application/axi_modelcar.h"
}
extern "C" THREAD_ENTRY(); // this is required because of the mixture of c and c++

THREAD_ENTRY() {
	
        t_axi_modelcar *axi_modelcar = axi_modelcar_init(0xA0000000);

        int time_count = 0;
        double time_interval = 0;
        float speed = 0;
        float wheel_radius = 0.357;
        rclcpp::Clock myclock(RCL_SYSTEM_TIME);
        builtin_interfaces__msg__Time customTime;
        while(1){
            time_count = axi_modelcar->Hall_Sensor_Interval;
            // Prevent division by zero
            if (time_count == 0){
                speed = 0;
            }
            // Convert the clock cycle count to a speed
            else{
                time_interval = (double) time_count / 100000000;
                speed = (wheel_radius / 8) / time_interval;
            }
            rhallsensor_speed_msg->twist.twist.linear.x = speed;
            rclcpp::Time currentTime= myclock.now();
            builtin_interfaces::msg::Time time(currentTime);
            customTime.sec = time.sec;
            customTime.nanosec = time.nanosec;
            rhallsensor_speed_msg->header.stamp = customTime;
            ROS_PUBLISH(rhallsensor_speed_pub, rhallsensor_speed_msg);
            // 10 Hz publish frequency
            usleep(100000);
        }
}
