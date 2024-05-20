#include "ROAR_UKF.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include "localization/imu.h"
#include "localization/encoder.h"

int main(int argc, char **argv) 
{

    ros::init(argc, argv, "localization");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<localization::imu>("sensors/imu", 1000);
    ros::Publisher pub2 = nh.advertise<localization::encoder>("sensors/encoders", 1000);
    ros::Publisher pub3 = nh.advertise<sensor_msgs::NavSatFix>("sensors/gps", 1000);

    while (ros::ok())
    {
        localization::imu imu_msg;
        pub.publish(imu_msg);

        localization::encoder encoder_msg;
        pub2.publish(encoder_msg);

        sensor_msgs::NavSatFix gps_msg;
        pub3.publish(gps_msg);

    }

    return 0;
}