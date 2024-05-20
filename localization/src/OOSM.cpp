#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include "localization/buffer.h"

class SensorDataPublisher
{
public:
    SensorDataPublisher() : nh("~")
    {
        imu_sub = nh.subscribe("/imu", 1000, &SensorDataPublisher::imuCallback, this);
        mag_sub = nh.subscribe("/magnetometer", 1000, &SensorDataPublisher::magCallback, this);

        pub = nh.advertise<localization::buffer>("/sensors", 1000);
    }

    void publishSensorData()
    {
        buffered_measurements.header.stamp = ros::Time::now();
        pub.publish(buffered_measurements);
        std::cout << "Published sensor data: " << std::endl << buffered_measurements << std::endl;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    ros::Subscriber encoder_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber mag_sub;
    ros::Publisher pub;

    localization::buffer buffered_measurements;
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        buffered_measurements.measurements[0] = msg->angular_velocity.x;
        buffered_measurements.measurements[1] = msg->angular_velocity.y;
        buffered_measurements.measurements[2] = msg->angular_velocity.z;

        buffered_measurements.measurements[3] = msg->linear_acceleration.x;
        buffered_measurements.measurements[4] = msg->linear_acceleration.y;
        buffered_measurements.measurements[5] = msg->linear_acceleration.z;
    }
    
    void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        if (msg->velocity.size() != 6) return;

        for (int i = 0; i < 6; ++i) {
            buffered_measurements.wheel_odometry[i] = msg->velocity[i];
        }
    }

    void magCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        buffered_measurements.measurements[6] = msg->vector.x * 1e-6;
        buffered_measurements.measurements[7] = msg->vector.y * 1e-6;
        buffered_measurements.measurements[8] = msg->vector.z * 1e-6;
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        buffered_measurements.measurements[9] = msg->latitude;
        buffered_measurements.measurements[10] = msg->longitude;
    }
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "OOSM");
    SensorDataPublisher data_publisher;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        data_publisher.publishSensorData();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
