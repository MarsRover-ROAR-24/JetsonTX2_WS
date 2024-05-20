#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

// Callback function to handle incoming IMU messages
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Print the IMU data
    ROS_INFO("IMU Data:");
    ROS_INFO("Linear Acceleration (x, y, z): %.2f, %.2f, %.2f",
             msg->linear_acceleration.x,
             msg->linear_acceleration.y,
             msg->linear_acceleration.z);
    ROS_INFO("Angular Velocity (x, y, z): %.2f, %.2f, %.2f",
             msg->angular_velocity.x,
             msg->angular_velocity.y,
             msg->angular_velocity.z);

    // Print the orientation quaternion
    ROS_INFO("Orientation Quaternion (x, y, z, w): %.2f, %.2f, %.2f, %.2f",
             msg->orientation.x,
             msg->orientation.y,
             msg->orientation.z,
             msg->orientation.w);
}

// Callback function to handle incoming Encoder messages
void EncoderCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Print the Encoder data
    ROS_INFO("Encoder Data:");
    ROS_INFO("Wheel Velocities Data: %.2f, %.2f, %.2f %.2f, %.2f, %.2f",
             msg->velocity[0],
             msg->velocity[1],
             msg->velocity[2],
             msg->velocity[3],
             msg->velocity[4],
             msg->velocity[5]);
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "sensor_subscriber");
    ros::NodeHandle nh;

    // Subscribe to the IMU topic
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 10, imuCallback);

    // Subscribe to the Encoder topic
    ros::Subscriber encoder_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 10, EncoderCallback);

    // Spin and process callbacks
    ros::spin();

    return 0;
}
