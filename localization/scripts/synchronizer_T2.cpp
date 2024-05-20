#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace sensor_msgs;
using namespace message_filters;

void syncCallback (const sensor_msgs::ImuConstPtr &imu_msg, const sensor_msgs::JointStateConstPtr &joint_states_msg)
   {
    // Check if time difference is within tolerance
    ROS_INFO_STREAM("inside the callback");
    double time_diff = std::abs(imu_msg->header.stamp.toSec() - joint_states_msg->header.stamp.toSec());
    ROS_INFO_STREAM("HI");
    if (time_diff > 0.03)
    {
        ROS_INFO_STREAM("Time difference between IMU and joint states exceeds 0.03 seconds: " << time_diff);
        ROS_INFO_STREAM("Received IMU message sync at: " << imu_msg->header.stamp.nsec);
        ROS_INFO_STREAM("Received encoder message sync at: " << joint_states_msg->header.stamp.nsec);
        return; // Skip processing if time difference is too large
    }
    ROS_INFO_STREAM("Synchronized message at: " << time_diff);
    ROS_INFO_STREAM("Received IMU message synv at: " << imu_msg->header.stamp.nsec);
    ROS_INFO_STREAM("Received encoder message synv at: " << joint_states_msg->header.stamp.nsec);
    }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Synchronizer_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Imu> sub_imu(nh, "/imu", 5);
    ROS_INFO_STREAM("i am here");
    message_filters::Subscriber<sensor_msgs::JointState> sub_joint_states(nh, "/joint_states", 10); 
    ROS_INFO_STREAM("Hello");
    typedef sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::JointState> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_imu, sub_joint_states);
    sync.registerCallback(boost::bind(&syncCallback, _1, _2));
    ROS_INFO_STREAM("Done!");
    
    ros::spin();
    ROS_INFO_STREAM("Doneeee!");

    return 0;
}
