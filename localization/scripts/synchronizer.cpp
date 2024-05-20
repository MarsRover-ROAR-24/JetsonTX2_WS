#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/console.h>

using namespace message_filters;

class Node {
public:
    Node() : sub_imu(node, "imu", 10), sub_joint_states(node, "joint_states", 10), sync(sub_imu, sub_joint_states, 10) {
        sync.registerCallback(&Node::syncCallback, this);
    }

private:
    ros::NodeHandle node;
    message_filters::Subscriber<sensor_msgs::Imu> sub_imu;
    message_filters::Subscriber<sensor_msgs::JointState> sub_joint_states;
    TimeSynchronizer<sensor_msgs::Imu, sensor_msgs::JointState> sync;

    void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
        // Process IMU data here
        ROS_INFO_STREAM("hi, Received IMU message at: " << msg->header.stamp);
    }

    void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg) {
        // Process joint states data here
        ROS_INFO_STREAM("hi, Received joint states message at: " << msg->header.stamp);
    }

    void syncCallback(const sensor_msgs::ImuConstPtr& imu_msg, const sensor_msgs::JointStateConstPtr& joint_states_msg) {
        // Check if time difference is within tolerance
        double time_diff = std::abs(imu_msg->header.stamp.toSec() - joint_states_msg->header.stamp.toSec());
        ROS_INFO_STREAM("HI");
        if (time_diff > 0.025) {
            ROS_INFO_STREAM("Time difference between IMU and joint states exceeds 0.03 seconds: " << time_diff);
            ROS_INFO_STREAM("Received IMU message sync at: " << imu_msg->header.stamp.nsec);
            ROS_INFO_STREAM("Received encoder message sync at: " << joint_states_msg->header.stamp.nsec);
            return; // Skip processing if time difference is too large
        }
        ROS_INFO_STREAM("Synchronized message at: " << time_diff);
        ROS_INFO_STREAM("Received IMU message synv at: " << imu_msg->header.stamp.nsec);
        ROS_INFO_STREAM("Received encoder message synv at: " << joint_states_msg->header.stamp.nsec);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "synchronizer");
    Node node;
    
    ros::spin();
    return 0;
}
