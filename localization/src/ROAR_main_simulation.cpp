#include "ROAR_UKF.h"
#include <ros/ros.h>
#include <chrono>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float64MultiArray.h>
#include "localization/buffer.h"
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>

using namespace std;

Eigen::VectorXd z_measurement(11);
Eigen::VectorXd encoder_measurement(6);
Eigen::VectorXd ground_truth(3);

const int n_state_dim = 9;  // x_state dimension
const float alpha = 0.3;
const float beta_ = 2.0;
const float kappa = 0.1;
ros::Time encoder_prev_time_stamp;
ros::Time imu_prev_time_stamp;
ros::Time gps_prev_time_stamp;
double dt = 0.01;
bool new_measurement_received = false;
bool intial_measurment = true;
double lat0 = 0.0;
double lon0 = 0.0;

// Initialize Sigma Points and UKF
MerwedSigmaPoints sigma_points(n_state_dim, alpha, beta_, kappa);
UKF ukf(sigma_points);

ros::Subscriber imu_sub;
ros::Subscriber encoder_sub;
ros::Subscriber gps_sub;
ros::Subscriber ground_truth_sub;

ros::Publisher state_publisher;

void publishTransform(const Eigen::VectorXd& states) {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";  // Fixed frame
    transformStamped.child_frame_id = "imu_frame";  // Frame of your IMU

    // Fill in transform
    transformStamped.transform.translation.x = 0.0;  // Adjust as needed
    transformStamped.transform.translation.y = 0.0;  // Adjust as needed
    transformStamped.transform.translation.z = 0.0;  // Adjust as needed
    Eigen::Quaterniond quat(states[0], states[1], states[2], states[3]);
    quat.normalize(); // Ensure unit magnitude
    transformStamped.transform.rotation.w = quat.w();
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();

    // Publish transform
    static_broadcaster.sendTransform(transformStamped);
}
void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std_msgs::Float64MultiArray state_msg;

    if (msg->velocity.size() != 6) return;

    if (encoder_prev_time_stamp.isZero()) 
    {
        encoder_prev_time_stamp = msg->header.stamp;
        return;
    }
    ros::Time encoder_current_time_stamp = msg->header.stamp;
    dt = (encoder_current_time_stamp - encoder_prev_time_stamp).toSec();

    for (int i = 0; i < 6; ++i) {
        encoder_measurement[i] = msg->velocity[i];
    }
    // cout << "encoder process" << endl;
    // cout << "ground_truth: " << ground_truth.transpose() << endl;
    ukf.encoder_callback(encoder_measurement, dt);
    encoder_prev_time_stamp = encoder_current_time_stamp;
    // cout << "encoder dt: " << dt << endl;
    // cout << "encoder_x_posterior: " << ukf.x_post.transpose() << endl;

    state_msg.data = {ukf.x_post[0], ukf.x_post[1], ukf.x_post[2], ukf.x_post[3], ukf.x_post[4], ukf.x_post[5], ukf.x_post[6], ukf.x_post[7], ukf.x_post[8]};
    state_publisher.publish(state_msg);
}
void gpsCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    if (intial_measurment == true)
    {
        lat0 = msg->vector.x;
        lon0 = msg->vector.y;
        intial_measurment = false;
    }
    if (encoder_prev_time_stamp.isZero()) 
    {
        encoder_prev_time_stamp = msg->header.stamp;
        return;
    }
    ros::Time gps_current_time_stamp = msg->header.stamp;
    dt = (gps_current_time_stamp - gps_prev_time_stamp).toSec();

    z_measurement[9] = msg->vector.x;
    z_measurement[10] = msg->vector.y;
    cout << "gps measurement: " << z_measurement.transpose() << endl;
    ukf.gps_callback(z_measurement, dt, lat0, lon0);
    gps_prev_time_stamp = gps_current_time_stamp;
    cout << "gps callback:" << ukf.x_post.transpose() << endl;
}
void imuCallback(const localization::buffer::ConstPtr& msg)
{
    std_msgs::Float64MultiArray state_msg;

    if (imu_prev_time_stamp.isZero()) 
    {
        imu_prev_time_stamp = msg->header.stamp;
        return;
    }
    ros::Time imu_current_time_stamp = msg->header.stamp;
    dt = (imu_current_time_stamp - imu_prev_time_stamp).toSec();

    for (int i = 0; i < 9; ++i) 
    {
        z_measurement[i] = msg->measurements[i];
    }

    ukf.imu_callback(z_measurement, dt);
    imu_prev_time_stamp = imu_current_time_stamp;

    // Publish the quaternion transform
    publishTransform(ukf.x_post);

}
void ground_truth_callback(const gazebo_msgs::ModelStates::ConstPtr msg)
{

    // Saving pose
// ground_truth << msg->pose[1].orientation.w,
//                 msg->pose[1].orientation.x,
//                 msg->pose[1].orientation.y,
//                 msg->pose[1].orientation.z,
//                 msg->pose[1].position.x,
//                 msg->pose[1].position.y;

    float roll = atan2(2*(msg->pose[1].orientation.w*msg->pose[1].orientation.x + msg->pose[1].orientation.y
                            *msg->pose[1].orientation.z), 1 - 2*(msg->pose[1].orientation.x*msg->pose[1].orientation.x 
                                + msg->pose[1].orientation.y*msg->pose[1].orientation.y)) * 180/PI;
	float pitch = asin(2*(msg->pose[1].orientation.w*msg->pose[1].orientation.y - msg->pose[1].orientation.z*msg->pose[1].orientation.x))*180/PI;
	float yaw = atan2(2*(msg->pose[1].orientation.w*msg->pose[1].orientation.z + msg->pose[1].orientation.x
                        *msg->pose[1].orientation.y), 1 - 2*(msg->pose[1].orientation.y*msg->pose[1].orientation.y 
                            + msg->pose[1].orientation.z*msg->pose[1].orientation.z))*180/PI;

    ground_truth << roll, pitch, yaw;    
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ukf_localization");
    ros::NodeHandle nh;
    
    imu_sub = nh.subscribe("/imu_readings", 1000, imuCallback);
    // encoder_sub = nh.subscribe("/joint_states", 1000, encoderCallback);
    gps_sub = nh.subscribe("/GPS", 1000, gpsCallback);
    ground_truth_sub = nh.subscribe("gazebo/model_states", 1000, ground_truth_callback);

    state_publisher = nh.advertise<std_msgs::Float64MultiArray>("/filtered_state", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
    }

	return 0;
}
