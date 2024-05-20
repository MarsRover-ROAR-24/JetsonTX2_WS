#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray

# subscribe to gazebo/model_states topic then publish to /rover/ground_truth as Float64MultiArray
def model_state_callback(msg):
    state_vector = Float64MultiArray()
    # Append the position and orientation to the state vector
    state_vector.data = [msg.pose[1].orientation.w,msg.pose[1].orientation.x, msg.pose[1].orientation.y,
                         msg.pose[1].orientation.z,msg.twist[1].angular.x,msg.twist[1].angular.y,
                         msg.twist[1].angular.z,msg.pose[1].position.x, msg.pose[1].position.y]
    # Publish the state vector
    ground_truth_pub.publish(state_vector)

# Initialize the ROS node
rospy.init_node("model_state_publisher")

# Subscribe to the /gazebo/model_states topic
rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_callback)

# main
if __name__ == "__main__":
    # Initialize the ground truth publisher
    ground_truth_pub = rospy.Publisher("/ground_truth", Float64MultiArray, queue_size=10)
    # Spin
    rospy.spin()