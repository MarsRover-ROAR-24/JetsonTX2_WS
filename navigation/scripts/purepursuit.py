#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import String, Float64, Float64MultiArray, Int8MultiArray
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, LinkStates
from tf.transformations import euler_from_quaternion
import time
import numpy as np
import matplotlib.pyplot as plt
from turtlebot3_msgs.msg import wp_list

class Control:

    def __init__(self):

        rospy.init_node('controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/nav_action/supervised', Int8MultiArray, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/filtered_state', Float64MultiArray, self.update_pose)
        self.path_subscriber = rospy.Subscriber('tuple_list_topic', wp_list, self.tuple_list_callback)  #===>

        self.pose = ModelStates()
        self.throttle_output=Float64()

        self.rate = rospy.Rate(10)
        self.kp = 0.5
        self.ki = 0.5
        self.kd = 0.0
        self.dist_ld = 0.3

        self.dt = 0.1
        self.currentx = 0.0
        self.currenty = 0.0
        self.integral = 0.0
        self.max_velocity = 1.57

        self.robot_theta = 0.0
        self.width = 0.8
        self.time_values = []
        self.error_values = []

        self.waypoints = []
        # self.x_goal_point = 0.0
        # self.y_goal_point = 0.0
        self.waypoints_plot = None

        # Setup matplotlib for plotting
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 6))
        # self.fig, (self.ax1) = plt.subplots(1, figsize=(6, 6))


        # Plot for waypoints
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_title('Waypoints')
        self.ax1.set_xlim(-10, 1)  # Set x-axis limits from -10 to 10
        self.ax1.set_ylim(-2.5, 2.5)  # Set y-axis limits from -10 to 10
        self.waypoints_x = []
        self.waypoints_y = []
        self.waypoints_plot, = self.ax1.plot([], [], 'b--', label='Waypoints')
        self.robot_position_plot, = self.ax1.plot([], [], 'r^', label='Robot Position', markersize=6)  # Thinner shape
        # Add a plot for the robot's path
        self.past_positions_x = []
        self.past_positions_y = []
        self.robot_path_plot, = self.ax1.plot([], [], 'r-', label='Robot Path')  # Robot path plot
        self.ax1.legend()
    
        # Plot for error vs. time
        self.ax2.set_xlabel('Time')
        self.ax2.set_ylabel('Error')
        self.line, = self.ax2.plot([], [], label='Error vs. Time')
        self.ax2.legend()

        plt.tight_layout()

    def tuple_list_callback(self, msg):
        self.waypoints = [(msg.a[i], msg.b[i]) for i in range(msg.length)]
        self.x_goal_point = msg.a[0]
        self.y_goal_point = msg.b[0]
        self.waypoints.reverse()
        self.update_waypoints_plot()

    def update_pose(self, data:Float64MultiArray):
        self.pose = data
        self.currentx = self.pose.data[7]
        self.currenty = self.pose.data[8]
        orientation_list = [self.pose.data[1], self.pose.data[2], self.pose.data[3], self.pose.data[0]]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_theta = yaw

    def pidController(self):
        e= 0.0
        lookahead_point = self.find_lookahead_point((self.currentx, self.currenty))
        goal_point = self.waypoints[-1]
        distance_to_goal = np.linalg.norm(np.array((self.currentx, self.currenty)) - np.array(goal_point))
        print("Goal point:",goal_point)
        if distance_to_goal > 0.01:
            e = math.hypot(lookahead_point[0] - self.currentx, lookahead_point[1] - self.currenty)
            print("Selected Lookahead Point:", lookahead_point)
            e_past = 0
            if e > 0.1:
                self.integral += e * self.dt
                derivative = (e - e_past) / self.dt
                action = self.kp * e + self.ki * self.integral + self.kd * derivative
                self.throttle_output = self.max_velocity * math.tanh(action)
                e_past = e
                print('Error = ', e)
            else:
                self.throttle_output = 0.0     
        else:
            self.throttle_output = 0.0

        # Append time and error values for plotting
        self.time_values.append(rospy.get_time())
        self.error_values.append(e)

        # Publish to other wheel controllers
        rospy.loginfo('Error = %f', e)

        # Plot error vs. time
        self.line.set_xdata(self.time_values)
        self.line.set_ydata(self.error_values)
        self.ax2.relim()
        self.ax2.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        return self.throttle_output
    
    def find_lookahead_point(self, robot_position):
        candidate_lookahead_points = []
        max_index = -1

        for i, waypoint in enumerate(self.waypoints):
                distance_to_robot = np.linalg.norm(np.array(waypoint) - np.array(robot_position))

                if distance_to_robot < self.dist_ld and i > max_index:
                        candidate_lookahead_points = [waypoint]
                        max_index = i

        if not candidate_lookahead_points:
                return None  # No valid lookahead point found

        # Calculate distances from candidate lookahead points to the goal
        # distances_to_goal = [np.linalg.norm(np.array(waypoint) - np.array((self.x_goal_point, self.y_goal_point))) for waypoint in candidate_lookahead_points]

        # Find the index of the candidate with the maximum distance to the goal
        max_distance_index = np.argmax(distance_to_robot)

        # Select the lookahead point with the maximum distance to the goal
        lookahead_point = candidate_lookahead_points[max_distance_index]

        return lookahead_point

    def purePursuit(self):
        lookahead_point = self.find_lookahead_point((self.currentx, self.currenty))

        if lookahead_point is not None:
            alpha = math.atan2((lookahead_point[1] - self.currenty), (lookahead_point[0] - self.currentx))
            L = math.hypot(lookahead_point[0] - self.currentx, lookahead_point[1] - self.currenty)
            theta = alpha - self.robot_theta
            dx = L * math.cos(theta)
            Vr = self.pidController() * (1 - self.width * dx / (L * L))
            Vl = self.pidController() * (1 + self.width * dx / (L * L))

            Vr = min(max(Vr, -1.57), 1.57)
            Vl = min(max(Vl, -1.57), 1.57)

            Vr_mapped = int(((Vr + 1.57) / (1.57 * 2)) * 127 + 0.5) 
            Vl_mapped = int(((Vl + 1.57) / (1.57 * 2)) * 127 + 0.5)
            print('Right: ', Vr, ' Mapped Right:', Vr_mapped, ' Left: ', Vl, ' Mapped Left:', Vl_mapped)

            published_velocity = Int8MultiArray()
            published_velocity.data = [Vl_mapped, Vr_mapped, Vl_mapped, Vr_mapped, Vl_mapped, Vr_mapped]
            self.velocity_publisher.publish(published_velocity)
            
            # Plot rover position
            self.plot_rover_position()
            # Give time for plot to update
            plt.pause(0.001)

    def plot_rover_position(self):
        # self.ax1.plot(self.currentx, self.currenty, 'ro')  # Plot current position in red
        # self.robot_position_plot.set_data(self.currentx, self.currenty)  # Update the robot position plot
    # Append the current position to the past positions
        self.past_positions_x.append(self.currentx)
        self.past_positions_y.append(self.currenty)
        
        # Update the robot path plot
        self.robot_path_plot.set_data(self.past_positions_x, self.past_positions_y)
        self.robot_position_plot.set_data(self.currentx, self.currenty)  # Update the current position plot
    def update_waypoints_plot(self):
        self.waypoints_x, self.waypoints_y = zip(*self.waypoints)
        self.waypoints_plot.set_data(self.waypoints_x, self.waypoints_y)

if __name__ == '__main__':
    try:
        x = Control()
        while not rospy.is_shutdown():
            if len(x.waypoints) > 0:
                x.purePursuit()
    except rospy.ROSInterruptException:
        pass
