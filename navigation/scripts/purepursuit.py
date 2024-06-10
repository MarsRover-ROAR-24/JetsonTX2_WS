#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Float64, Float64MultiArray, Int8MultiArray
from tf.transformations import euler_from_quaternion
import numpy as np
# from turtlebot3_msgs.msg import wp_list

class Control:

    def __init__(self):

        rospy.init_node('controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/nav_action/supervised', Int8MultiArray, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/filtered_state', Float64MultiArray, self.update_pose)
        # self.path_subscriber = rospy.Subscriber('tuple_list_topic', wp_list, self.tuple_list_callback) 

        self.pose = Float64MultiArray()
        self.throttle_output=Float64()
        self.published_velocity = Int8MultiArray()

        self.rate = rospy.Rate(10)
        self.kp = 0.5
        self.ki = 0.5
        self.kd = 0.0
        self.dist_ld = 1

        self.dt = 0.1
        self.currentx = 0.0
        self.currenty = 0.0
        self.integral = 0.0
        self.max_velocity = 1.57

        self.robot_theta = 0.0
        self.width = 0.8

        self.waypoints = [(1,0)]

    # def tuple_list_callback(self, msg):
    #     self.waypoints = [(msg.a[i], msg.b[i]) for i in range(msg.length)]
    #     self.x_goal_point = msg.a[0]
    #     self.y_goal_point = msg.b[0]
    #     self.waypoints.reverse()

    def update_pose(self, data:Float64MultiArray):
        self.pose = data
        self.currentx = self.pose.data[7]
        self.currenty = self.pose.data[8]
        orientation_list = [self.pose.data[1], self.pose.data[2], self.pose.data[3], self.pose.data[0]]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_theta = yaw

    def map_velocity(self,velocity):
        # Clamping the value to be within the range -1.57 to 1.57
        velocity = min(max(velocity, -1.57), 1.57)
        
        if velocity < 0:
            # Mapping negative values from -1.57 to 0 to the range 0 to 55
            return int(((velocity + 1.57) / 1.57) * 55)
        else:
            # Mapping positive values from 0 to 1.57 to the range 75 to 127
            return int((velocity / 1.57) * 52 + 75)

    def pidController(self):
        e= 0.0
        lookahead_point = self.find_lookahead_point((self.currentx, self.currenty))
        goal_point = self.waypoints[-1]
        distance_to_goal = np.linalg.norm(np.array((self.currentx, self.currenty)) - np.array(goal_point))
        if distance_to_goal > 0.01:
            e = math.hypot(lookahead_point[0] - self.currentx, lookahead_point[1] - self.currenty)
            e_past = 0
            if e > 0.1:
                self.integral += e * self.dt
                derivative = (e - e_past) / self.dt
                action = self.kp * e + self.ki * self.integral + self.kd * derivative
                self.throttle_output = self.max_velocity * math.tanh(action)
                e_past = e
            else:
                self.throttle_output = 0.0     
        else:
            self.throttle_output = 0.0

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

            Vr_mapped = self.map_velocity(Vr)
            Vl_mapped = self.map_velocity(Vl)
            
            self.published_velocity.data = [Vl_mapped, Vr_mapped, Vl_mapped, Vr_mapped, Vl_mapped, Vr_mapped]
            self.velocity_publisher.publish(self.published_velocity)

if __name__ == '__main__':
    try:
        x = Control()
        while not rospy.is_shutdown():
            if len(x.waypoints) > 0:
                x.purePursuit()
    except rospy.ROSInterruptException:
        pass
