#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import math


def quaternion2euler(ox, oy, oz, ow):
    # Convert quaternion to Euler angles (yaw_z)
    t3 = +2.0 * (ow * oz + ox * oy)
    t4 = +1.0 - 2.0 * (oy * oy + oz * oz)
    yaw_z = math.atan2(t3, t4)
    return yaw_z


class Controller:
    def __init__(self):
        # Initialize the ROS node and create a subscriber and publisher
        rospy.init_node("pid", anonymous=True)
        self.sub = rospy.Subscriber("atom/odom", Odometry, self.get_pos)
        self.pub = rospy.Publisher("atom/cmd_vel", Twist, queue_size=10)

        # Sample set of key points to follow, you may change it for further testing
        # Or if you don't find it appropriate.
        self.path = [(0, 0), (4, 0), (4, 4), (0, 4), (2, 2)]

        # Initialize variables
        self.vel_msg = Twist()
        self.rate = rospy.Rate(30)
        self.x = 0
        self.y = 0
        self.org_x = 0
        self.org_y = 0
        self.theta = 0
        self.max_omega = 0.2
        self.min_omega = -0.2
        self.max_speed = 0.2
        self.min_speed = -0.2
        self.rot_flag = True

        # PID controller gains for linear and angular control
        self.kp_lin = 0.002
        self.ki_lin = 0.001
        self.kd_lin = 0.001
        self.kp_ang = 0.01
        self.ki_ang = 0.001
        self.kd_ang = 0.01

    def tester(self):
        counter = 0

        """
        Note: This is just a sample code to help you understand how the code works
        for publishing messages to the bot
        You are expected to change it up sufficiently to align it with the problem statement
        """

        while not rospy.is_shutdown():
            if counter % 20 == 0:
                # Print current velocity, angular velocity and coordinates every 2 seconds

                # Obviously your implementation shouldn't make changes every 2 seconds only. Updating values in every iteration of a while loop should
                # do the trick

                print("Current velocity: ", self.vel_msg.linear.x)
                print("Current angular velocity: ", self.vel_msg.angular.z)
                print("Current coordinates: ", self.x, self.y)

                # Stop the robot and prompt the user for new velocities every 2 seconds
                self.vel_msg.linear.x = (
                    0  # Set the x velocity for the message using this variable
                )
                self.vel_msg.angular.z = (
                    0  # Set the z angular velocity for the message using this variable
                )
                self.pub.publish(self.vel_msg)

                # Instead of taking user input, input the coordinates to determine velocity
                # HINT: This is what PID controllers are for, to calculate the velocity you need to reach your goal.
                self.vel_msg.linear.x = float(input("Enter linear velocity: "))
                self.vel_msg.angular.z = float(input("Enter angular velocity: "))

            # Publish the velocities to the cmd_vel topic and sleep for the specified rate
            print("cmd_vel vals", self.vel_msg.linear.x, self.vel_msg.angular.z)
            self.pub.publish(self.vel_msg)
            self.rate.sleep()
            counter += 1

    """
    Write your implementation here
    """

    def mover(self):
        self.org_x = float(input("Enter x coordinate: "))
        self.org_y = float(input("Enter y coordinate: "))
        self.vel_msg.linear.x = (
            0  # Set the x velocity for the message using this variable
        )
        self.vel_msg.angular.z = (
            0  # Set the z angular velocity for the message using this variable
        )
        print("Target coordinates: ", self.org_x, self.org_y)
        while not rospy.is_shutdown():
            inc_x = self.org_x - self.x
            inc_y = self.org_y - self.y
            angle_to_goal = math.atan2(inc_y, inc_x)
            angle_to_goal = math.degrees(angle_to_goal)
            if abs(angle_to_goal - self.theta) > 0.3:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = 0.2
            elif math.dist([self.org_x, self.org_y], [self.x, self.y]) > 0.3:
                self.vel_msg.linear.x = 0.3
                self.vel_msg.angular.z = 0.0
            else:
                pass

            # Publish the velocities to the cmd_vel topic and sleep for the specified rate
            print("cmd_vel vals", self.vel_msg.linear.x, self.vel_msg.angular.z)
            self.pub.publish(self.vel_msg)
            self.rate.sleep()

            self.vel_msg.linear.x = (
                0  # Set the x velocity for the message using this variable
            )
            self.vel_msg.angular.z = (
                0  # Set the z angular velocity for the message using this variable
            )
            self.pub.publish(self.vel_msg)
            self.rate.sleep()

    # Use this to verify if you have reached the correct position.
    # Note that some degree of inaccuracy is allowed but try to minimize it as much as possibe.
    # Do whatever you feel is appropriate but then it must work for subtask2 as well

    def get_pos(self, data):
        # Extract the orientation and position from the Odometry message and convert the orientation to Euler angles (yaw_z)
        ox, oy, oz, ow = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,
        )
        x, y = data.pose.pose.position.x, data.pose.pose.position.y
        yaw_z = quaternion2euler(ox, oy, oz, ow)
        yaw_z = math.degrees(yaw_z)

        # Update the current position and orientation of the robot (self.x, self.y and self.theta)
        self.x, self.y, self.theta = x, y, yaw_z


if __name__ == "__main__":
    try:
        controller = Controller()
        # controller.tester()  # Comment this line out once you are done testing our code and writing your own

        # And uncomment this to write your implementation

        controller.mover()
    except rospy.ROSInterruptException:
        pass
