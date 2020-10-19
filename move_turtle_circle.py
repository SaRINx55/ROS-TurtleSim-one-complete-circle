#!/usr/bin/env python

"""
Ros node to make to the turtle draw a circle
"""
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math


class TurtleController(object):
    """
    Controls the turtle to draw a circle
    """
    def __init__(self):
        # turtle current position
        self.turtle_x = 0
        self.turtle_y = 0
        self.turtle_theta = 0

        # initial turtle coordinates
        self.initial_turtle_x = None
        self.initial_turtle_y = None
        self.initial_turtle_theta = None

        self.initial_coords_updated = False

        # turtle angular speed (rad/s)
        self.turtle_speed = 0.8
        # radius of the circle to be drawn
        self.circle_radius = 1

        rospy.init_node("node_turtle_revolve", anonymous=False)

        # Subscriber for "turtle1/pose" to get current location and speed.
        rospy.Subscriber("/turtle1/pose", Pose, self.update_pose)

        # Publisher for "turtle1/cmd_vel" to publish angular and linear vel.
        self.velocity_pub = rospy.Publisher("/turtle1/cmd_vel", Twist,
                                            queue_size=10)

    def update_pose(self, msg):
        """
        Callback funtion for topic "/turtle1/pose".
        Updates x, y and theta.
        """
        self.turtle_x = msg.x
        self.turtle_y = msg.y
        self.turtle_theta = msg.theta

        if not self.initial_coords_updated:
            self.initial_turtle_x = msg.x
            self.initial_turtle_y = msg.y
            self.initial_turtle_theta = msg.theta
            self.initial_coords_updated = True

    def run(self):
        msg = Twist()
        current_distance = 0
        circumfence = 2 * math.pi * self.circle_radius

        msg.linear.x = self.turtle_speed
        msg.angular.z = self.turtle_speed

        rate = rospy.Rate(5)

        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and current_distance <= circumfence:
            # calculate the position and update
            rospy.loginfo("Peri: %f Distance Travelled: %f",
                          circumfence, current_distance)
            rospy.loginfo("x: %f y: %f theta: %f", self.turtle_x,
                           self.turtle_y, self.turtle_theta)

            self.velocity_pub.publish(msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = (self.turtle_speed * self.circle_radius)
            current_distance *= (t1 - t0)

            rate.sleep()
	# calculate the position and update
        rospy.loginfo("Peri: %f Distance Travelled: %f",
                          circumfence, current_distance)
        rospy.loginfo("x: %f y: %f theta: %f", self.turtle_x,
                           self.turtle_y, self.turtle_theta)

        self.velocity_pub.publish(msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = (self.turtle_speed * self.circle_radius)
        current_distance *= (t1 - t0)

        rate.sleep()

        msg.linear.x = 0
        msg.angular.z = 0
        self.velocity_pub.publish(msg)

        rospy.loginfo("Goal reached")


if __name__ == "__main__":
    try:
        controller = TurtleController()
        controller.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
