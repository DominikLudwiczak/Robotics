#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Twist, Vector3
from turtlesim.msg import Pose

class Turtle:
    def __init__(self):
        self.eps = 0.05
        rospy.init_node('turtle', anonymous=True)
        self.turtle_position = Pose()
        rospy.Subscriber('/turtle1/goal', Point, self.move_to_goal)
        self.pose_listener = rospy.Subscriber('/turtle1/pose', Pose, self.update_turtle_pose)
        self.velocity_publisher = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 10)
        rospy.spin()


    def update_turtle_pose(self, pose):
        self.turtle_position = pose


    def move_to_goal(self, destination):
        dist_x = destination.x - self.turtle_position.x
        dist_y = destination.y - self.turtle_position.y
        while abs(dist_x) > self.eps or abs(dist_y) > self.eps:
            line_vel = Vector3(dist_x, dist_y, 0)
            line_ang = Vector3(0, 0, 0)
            tw = Twist(line_vel, line_ang)
            self.velocity_publisher.publish(tw)
            dist_x = destination.x - self.turtle_position.x
            dist_y = destination.y - self.turtle_position.y
            rospy.loginfo(f"{self.turtle_position.x}, {destination.x}")
            rospy.loginfo(f"{self.turtle_position.y}, {destination.y}")


if __name__ == '__main__':
    turtle = Turtle()