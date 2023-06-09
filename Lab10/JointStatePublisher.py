import rospy
import numpy as np
from sensor_msgs.msg import JointState
import message_filters
from kinematics.utilities import create_joint_state_msg


class JointStatePublisher:
    def __init__(self):
        self.rate = rospy.get_param("~rate", 10)
        inv_kin_topic = rospy.get_param("~inv_kin_joint_states", "inv_kin_joint_states")
        joint_state_topic = rospy.get_param("~joint_states_topic", "joint_states")

        ### [TODO IK]: Write a publisher that publishes JointState message type on a joint_state_topic topic
        self.j_pub = rospy.publisher(joint_state_topic, JointState, queue_size=10)

        ### [TODO IK]: Write a subscriber:
        ### * do not register callback
        ### * use message filters http://wiki.ros.org/message_filters to create cache
        self.j_sub = message_filters.Subscriber(self.inv_kin_topic, JointState)
        self.j_cache = message_filters.Cache(self.j_sub, cache_size=100, allow_headerless=True)

        ### [TODO IK]: Create some initial state message that contains joint angles:
        ### * you might use create_joint_state_msg(...) function for that
        self.state = create_joint_state_msg(np.zeros((6,),dtype=int))

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            ### [TODO IK]: implement loop that publishes self.state when it changes:
            ### * get last cached message from created self.j_cache Cache object
            ### * remember to update self.state.header.stamp with a current time
            msg = self.j_cache.getLast()
            if msg is None:
                self.state = msg
            elif self.state != msg:
                self.state = msg
                self.state.header.stamp = rospy.Time.now()
                self.j_pub.publish(self.state)
            ###
            rate.sleep()
