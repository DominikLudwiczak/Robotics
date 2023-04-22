#!/usr/bin/env python3
import rospy

import math
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf2_homework_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    base = rospy.get_param('base', 'base')

    pub = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans_lf = tfBuffer.lookup_transform(base, 'LF_SHANK', rospy.Time())
            trans_lh = tfBuffer.lookup_transform(base, 'LH_SHANK', rospy.Time())
            trans_rf = tfBuffer.lookup_transform(base, 'RF_SHANK', rospy.Time())
            trans_rh = tfBuffer.lookup_transform(base, 'RH_SHANK', rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "base"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "CoS"
        t.transform.translation.x = (trans_lf.transform.translation.x+trans_lh.transform.translation.x+trans_rf.transform.translation.x+trans_rh.transform.translation.x)/4
        t.transform.translation.y = (trans_lf.transform.translation.y+trans_lh.transform.translation.y+trans_rf.transform.translation.y+trans_rh.transform.translation.y)/4
        t.transform.translation.z = (trans_lf.transform.translation.z+trans_lh.transform.translation.z+trans_rf.transform.translation.z+trans_rh.transform.translation.z)/4

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        cos = tf2_msgs.msg.TFMessage([t])
        pub.publish(cos)

        rate.sleep()
