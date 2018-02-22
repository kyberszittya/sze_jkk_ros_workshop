import rospy
import tf2_ros

# Based on http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29

if __name__ == '__main__':
    rospy.init_node('tf2_ur_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    start_link_name = 'base_link'
    mid_link_name = 'upper_arm_link'
    end_link_name ='wrist_3_link'
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans_cy = tfBuffer.lookup_transform(start_link_name, end_link_name, rospy.Time())
            trans_mid = tfBuffer.lookup_transform(mid_link_name, end_link_name, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        rate.sleep()
        print trans_cy.transform
        print trans_mid.transform
