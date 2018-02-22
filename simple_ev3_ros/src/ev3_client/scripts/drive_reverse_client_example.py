import roslib
import rospy
import actionlib

from ev3_msgs.msg import DriveSimpleReverseAction, DriveSimpleReverseGoal, DriveSimpleReverseFeedback

def callback_drive_simple_feedback(data):
    print data.feedback.percentage

if __name__ == '__main__':
    rospy.init_node('drive_simple_reverse_action_client')
    client = actionlib.SimpleActionClient('drive_simple_reverse', DriveSimpleReverseAction)
    client.wait_for_server()
    print "Ready to move"
    rospy.Subscriber("/drive_simple_reverse/feedback", DriveSimpleReverseFeedback, callback_drive_simple_feedback)
    goal = DriveSimpleReverseGoal()
    goal.speed = 1000
    goal.duration = 2000
    client.send_goal(goal)
    client.wait_for_result()
    