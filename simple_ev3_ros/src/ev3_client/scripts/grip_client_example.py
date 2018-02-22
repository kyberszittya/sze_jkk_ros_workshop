import roslib
import rospy
import actionlib

from ev3_msgs.msg import GripSimpleAction, GripSimpleGoal, GripSimpleFeedback

def callback_grip_feedback(data):
    print(data.feedback.progress)

if __name__ == '__main__':
    rospy.init_node('grip_simple_action')
    client = actionlib.SimpleActionClient('grip_simple', GripSimpleAction)
    client.wait_for_server()
    print "Ready to grip"
    rospy.Subscriber("/grip_simple/feedback", GripSimpleFeedback, callback_grip_feedback)
    goal = GripSimpleGoal()
    goal.release = True
    client.send_goal(goal)
    client.wait_for_result()
    goal = GripSimpleGoal()
    goal.release = False
    client.send_goal(goal)
    client.wait_for_result()