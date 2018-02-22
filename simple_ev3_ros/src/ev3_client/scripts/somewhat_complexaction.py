import roslib
import rospy
import actionlib

from ev3_msgs.msg import DriveUntilDistanceAction, DriveUntilDistanceGoal, DriveUntilDistanceFeedback
from ev3_msgs.msg import GripSimpleAction, GripSimpleGoal, GripSimpleFeedback
from ev3_msgs.msg import DriveSimpleAction, DriveSimpleGoal, DriveSimpleFeedback


def callback_drive_until_feedback(data):
    print "Current distance (DRIVE_UNTIL_FEEDBACK): "+str(data.feedback.current_distance)

def callback_grip_feedback(data):
    print "Progress (GRIP): "+str(data.feedback.progress)

def callback_drive_simple_feedback(data):
    print "Progress (DRIVE_SIMPLE): "+str(data.feedback.percentage)

if __name__ == '__main__':
    rospy.init_node('somewhat_complex_action')
    client_distance = actionlib.SimpleActionClient('drive_until_distance', DriveUntilDistanceAction)
    client_grip = actionlib.SimpleActionClient('grip_simple', GripSimpleAction)
    client_drive = actionlib.SimpleActionClient('drive_simple', DriveSimpleAction)
    client_distance.wait_for_server()
    client_grip.wait_for_server()
    client_drive.wait_for_server()
    
    rospy.Subscriber("/drive_until_distance/feedback", DriveUntilDistanceFeedback, callback_drive_until_feedback)
    rospy.Subscriber("/grip_simple/feedback", GripSimpleFeedback, callback_grip_feedback)
    rospy.Subscriber("/drive_simple/feedback", DriveSimpleFeedback, callback_drive_simple_feedback)
    print "Ready to grip & go"
    goal_grip = GripSimpleGoal()
    goal_grip.release = True
    client_grip.send_goal(goal_grip)
    client_grip.wait_for_result()    
    goal = DriveUntilDistanceGoal()
    goal.target_distance = 40
    goal.speed = 1000
    client_distance.send_goal(goal)
    client_distance.wait_for_result()
    goal_grip = GripSimpleGoal()
    goal_grip.release = False
    client_grip.send_goal(goal_grip)
    client_grip.wait_for_result()
    goal_drive = DriveSimpleGoal()
    goal_drive.speed= -1000
    goal_drive.duration = 2000
    client_drive.send_goal(goal_drive)
    client_drive.wait_for_result()
    