import roslib
import rospy
import actionlib

from ev3_msgs.msg import DriveUntilDistanceAction, DriveUntilDistanceGoal, DriveUntilDistanceFeedback

def callback_drive_until_feedback(data):
    print data.feedback.current_distance

if __name__ == '__main__':
    rospy.init_node('drive_until_distance_action')
    client = actionlib.SimpleActionClient('drive_until_distance', DriveUntilDistanceAction)
    client.wait_for_server()
    print "Ready to go"
    rospy.Subscriber("/drive_until_distance/feedback", DriveUntilDistanceFeedback, callback_drive_until_feedback)
    goal = DriveUntilDistanceGoal()
    goal.target_distance = 30
    goal.speed = 1000
    client.send_goal(goal)
    client.wait_for_result()