#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal

class GoalRelay:
    def __init__(self):
        rospy.init_node('goal_relay')

        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.loginfo("Subscribed to /move_base_simple/goal")

        rospy.spin()

    def goal_callback(self, msg):
        goal = MoveBaseGoal()
        goal.target_pose = msg
        rospy.loginfo("Relaying goal to /move_base action server")
        self.client.send_goal(goal)

if __name__ == '__main__':
    try:
        GoalRelay()
    except rospy.ROSInterruptException:
        pass
