#!/usr/bin/env python
import rospy
import actionlib
from math import pi
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose2D, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback

class GoalController:
    def __init__(self):
        rospy.init_node("goal_controller")
        rospy.Subscriber("/send_goal", Pose2D, self.push_goal_to_array)
        self.current_goal = None
        self.current_goal_state = None
        self.target_array = []
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()
        pass

    def push_goal_to_array(self,data):
        self.target_array.append(data)

    def proccess_target(self, value):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = value.x
        goal.target_pose.pose.position.y = value.y
        q = quaternion_from_euler(0,0,value.theta/(2*pi))
        goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        return goal
    
    def set_current_goal(self):
        self.current_goal = self.proccess_target(self.target_array[0])

    def feedback_cb(self,data):
        print(data)
    


if __name__ == "__main__":
    goal_controller = GoalController()
    print("RosNode Started")
    rate = rospy.Rate(300)


        # Waiting For Target
            # Check Target Length
        # Processing Target
            # Convert Target to Goal
            # Set Current Goal
        # Sending Goal
            # Send Goal
        # Checking Goal State
            # Check Goal State
        # Popping Target Array
            # Remove Target from Array 

    current_state = 0

    def check_target_array():
        global current_state
        if len(goal_controller.target_array):
            current_state = 1

    def proccess_target():
        global current_state, goal_controller
        goal_controller.set_current_goal()
        current_state = 2
            

    def send_goal():
        global current_state, goal_controller
        goal_controller.client.send_goal(goal_controller.current_goal, feedback_cb=goal_controller.feedback_cb)
        current_state = 3

    def check_goal():
        global current_state, goal_controller
        state = goal_controller.client.get_state()
        if state > 1:
            current_state = 4

    def remove_target():
        global current_state, goal_controller
        goal_controller.target_array.pop(0)
        current_state = 0
    
    
    
    switch={
        0:check_target_array,
        1:proccess_target,
        2:send_goal,
        3:check_goal,
        4:remove_target
    }

    while not rospy.is_shutdown():
        print('Main Loop')
        print(len(goal_controller.target_array))
        print("Current State: " + str(current_state))

        func = switch.get(current_state,lambda : "Invalid State")
        func()

        rate.sleep()
