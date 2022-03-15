"""
.. module:: motion_controller.py
   :platform: Unix
   :synopsis: this file is an implementation of motion controller action server node
   
.. moduleauthor::  Yara Abdelmottaleb
 
This node implements the motion controller action server. 
When an action request is received with a target (x,y) position, the server mimicks the motion towards the target using a simple waiting for 5 seconds.
If a cancel request is received, the action is aborted.
 
Services:
   /go_to_point
  
"""

import rospy
import actionlib
import time

from cluedo.msg import GoToPointAction
from cluedo.msg import GoToPointFeedback
from cluedo.msg import GoToPointResult
from cluedo.msg import GoToPointGoal

#define action server as a global variable
action_service = None

def execute_action(goal):
    """
    This is the callback function for the action service /go_to_point
    It mimicks the execution of a motion controller algorithm that drives the robot towards a target (x,y) position using a simple time waiting for 5 seconds.
    If a cancel request is received, the action is aborted.
    After 5 seconds, it sets the reached flag of the result to true and publishes it.
    
    Args:
      goal(GoToPointGoal): the action request composing of x and y float values
    
     
    """
    result = GoToPointResult()
    feedback = GoToPointFeedback()
    result.reached = False
    count = 0
    while count < 5:
        if action_service.is_preempt_requested():
            action_service.set_preempted()
            break
        feedback.x = 0
        feedback.y = 0
        action_service.publish_feedback(feedback)
        time.sleep(1)
        count = count + 1
    result.reached = True
    action_service.set_succeeded(result)
    #print('reached the target')
        
def main():
    """
    This is the main function where the node and the action service /go_to_point are initialized
     
    """
    global action_service
    rospy.init_node('motion_controller')
    action_service = actionlib.SimpleActionServer('/go_to_point', GoToPointAction, execute_cb = execute_action, auto_start = False)
    action_service.start()
    rospy.spin()
    
if __name__ == '__main__':
    main()
