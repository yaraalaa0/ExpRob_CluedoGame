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
    global action_service
    rospy.init_node('motion_controller')
    action_service = actionlib.SimpleActionServer('/go_to_point', GoToPointAction, execute_cb = execute_action, auto_start = False)
    action_service.start()
    rospy.spin()
    
if __name__ == '__main__':
    main()
