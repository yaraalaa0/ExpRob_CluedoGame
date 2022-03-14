#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random

# INSTALLATION
# - create ROS package in your workspace:
#          $ catkin_create_pkg smach_tutorial std_msgs rospy
# - move this file to the 'smach_tutorial/scr' folder and give running permissions to it with
#          $ chmod +x state_machine.py
# - run the 'roscore' and then you can run the state machine with
#          $ rosrun smach_tutorial state_machine.py
# - install the visualiser using
#          $ sudo apt-get install ros-kinetic-smach-viewer
# - run the visualiser with
#          $ rosrun smach_viewer smach_viewer.py

def user_action():
    return random.choice(['tired','command','gesture'])

# define state GoAtHome
class GoAtHome(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['tired','wakeup','command','gesture'])
        self.timer = 0
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(1)
        rospy.loginfo('Executing state GoAtHome')
        self.timer = self.timer + 1
        if self.timer > 3:
            self.timer = 0
            return 'wakeup'
        return user_action()
    
# define state RandomRoam
class RandomRoam(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['tired','wakeup','command','gesture'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(4)
        rospy.loginfo('Executing state RandomRoam')
        return user_action()

# define state GoAtPerson
class GoAtPerson(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['tired','wakeup','command','gesture'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(3)
        rospy.loginfo('Executing state GoAtPerson')
        return user_action()

# define state GoAtGesture
class GoAtGesture(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['tired','wakeup','command','gesture'])
        self.timer = 0
    def execute(self, userdata):
        time.sleep(1)
        rospy.loginfo('Executing state GoAtGesture')
        self.timer = self.timer + 1
        if self.timer > 3:
            self.timer = 0
            return 'wakeup'
        return user_action()
        

        
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GoAtHome', GoAtHome(), 
                               transitions={'tired':'GoAtHome', 
                                            'wakeup':'RandomRoam',
                                            'command':'GoAtHome',
                                            'gesture':'GoAtHome'})
        smach.StateMachine.add('RandomRoam', RandomRoam(), 
                               transitions={'tired':'GoAtHome', 
                                            'wakeup':'RandomRoam',
                                            'command':'GoAtPerson',
                                            'gesture':'RandomRoam'})
        smach.StateMachine.add('GoAtPerson', GoAtPerson(), 
                               transitions={'tired':'GoAtHome', 
                                            'wakeup':'GoAtPerson',
                                            'command':'GoAtPerson',
                                            'gesture':'GoAtGesture'})
        smach.StateMachine.add('GoAtGesture', GoAtGesture(), 
                               transitions={'tired':'GoAtHome', 
                                            'wakeup':'RandomRoam',
                                            'command':'GoAtGesture',
                                            'gesture':'GoAtGesture'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
