#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
import actionlib
from armor_api.armor_client import ArmorClient
from cluedo.msg import GoToPointAction
from cluedo.msg import GoToPointFeedback
from cluedo.msg import GoToPointResult
from cluedo.msg import GoToPointGoal
from cluedo.srv import Hint, HintRequest
from cluedo.srv import HypCheck, HypCheckRequest
from cluedo.srv import Room, RoomRequest, RoomResponse

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

path = "/root/Desktop/"
ontology_IRI = "http://www.emarolab.it/cluedo-ontology"



def user_action():
    return random.choice(['reached','hyp_non_comp','hyp_comp','hyp_false'])

    

# define state GoToRandomRoom
class GoToRandomRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['reached','hyp_non_comp','hyp_comp','hyp_false'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GoToRandomRoom ')
        #get a random room coordinates
        req = RoomRequest(randFlag = True)
        room_coords = map_client(req)
        #go to this random room
        goal = GoToPointGoal(x = room_coords.x, y= room_coords.y)
        motion_action_client.send_goal(goal)
        motion_action_client.wait_for_result()
        result = motion_action_client.get_result()
        while result.reached != True:
            #get another random room coordinates
            req = RoomRequest(randFlag = True)
            room_coords = map_client(req)
            #go to this random room
            goal = GoToPointGoal(x = room_coords.x, y= room_coords.y)
            motion_action_client.send_goal(goal)
            motion_action_client.wait_for_result()
            result = motion_action_client.get_result()
        print("Reached the Random Room")
        return 'reached'
        
            

# define state LookForHints
class LookForHints(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['reached','hyp_non_comp','hyp_comp','hyp_false'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        rospy.loginfo('Executing state LookForHints')
        
        #get a random hint from the oracle
        req = HintRequest(ID = currID)
        hint_args = oracle_hint_client(req)
        print("The hint is:")
        print(hint_args)
        command = hint_args.arg1
        thing = hint_args.arg2
        
        #add the hint to the current hypothesis properties
        if hint_args.arg1 == "what":
            armor_client.manipulation.add_ind_to_class(thing, "WEAPON")
            armor_client.manipulation.add_objectprop_to_ind(hint_args.arg1, currID, hint_args.arg2)
        elif hint_args.arg1 == "who":
            armor_client.manipulation.add_ind_to_class(thing, "PERSON")
            armor_client.manipulation.add_objectprop_to_ind(hint_args.arg1, currID, hint_args.arg2)
        elif hint_args.arg1 == "where":
            armor_client.manipulation.add_ind_to_class(thing, "PLACE")
            armor_client.manipulation.add_objectprop_to_ind(hint_args.arg1, currID, hint_args.arg2)
        #apply the changes
        armor_client.utils.apply_buffered_changes()
        armor_client.utils.sync_buffered_reasoner()
        
        #get the list of current complete hypotheses
        complete_hyp_links = armor_client.query.ind_b2_class("COMPLETED")
        complete_hyp = [x.replace("<"+ontology_IRI+"#", '').replace('>','') for x in complete_hyp_links]
        
        #check if the current hypothesis is complete
        if currID in complete_hyp:
            print("hypothesis %s is COMPLETE"%currID)
            return "hyp_comp"
        else:
            print("Hypothesis not complete yet! Keep searching for hints")
            return "hyp_non_comp"
        
# define state GoToOracle
class GoToOracle(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['reached','hyp_non_comp','hyp_comp','hyp_false'])
        

    def execute(self, userdata):
        rospy.loginfo('Executing state GoToOracle')
        #get a random room coordinates
        req = RoomRequest(randFlag = False)
        oracle_coords = map_client(req)
        #go to this random room
        goal = GoToPointGoal(x = oracle_coords.x, y= oracle_coords.y)
        motion_action_client.send_goal(goal)
        motion_action_client.wait_for_result()
        result = motion_action_client.get_result()
        while result.reached != True:
            #get another random room coordinates
            req = RoomRequest(randFlag = False)
            oracle_coords = map_client(req)
            #go to this random room
            goal = GoToPointGoal(x = oracle_coords.x, y= oracle_coords.y)
            motion_action_client.send_goal(goal)
            motion_action_client.wait_for_result()
            result = motion_action_client.get_result()
        print("Reached the Oracle")
        return 'reached'
            

# define state CheckHypothesis
class CheckHypothesis(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['reached','hyp_non_comp','hyp_comp','hyp_false'])
        

    def execute(self, userdata):
        global currID
        rospy.loginfo('Executing state CheckHypothesis')
        #check if hypothesis is correct or not
        req = HypCheckRequest(ID = currID)
        res = oracle_check_client(req)
        if res.correct:
            print("THIS IS THE CORRECT ANSWER! Please, terminate the program")
            armor_client.utils.save_ref_with_inferences(path + "cluedo_submitted.owl")
            time.sleep(120)
            return "hyp_comp"
        else:
            print("Hypothesis is FALSE")
            # Get a new random hypothesis ID
            randInd = random.randint(0,len(IDs)-1)  #get a random index of the IDs list
            currID = 'ID'+str(IDs[randInd])  #get the current hypothesis ID to be investigated (chosen randomly)
            del IDs[randInd]    #delete this ID from the list so as not to be chosen again
            print("new hypothesis ID is %s"%currID)
            return "hyp_false"
            

def main():
    global armor_client
    global motion_action_client, oracle_hint_client, oracle_check_client, map_client
    global IDs
    global currID
    
    rospy.init_node('state_machine')
    
    # Start ARMOR client and load the cluedo ontology
    
    armor_client = ArmorClient("client", "reference")
    armor_client.utils.load_ref_from_file(path + "cluedo_ontology.owl", ontology_IRI,
                                True, "PELLET", True, False)  # initializing with buffered manipulation and reasoning
    armor_client.utils.mount_on_ref()
    armor_client.utils.set_log_to_terminal(True)
    
    # Initialize the motion controller action client
    motion_action_client = actionlib.SimpleActionClient('/go_to_point', GoToPointAction)
    motion_action_client.wait_for_server()
    
    # Initialize the oracle and map service clients
    oracle_hint_client = rospy.ServiceProxy('/hint',Hint)
    oracle_check_client = rospy.ServiceProxy('/check_hyp',HypCheck)
    map_client = rospy.ServiceProxy('/room_coords',Room)
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GoToRandomRoom', GoToRandomRoom(), 
                               transitions={'reached':'LookForHints', 
                                            'hyp_non_comp':'GoToRandomRoom', 
                                            'hyp_comp':'GoToRandomRoom', 
                                            'hyp_false':'GoToRandomRoom'})
        smach.StateMachine.add('LookForHints', LookForHints(), 
                               transitions={'reached':'LookForHints', 
                                            'hyp_non_comp':'GoToRandomRoom', 
                                            'hyp_comp':'GoToOracle',
                                            'hyp_false':'LookForHints'})
        smach.StateMachine.add('GoToOracle', GoToOracle(), 
                               transitions={'reached':'CheckHypothesis', 
                                            'hyp_non_comp':'GoToOracle', 
                                            'hyp_comp':'GoToOracle', 
                                            'hyp_false':'GoToOracle'})
        smach.StateMachine.add('CheckHypothesis', CheckHypothesis(), 
                               transitions={'reached':'CheckHypothesis', 
                                            'hyp_non_comp':'CheckHypothesis', 
                                            'hyp_comp':'CheckHypothesis',
                                            'hyp_false':'GoToRandomRoom'})
    
    IDs = [1,2,3,4,5,6,7,8,9,10] #the hypothesis IDs
    
    randInd = random.randint(0,len(IDs)-1)  #get a random index of the IDs list
    currID = 'ID'+str(IDs[randInd])  #get the current hypothesis ID to be investigated (chosen randomly)
    del IDs[randInd]    #delete this ID from the list so as not to be chosen again
    
    print("New hypothesis ID is %s"%currID)
    
    #add it to the hypothesis class
    armor_client.manipulation.add_ind_to_class(currID, "HYPOTHESIS")
    armor_client.manipulation.add_dataprop_to_ind("hasID", currID, "STRING", currID)
    #apply the changes
    armor_client.utils.apply_buffered_changes()
    armor_client.utils.sync_buffered_reasoner()
    
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
