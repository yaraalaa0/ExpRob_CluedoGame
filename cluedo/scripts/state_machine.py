#!/usr/bin/env python

"""
.. module:: state_machine.py
   :platform: Unix
   :synopsis: this file is an implementation of the state machine node
   
.. moduleauthor:: Yara Abdelmottaleb
 
This node implements the state machine of the robot playing Cluedo Game. 

There are four possible states:
    - GoToRandomRoom: the robot is going to a random room for exploration
    - LookForHints: the robot is looking for hints in the place it is currently in
    - GoToOracle: the robot is going to the oracle place
    - CheckHypothesis: the robot is checking whether its current collected hypothesis is true or not

There are, also, four possible events (state transitions):
    - reached: indicating an event that the robot reached its target position
    - hyp_non_comp: indicating an event that the robot checked the current hypothesis and found that it is not complete yet
    - hyp_comp: indicating an event that the robot checked the current hypothesis and found that it is complete
    - hyp_false: indicating that the robot checked in the oracle the current hypothesis and found that it is false.
 
Clients:
   /go_to_point
   /hint
   /check_hyp
   /room_coords
  
"""

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



path = "/root/Desktop/" #the path to cluedo ontology (.owl) file
ontology_IRI = "http://www.emarolab.it/cluedo-ontology"


class GoToRandomRoom(smach.State):
    """
    This is the class for GoToRandomRoom state. 
     
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['reached','hyp_non_comp','hyp_comp','hyp_false'])

    def execute(self, userdata):
        """
        This function executes the GoToRandomRoom state.
        It requests a random room coordinates from /room_coords service and then calls the /go_to_point service with the acquired room coordinates as the target. 
        It waits until the robot reaches its target. If it didn't, it keeps sending the target to the /go_to_point service unitl it is successfully reached. 
        
        
        Returns:
          the 'reached' state transition
     
        """
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
        
            

class LookForHints(smach.State):
    """
    This is the class for LookForHints state. 
     
    """
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['reached','hyp_non_comp','hyp_comp','hyp_false'])
        
    def execute(self, userdata):
        """
        This function executs the LookForHints state.
        It sends a request to /hint service with the current hypothesis ID, and it receives a random hint from the server.
        It adds the received hint to the ontology and, then, checks if the current hypothesis is completed or not.
        
    
        Returns:
          the 'hyp_comp' state transition if the current hypothesis is complete
          the 'hyp_non_comp' state transition if the current hypothesis is not complete
     
        """
        # function called when exiting from the node, it can be blacking
        rospy.loginfo('Executing state LookForHints')
        
        #get a random hint from the oracle
        req = HintRequest(ID = currID)
        hint_args = oracle_hint_client(req)
        print("The hint is:")
        print(hint_args)
        command = hint_args.arg1
        thing = hint_args.arg2
        
        #add the hint to the current hypothesis object properties
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
        

class GoToOracle(smach.State):
    """
    This is the class for GoToOracle state. 
     
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['reached','hyp_non_comp','hyp_comp','hyp_false'])
        

    def execute(self, userdata):
        """
        This function executes the GoToOracle state.
        It sends a service request to /room_coords to get the (x,y) coordinates of the oracle. 
        Then, it sends these coordinates as a target to /go_to_point service. It waits until the robot reaches its target. 
        If it didn't, it keeps sending the target to the /go_to_point service unitl it is successfully reached. 
        
        
        Returns:
          the 'reached' state transition
        
        """
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
            


class CheckHypothesis(smach.State):
    """
    This is the class for CheckHypothesis state. 
     
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['reached','hyp_non_comp','hyp_comp','hyp_false'])
        

    def execute(self, userdata):
        """
        This function executes the CheckHypothesis state.
        It sends a request to /check_hyp service with the current hypothesis ID (that is completed).
        If the service response indicates that the hypothesis is correct, it displayes the information on the screen, saves the ontology, and asks the user to terminate the program.
        If it is not correct, it selects a new random ID for the next hypothesis. 
        
        Returns:
          the 'hyp_false' state transition if the hypothesis is not correct.
          the 'hyp_comp' state transition if the hypothesis is correct (this transition should never be reached because the user should terminate the program)
        
        """
        global currID
        rospy.loginfo('Executing state CheckHypothesis')
        #check if hypothesis is correct or not
        req = HypCheckRequest(ID = currID)
        res = oracle_check_client(req)
        if res.correct:
            print("THIS IS THE CORRECT ANSWER! Please, terminate the program")
            armor_client.utils.save_ref_with_inferences(path + "cluedo_submitted.owl")
            #wait for two minutes to allow the user to terminate
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
    """
    This is the main function where the node, the Armor client, /hint client, /check_hyp client, and /room_coords client are initialized.
    The SMACH state machine is created with four states:
    - GoToRandomRoom: the robot is going to a random room for exploration
    - LookForHints: the robot is looking for hints in the place it is currently in
    - GoToOracle: the robot is going to the oracle place
    - CheckHypothesis: the robot is checking whether its current collected hypothesis is true or not
    
    The state transitions are also defined in the state machine.
    The list of possible hypothesis IDs is created and a random initial ID is selected to be the first hypothesis
    
    
    """
    global armor_client
    global motion_action_client, oracle_hint_client, oracle_check_client, map_client
    global IDs
    global currID
    
    rospy.init_node('state_machine')
    
    IDs = [1,2,3,4,5,6,7,8,9,10] #the hypothesis IDs
    
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
