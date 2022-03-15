#!/usr/bin/env python
"""
.. module:: oracle.py
   :platform: Unix
   :synopsis: this file is an implementation of oracle server node
   
.. moduleauthor:: Yara Abdelmottaleb
 
This node implements the oracle hint service and the oracle hypothesis checking service. 
It stores a dictionary of hypothesis IDs along with their possible hints including the empty hint. 
It also stores the ID of the coorect hypothesis.
When a /hint service request is received with an ID, it replies back with a random hint chosen from the possible hints of this hypothesis ID in the dictionary
When a /check_hyp service request is received with an ID, it checks whether this ID is the correct_ID or not. It replies back with a boolean value.

 
Services:
   /hint
   /check_hyp
  
"""

import rospy
from cluedo.srv import Hint, HintResponse
from cluedo.srv import HypCheck
import random

correct_ID = 'ID6'

hints_dict = {'ID1' : [ ['who','Green'], ['where','kitchen'], ['what','candlestick'], ['',''] ],  
              'ID2' : [ ['who','Plum'], ['where','lounge'], ['what','pipe'], ['',''] ],
              'ID3' : [ ['who','Mustard'], ['where','hall'], ['what','dagger'], ['',''] ],
              'ID4' : [ ['who','Green'], ['where','library'], ['what','revolver'], ['',''] ],
              'ID5' : [ ['who','Peacock'], ['where','study'], ['what','pipe'], ['',''] ],
              'ID6' : [ ['who','Plum'], ['where','hall'], ['what','rope'], ['',''] ],
              'ID7' : [ ['who','Peacock'], ['where','study'], ['what','revolver'], ['',''] ],
              'ID8' : [ ['who','White'], ['where','dining'], ['what','spanner'], ['',''] ],
              'ID9' : [ ['who','Green'], ['where','billiard'], ['what','spanner'], ['',''] ],
              'ID10': [ ['who','Mustard'], ['where','lounge'], ['what','dagger'], ['',''] ]
              }

def send_hint(req):
    """
    This function is the callback for the service /hint
    It receives a request with a specific hypothesis ID
    It selects a random hint for this ID drawn from its dictionary values. Whenever a hint is selected, it is deleted from the dictionary in order not to be selected again.
    It sends the selected hint as the service response 
    
    Args:
      req(HintRequest): the service request containing the hypothesis ID
    
    Returns:
      the service result containing the randomly selected hint
     
    """
    req_id = req.ID
    id_hints = hints_dict[req_id]
    #get the index of a random hint
    rand_hint_ind = random.randint(0,len(id_hints)-1)
    rand_hint = id_hints[rand_hint_ind]
    res = HintResponse(arg1 = rand_hint[0], arg2 = rand_hint[1])
    
    # delete the selected hint from the list in order not to be selected again
    if rand_hint_ind != len(id_hints)-1:
        #not the empty hint
        del id_hints[rand_hint_ind]
        hints_dict.update({req_id: id_hints})
    return res
  
def check(req):
    """
    This function is the callback for the service /check_hyp 
    It receives a request with a specific hypothesis ID, and checks whether this ID is the correct hypothesis ID or not.
    
    Args:
      req(HypCheckRequest): the service request containing the hypothesis ID to be checked
    
    Returns:
      a boolean value indicating whether this is the correct ID or not
     
    """
    return req.ID == correct_ID      
    
def main():
    """
    This is the main function where the node and the service /hint and /check_hyp are initialized
     
    """
    rospy.init_node('oracle')
    hint_service = rospy.Service('/hint', Hint, send_hint)
    check_service = rospy.Service('/check_hyp', HypCheck, check)
    rospy.spin()

if __name__ == '__main__':
    main()
