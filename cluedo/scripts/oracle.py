#!/usr/bin/env python

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
    req_id = req.ID
    id_hints = hints_dict[req_id]
    #get the index of a random hint
    rand_hint_ind = random.randint(0,len(id_hints)-1)
    rand_hint = id_hints[rand_hint_ind]
    #print(rand_hint)
    res = HintResponse(arg1 = rand_hint[0], arg2 = rand_hint[1])
    
    # delete the selected hint from the list in order not to be selected again
    if rand_hint_ind != len(id_hints)-1:
        #not the empty hint
        del id_hints[rand_hint_ind]
        hints_dict.update({req_id: id_hints})
    return res
  
def check(req):
    return req.ID == correct_ID      
    
def main():
    rospy.init_node('oracle')
    hint_service = rospy.Service('/hint', Hint, send_hint)
    check_service = rospy.Service('/check_hyp', HypCheck, check)
    rospy.spin()

if __name__ == '__main__':
    main()
