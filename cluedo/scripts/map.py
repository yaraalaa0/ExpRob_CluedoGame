#!/usr/bin/env python

"""
.. module:: map.py
   :platform: Unix
   :synopsis: this file is an implementation of map server node
   
.. moduleauthor::  Yara Abdelmottaleb
 
This node implements the map server. 
It stores the dictionary of the rooms x and y coordinates. 
It also stores the x and y coordinates of the oracle.
When it receives a request, it replies back with the x and y coordinates of a random room or of the oracle depending on the request.

Services:
   /room_coords
  
"""

import rospy
from cluedo.srv import Room, RoomResponse
import random


rooms_dict = {'kitchen' : [2.0,4.5],  
              'bathroom' : [-3.0,1.5],
              'hall' : [-6.0,-2.0],
              'lounge' : [6.5,-3.0],
              'billiard' : [1.0,-7.0],
              'study' : [3.0,5.5],
              'library' : [-4.0,2.0],
              'dining' : [-2.5,-6.0]
              }
oracle_coords = [0.5, 1.0]

def send_room_coords(req):
    """
    This is the service callback function. When the service receives a request, it checks the randFlag.
    If the randFlag is true, it responds with the x and y coordinates of a random selected room.
    If the randFlag is false, it responds with the x and y coordinates of the oracle.
    
    Args:
      req(RoomRequest): the service request
    
    Returns:
      the result x and y coordinates depending on the request
     
    """
    rand_flag = req.randFlag
    if rand_flag:
        coords_list = list(rooms_dict.values())
        rand_coords = random.choice(coords_list)
        res = RoomResponse(x = rand_coords[0], y = rand_coords[1])
    else:
        res = RoomResponse(x = oracle_coords[0], y = oracle_coords[1])
    
    return res
 
    
def main():
    """
    This is the main function where the ROS node and the service /room_coords are initialized.
    
    """
    rospy.init_node('map')
    room_service = rospy.Service('/room_coords', Room, send_room_coords)
    rospy.spin()

if __name__ == '__main__':
    main()
