#!/usr/bin/env python

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
    rand_flag = req.randFlag
    if rand_flag:
        coords_list = list(rooms_dict.values())
        rand_coords = random.choice(coords_list)
        res = RoomResponse(x = rand_coords[0], y = rand_coords[1])
    else:
        res = RoomResponse(x = oracle_coords[0], y = oracle_coords[1])
    
    return res
 
    
def main():
    rospy.init_node('map')
    room_service = rospy.Service('/room_coords', Room, send_room_coords)
    rospy.spin()

if __name__ == '__main__':
    main()
