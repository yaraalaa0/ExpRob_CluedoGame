# ExpRob_CluedoGame
This is a ROS implementation of a robot agent playing simplified Cluedo Game with knowledge representation in OWL ontology

|        Name                | Student ID |      Email Address       |
| :------------------------: | :--------: | :----------------------: |
|     Yara Abdelmottaleb     |  5066359   |  [yara.ala96@gmail.com](mailto:yara.ala96@gmail.com)   |


## Introduction:

This is a ROS implementation of an agent playing a simplified Cluedo Game. The agent goes to random rooms and collects hints in the form of (who, PERSON), (where, PLACE) and (what, WEAPON). When the agent's current working hypothesis is COMPLETE (that means it contains all three types of hints), it goes to a place called the Oracle where the hypothesis is checked. The agent continues to explore the environment, collect hints, and check hypothesis until it finds the correct hypothesis.

## Component Diagram:

The software architecture of the system is composed of six main components: 

- The knowledge base (ontology): this is the OWL ontology representing the current knowledge of the robot agent. In the beginning it contains the class definitions of HYPOTHESIS, COMPLETE, INCONSISTENT, PERSON, PLACE, and WEAPON, as well as the object properties definitions of (who, PERSON), (where, PLACE), and (what, WEAPON). As the robot explores the environment, new individuals and proberties assertions are added to the ontology.
- ARMOR: the armor service responsible for connection with the knowledge base for querying the ontology or manipulating it. It is fully implemented by [EmaroLab](https://github.com/EmaroLab/armor). 
- State Machine: this is the state manager of the robot. It is responsible for controlling the transitions between different robot states (GoToRandomRoom, LookForHints, GoToOracle, and CheckHypothesis). It also implements the robot behaviour in each state. It communicates with the other servers through different ROS messages. The ROS messages and parameters are indicated in the component diagram.
- Map Server: this is the component holding the (x, y) poistion of all rooms in the map. The service request is composed of a flag (bool randFlag) indicating whether the server should return a random room position (randFlag = True) or the oracle postion (randFlag = False)
- Motion Controller: this is the action server responsible for dricing the robot towards a target (x,y) position. For now, it is implemented as a simple waiting function for 5 seconds. 
- Oracle: this is the component holding the dictionary of possible hypothesis IDs along with their hints. Each hypothesis ID has 4 hints: one is the empty hint, and the others are (who, PERSON), (what, WEAPON), and (where, PLACE). Whenever the oracle receives a request for the service (/hint) with a specific ID, it returns back a random hint corresponding to this ID. If a hint is sent once, it is deleted from the dictionary in order not to be sent again. This component also holds the correct hypothesis ID. Whenever it receives a request for the service (/check_hyp) with a specific ID, it returns back whether this is the correct hypothesis ID or not.

![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/cluedo_comp.PNG?raw=true)

## State Diagram:


![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/cluedo_state_diag.PNG?raw=true)

## Sequence Diagram:

![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/cluedo_seq_diag2.PNG?raw=true)

## Installation and Running Procedures:

To run the program, you need first to install [ARMOR](https://github.com/EmaroLab/armor) in your ROS workspace.

Then, you need to adapt the code in armor_py_api scripts to be in Python3 instead of Python2:
  - add "from armor_api.armor_exceptions import ArmorServiceInternalError, ArmorServiceCallError" in armor_client.py
  - replace all "except rospy.ServiceException, e" with "except rospy.ServiceException as e"
  - modify line 132 of armor_query_client with: "if res.success and len(res.queried_objects) > 1:"

Add the path of the armor modules to your Python path:

`export PYTHONPATH=$PYTHONPATH:/root/ros_ws/src/armor/armor_py_api/scripts/armor_api/ `

Download this repository to your workspace. Then, build it

`catkin_make`

To launch the program, run the following commands on different terminal tabs:

`roscore`

`rosrun armor execute it.emarolab.armor.ARMORMainService`

`roslaunch cluedo cluedo.launch`

To display the states:

`rosrun smach ...`

