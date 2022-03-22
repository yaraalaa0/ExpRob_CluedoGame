# ExpRob_CluedoGame
This is a ROS implementation of an agent playing simplified Cluedo Game with knowledge representation as OWL ontology

|        Name                | Student ID |      Email Address       |
| :------------------------: | :--------: | :----------------------: |
|     Yara Abdelmottaleb     |  5066359   |  [yara.ala96@gmail.com](mailto:yara.ala96@gmail.com)   |


## Introduction:

This is a ROS implementation of an agent playing a simplified Cluedo Game. The agent goes to random rooms and collects hints in the form of (who, PERSON), (where, PLACE) and (what, WEAPON). When the agent's current working hypothesis is COMPLETE (that means it contains all three types of hints), it goes to a place called the Oracle where the hypothesis is checked. The agent continues to explore the environment, collect hints, and check hypothesis until it finds the correct hypothesis.

## Component Diagram:

The software architecture of the system is composed of six main components: 

- The knowledge base (ontology): this is the OWL ontology representing the current knowledge of the robot agent. In the beginning it contains the class definitions of HYPOTHESIS, COMPLETE, INCONSISTENT, PERSON, PLACE, and WEAPON, as well as the object properties definitions of (who, PERSON), (where, PLACE), and (what, WEAPON). As the robot explores the environment, new individuals and proberties assertions are added to the ontology.
- ARMOR Client: the armor service responsible for connection with the knowledge base for querying existing ontology or manipulating it. It is already implemented by [EmaroLab](https://github.com/EmaroLab/armor). 

![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/cluedo_comp_diag2.PNG?raw=true)

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

