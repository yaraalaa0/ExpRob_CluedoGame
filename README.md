# ExpRob_CluedoGame
ROS implementation of an agent playing simplified Cluedo Game with the help of OWL ontology and SMACH

[yara.ala96@gmail.com](mailto:yara.ala96@gmail.com)

Component Diagram:

![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/cluedo_comp_diag2.PNG?raw=true)

State Diagram:

![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/cluedo_state_diag.PNG?raw=true)

Sequence Diagram:

![alt text](https://github.com/yaraalaa0/ExpRob_CluedoGame/blob/main/cluedo_seq_diag2.PNG?raw=true)

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

