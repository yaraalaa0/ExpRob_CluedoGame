# ExpRob_CluedoGame
ROS implementation of an agent playing simplified Cluedo Game with the help of OWL ontology and SMACH

To run the program, you need to install [ARMOR](https://github.com/EmaroLab/armor) in your ROS workspace. 
Then, you need to adapt the code in armor_py_api scripts to be in Python3 instead of Python2 (slight changes)

Add the path of the armor modules to your Python path:

`export PYTHONPATH=$PYTHONPATH:/root/ros_ws/src/armor/armor_py_api/scripts/armor_api/ `

Then, build the ROS workspace

`catkin_make`

To launch the program, run the following commands on different terminal tabs:

`roscore`

`rosrun armor execute it.emarolab.armor.ARMORMainService`

`roslaunch cluedo cluedo.launch`

