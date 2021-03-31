Laurent LEQUIEVRE<br/>
Research Engineer, CNRS (France)<br/>
Institut Pascal UMR6602<br/>
laurent.lequievre@uca.fr<br/>


# ROS Simulation - Platform Kuka - Institut Pascal UMR6602

Arms and Pantilt only :
----------------------

roslaunch platform_gazebo platform_gazebo.launch use_pantilt:=true<br/>


rostopic pub -1 /pantilt/pantilt_group_position_controller/command std_msgs/Float64MultiArray "data: [1.0,0.5,0.0,0.0]"<br/>

rostopic pub -1 /kuka_lwr_left/kuka_group_position_controller/command std_msgs/Float64MultiArray "data: [0.7,0.5,0.0,0.0,0.0,0.0,0.0]"<br/>

rosrun image_view image_view image:=/pantilt/camera1/image_raw<br/>
rosrun image_view image_view image:=/pantilt/camera2/image_raw<br/>


Arms, Pantilt and barrett hand :
------------------------------
roslaunch platform_gazebo platform_gazebo_with_hands.launch use_left_bh:=true use_right_bh:=true<br/>
roslaunch platform_gazebo platform_gazebo_with_hands.launch use_left_bh:=true use_pantilt:=false<br/>

* options :<br/>
use_pantilt:=true or false<br/>
use_left_bh:=true or false<br/>
use_right_bh:=true or false<br/>


How to move Barrett Hand : (namespace 'rbh' means 'right barrett hand', namespace 'lbh' means 'left barrett hand')
------------------------------------------------------------------------------------------------------------------
rostopic pub -1 /rbh/rbh_group_position_controller/command std_msgs/Float64MultiArray "data: [0.0,0.0,0.0,0.0]"<br/>

rostopic pub -1 /lbh/lbh_group_position_controller/command std_msgs/Float64MultiArray "data: [1.5,1.5,1.5,1.5]"<br/>


break away service -> rosservice call /rbh/rbh_group_position_controller/setBreakAway true<br/>

rostopic echo /rbh/joint_states<br/>

Contacts :
--------
rostopic echo /contacts/rbh/finger1/distal<br/>
rostopic echo /contacts/rbh/finger1/knuckle<br/>
rostopic echo /contacts/rbh/finger1/proximal<br/>
rostopic echo /contacts/rbh/finger2/distal<br/>
rostopic echo /contacts/rbh/finger2/knuckle<br/>
rostopic echo /contacts/rbh/finger2/proximal<br/>
rostopic echo /contacts/rbh/finger3/distal<br/>
rostopic echo /contacts/rbh/finger3/proximal<br/>
rostopic echo /contacts/rbh/palm<br/>

