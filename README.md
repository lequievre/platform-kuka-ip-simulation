Laurent LEQUIEVRE<br/>
Research Engineer, CNRS (France)<br/>
Institut Pascal UMR6602<br/>
laurent.lequievre@uca.fr<br/>


# ROS Simulation - Platform Kuka - Institut Pascal UMR6602

Arms and Pantilt only :
----------------------

roslaunch platform_gazebo platform_gazebo.launch use_pantilt:=true


rostopic pub -1 /pantilt/pantilt_group_position_controller/command std_msgs/Float64MultiArray "data: [1.0,0.5,0.0,0.0]"

rostopic pub -1 /kuka_lwr_left/kuka_group_position_controller/command std_msgs/Float64MultiArray "data: [0.7,0.5,0.0,0.0,0.0,0.0,0.0]"

rosrun image_view image_view image:=/pantilt/camera1/image_raw
rosrun image_view image_view image:=/pantilt/camera2/image_raw


Arms, Pantilt and barrett hand :
------------------------------
roslaunch platform_gazebo platform_gazebo_with_hands.launch use_left_bh:=true use_right_bh:=true
roslaunch platform_gazebo platform_gazebo_with_hands.launch use_left_bh:=true use_pantilt:=false



How to move Barrett Hand : (namespace 'rbh' means 'right barrett hand', namespace 'lbh' means 'left barrett hand')
------------------------
rostopic pub -1 /rbh/rbh_group_position_controller/command std_msgs/Float64MultiArray "data: [0.0,0.0,0.0,0.0]"

rostopic pub -1 /lbh/lbh_group_position_controller/command std_msgs/Float64MultiArray "data: [1.5,1.5,1.5,1.5]"


break away service -> rosservice call /rbh/rbh_group_position_controller/setBreakAway true

rostopic echo /rbh/joint_states

Contacts :
--------
rostopic echo /contacts/rbh/finger1/distal
rostopic echo /contacts/rbh/finger1/knuckle
rostopic echo /contacts/rbh/finger1/proximal
rostopic echo /contacts/rbh/finger2/distal
rostopic echo /contacts/rbh/finger2/knuckle
rostopic echo /contacts/rbh/finger2/proximal
rostopic echo /contacts/rbh/finger3/distal
rostopic echo /contacts/rbh/finger3/proximal
rostopic echo /contacts/rbh/palm
