roslaunch platform_gazebo platform_gazebo.launch use_pantilt:=true


rostopic pub -1 /pantilt/pantilt_group_position_controller/command std_msgs/Float64MultiArray "data: [1.0,0.5,0.0,0.0]"

rostopic pub -1 /kuka_lwr_left/kuka_group_position_controller/command std_msgs/Float64MultiArray "data: [0.7,0.5,0.0,0.0,0.0,0.0,0.0]"


