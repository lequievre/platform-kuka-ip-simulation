/*
 *  Laurent LEQUIEVRE
 *  Research Engineer, CNRS (France)
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
 * Fingers configuration :
 * =====================
 * Finger1_KNUCKLE (j11_joint) <-> Finger1_PROXIMAL (j12_joint) <-> Finger1_DISTAL (j13_joint)
 * Finger2_KNUCKLE (j21_joint) <-> Finger2_PROXIMAL (j22_joint) <-> Finger2_DISTAL (j23_joint)
 * Finger3_PROXIMAL (j32_joint) <-> Finger3_DISTAL (j33_joint)
 * 
 * Command configuation (4 angle values in radian) : [index0,index1,index2,index3,index4]
 * ===============================================
 * For a left barrett hand -> rostopic pub -1 /lbh/lbh_group_position_controller/command std_msgs/Float64MultiArray "data: [0.0,0.0,0.0,0.0]"
 * For a right barrett hand -> rostopic pub -1 /rbh/rbh_group_position_controller/command std_msgs/Float64MultiArray "data: [0.0,0.0,0.0,0.0]"
 * 
 * Without break away (by default)
 * index 0 (angle0) -> Finger 1 Knuckle  : angle0  && Finger 2 Knuckle : angle0
 * index 1 (angle1) -> Finger 1 Proximal : angle1  && Finger 1 Distal  : angle1 * 0.344
 * index 2 (angle2) -> Finger 2 Proximal : angle2  && Finger 2 Distal  : angle2 * 0.344
 * index 3 (angle3) -> Finger 3 Proximal : angle3  && Finger 3 Distal  : angle3 * 0.344
 * 
 * With break away (set by a service)
 * For a left -> rosservice call /lbh/lbh_group_position_controller/setBreakAway true
 * For a right -> rosservice call /rbh/rbh_group_position_controller/setBreakAway true
 * 
 * index 0 (angle0) -> Finger 1 Knuckle  : angle0                 && Finger 2 Knuckle : angle0
 * index 1 (angle1) -> Finger 1 Proximal : fixed to current value && Finger 1 Distal  : angle1
 * index 2 (angle2) -> Finger 2 Proximal : fixed to current value && Finger 2 Distal  : angle2
 * index 3 (angle3) -> Finger 3 Proximal : fixed to current value && Finger 3 Distal  : angle3
 * 
*/

#ifndef BARRETT_HAND_GROUP_POSITION_CONTROLLER_H
#define BARRETT_HAND_GROUP_POSITION_CONTROLLER_H

#define TRACE_ACTIVATED 0

// ROS
#include <controller_interface/controller.h>

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface

// URDF
#include <urdf/model.h>

// KDL
#include <kdl/kdl.hpp>
#include <kdl/jntarrayacc.hpp>

// msgs Float64MultiArray
#include <std_msgs/Float64MultiArray.h>

// srvs SetBool
#include <std_srvs/SetBool.h>

// ros tools for realtime_buffer
#include <realtime_tools/realtime_buffer.h>

namespace barrett_hand_controllers
{
	class BarrettHandGroupPosition: public controller_interface::Controller<hardware_interface::PositionJointInterface>
	{
		public:
			BarrettHandGroupPosition();
			~BarrettHandGroupPosition();
			
			bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n); // Init the controller
			
			void starting(const ros::Time& time);  // Start the controller
			void stopping(const ros::Time& time);  // Stop the controller
			void update(const ros::Time& time, const ros::Duration& period);  // Update the controller
			
		private:
		
			// Useful to convert enum Class to int (necessary if you need to use enum Class as index of array)
			template <typename T>
			constexpr typename std::underlying_type<T>::type enum_value(T val)
			{
				return static_cast<typename std::underlying_type<T>::type>(val);
			}
			
			
			// Define "Break Away" option for grasping.
			enum class GraspingMode : int
			{
				Break_Away_Active = 0,
				Break_Away_Inactive = 1
			};
			
			// Define index value of each joints.
			enum class Fingers : int
			{
				Finger1_KNUCKLE = 0,
				Finger1_PROXIMAL = 1,
				Finger1_DISTAL = 2,
				Finger2_KNUCKLE = 3,
				Finger2_PROXIMAL = 4,
				Finger2_DISTAL = 5,
				Finger3_PROXIMAL = 6,
				Finger3_DISTAL = 7
			};
		
			std::string robot_namespace_; // name of namespace
			ros::NodeHandle nh_; // Node Handle of controller
			
			// configuration
			int n_joints_ = 8; // the barret hand has 8 joints maximum
			int n_dof_ = 4; // 4 degrees of freedom (for command)
			std::vector<std::string> joint_names_; // vector of joint names
			
			std::vector<hardware_interface::PositionJointInterface::ResourceHandleType> joint_handles_; // Vector of Joint HW Handles
			
			// KDL structures to store limits min, max of each joint
			struct limits_
			{
				KDL::JntArray min;
				KDL::JntArray max;
			} joint_limits_; 
			
			void commandCB_(const std_msgs::Float64MultiArrayConstPtr& msg); // function associate to a subscribe command topic
			bool setBreakAwayCB_(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res); // service function to set the break away mode
			void checkLimit_(double &value, const Fingers& index) const noexcept; // function to verify joint limit
			std::vector<std::string> getStrings_(const ros::NodeHandle& nh, const std::string& param_name) const noexcept; // function to get the name of joints by reading rosparam param of controller
			
			ros::Subscriber sub_command_; // ros subsciber for topic command
			ros::ServiceServer srv_command_; // ros service server for service command 
			realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_; // buffer of desired joint values
			realtime_tools::RealtimeBuffer<GraspingMode> grasping_mode_buffer_; // buffer of grasping mode (Break Away active or not)
	}; // End of class BarrettHandGroupPosition
}

#endif
