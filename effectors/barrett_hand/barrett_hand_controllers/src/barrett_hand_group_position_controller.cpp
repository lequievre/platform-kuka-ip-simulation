/*
 *  Laurent LEQUIEVRE
 *  Research Engineer, CNRS (France)
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#include <barrett_hand_group_position_controller.h>

// For plugin
#include <pluginlib/class_list_macros.h>

namespace barrett_hand_controllers 
{
	BarrettHandGroupPosition::BarrettHandGroupPosition() 
	{
		#if TRACE_ACTIVATED
			ROS_INFO("BarrettHandGroupPosition: constructor !");
		#endif
	}
	
	BarrettHandGroupPosition::~BarrettHandGroupPosition() 
	{
		#if TRACE_ACTIVATED
			ROS_INFO("BarrettHandGroupPosition: destructor !");
		#endif
	}
	
	void BarrettHandGroupPosition::starting(const ros::Time& time)
    {
		#if TRACE_ACTIVATED
			ROS_INFO("BarrettHandGroupPosition: Starting of robot %s !",robot_namespace_.c_str());
		#endif
    }
    
    void BarrettHandGroupPosition::stopping(const ros::Time& time)
	{
		#if TRACE_ACTIVATED
			ROS_INFO("BarrettHandGroupPosition: Stopping of robot %s !",robot_namespace_.c_str());
		#endif
	}
	
	std::vector<std::string> BarrettHandGroupPosition::getStrings_(const ros::NodeHandle& nh, const std::string& param_name) const noexcept
	{
	  using namespace XmlRpc;
	  XmlRpcValue xml_array;
	  
	  if (!nh.getParam(param_name, xml_array))
	  {
		ROS_ERROR_STREAM("BarrettHandGroupPosition:: Could not find '" << param_name << "' parameter (namespace: " << nh.getNamespace() << ").");
		return std::vector<std::string>();
	  }
	  
	  if (xml_array.getType() != XmlRpcValue::TypeArray)
	  {
		ROS_ERROR_STREAM("BarrettHandGroupPosition:: The '" << param_name << "' parameter is not an array (namespace: " <<
						 nh.getNamespace() << ").");
		return std::vector<std::string>();
	  }

	  std::vector<std::string> out;
	  for (int i = 0; i < xml_array.size(); ++i)
	  {
		if (xml_array[i].getType() != XmlRpcValue::TypeString)
		{
		  ROS_ERROR_STREAM("BarrettHandGroupPosition:: The '" << param_name << "' parameter contains a non-string element (namespace: " <<
						   nh.getNamespace() << ").");
		  return std::vector<std::string>();
		}
		out.push_back(static_cast<std::string>(xml_array[i]));
	  }
	  
	  return out;
	}
	
	bool BarrettHandGroupPosition::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
		#if TRACE_ACTIVATED
			ROS_INFO("BarrettHandGroupPosition -> (init) Start init of robot !");
		#endif
		
		nh_ = n;
		robot_namespace_ = nh_.getNamespace();
		
		#if TRACE_ACTIVATED
			ROS_INFO("BarrettHandGroupPosition -> (init) ros::NodeHandle namespace = %s", robot_namespace_.c_str());
		#endif
		
		// the 'getNamespace()' function return '/namespace_of_controller/name_of_controller'
		// There is a parameter named '/namespace_of_controller/name_of_controller/joints' 
		// This parameter is defined by the 'joints' field defined in the 'config_file_of_controllers.yaml' file.
		joint_names_.clear();
		joint_names_ = getStrings_(nh_,"joints");
		n_joints_ = joint_names_.size();
		
		if (n_joints_ == 0) {
			ROS_ERROR_STREAM("BarrettHandGroupPosition -> (init) List of joint names is empty.");
			return false;
		}
		
		#if TRACE_ACTIVATED
			ROS_INFO("BarrettHandGroupPosition -> (init) nb joints -> %zd", joint_names_.size());
		
			for (size_t i=0; i<joint_names_.size(); ++i)
				ROS_INFO("BarrettHandGroupPosition -> (init) name of joint %zd = %s", i, joint_names_[i].c_str());
		#endif

		// get URDF and name of root and tip from the parameter server
		std::string robot_description;

		if (!ros::param::search(robot_namespace_,"robot_description", robot_description))
		{
		    ROS_ERROR_STREAM("BarrettHandGroupPosition -> (init) No robot description (URDF) found on parameter server (" << robot_namespace_ << "/robot_description)");
		    return false;
		}
		
		// Construct an URDF model from the xml string
		std::string xml_string;

		if (n.hasParam(robot_description))
		    n.getParam(robot_description.c_str(), xml_string);
		else
		{
		    ROS_ERROR("BarrettHandGroupPosition -> (init) Parameter %s not set, shutting down node...",robot_description.c_str());
		    n.shutdown();
		    return false;
		}

		if (xml_string.size() == 0)
		{
		    ROS_ERROR("BarrettHandGroupPosition -> (init) Unable to load robot model from parameter %s",robot_description.c_str());
		    n.shutdown();
		    return false;
		}
		
		// Get urdf model out of robot_description
		urdf::Model model;
		if (!model.initString(xml_string))
		{
		    ROS_ERROR("BarrettHandGroupPosition -> (init) Failed to parse urdf file");
		    n.shutdown();
		    return false;
		}
		
		#if TRACE_ACTIVATED
			ROS_INFO("BarrettHandGroupPosition -> (init) Successfully parsed urdf file");
		#endif
		
		// Resize joints limits arrays with number of joints : n_joints_
		joint_limits_.min.resize(n_joints_);
		joint_limits_.max.resize(n_joints_);
		
		urdf::JointConstSharedPtr a_joint;  // A Joint defined in a URDF structure
		
		for (int i = 0; i < n_joints_; i++)
		{
		    a_joint = model.getJoint(joint_names_[i]);
		    
			#if TRACE_ACTIVATED
				ROS_INFO("BarrettHandGroupPosition -> (init) Getting limits for joint: %s", a_joint->name.c_str());
			#endif

		    joint_limits_.min(i) = a_joint->limits->lower;
		    joint_limits_.max(i) = a_joint->limits->upper;
		    
		    try
			{
				joint_handles_.push_back(robot->getHandle(joint_names_[i]));  
			}
			catch (const hardware_interface::HardwareInterfaceException& e)
			{
				ROS_ERROR_STREAM("BarrettHandGroupPosition -> (init) Exception thrown when robot getHandle : " << e.what());
				return false;
			}
		}
		
		#if TRACE_ACTIVATED
			ROS_INFO("BarrettHandGroupPosition -> (init) Number of joints in handle = %lu", joint_handles_.size() );
		#endif
        
        // set joint positions, inital values
        commands_buffer_.writeFromNonRT(std::vector<double>(n_dof_, 0.0));
        grasping_mode_buffer_.writeFromNonRT(GraspingMode::Break_Away_Inactive);
        
		sub_command_ = nh_.subscribe("command", 1, &BarrettHandGroupPosition::commandCB_, this);
		srv_command_ = nh_.advertiseService("setBreakAway", &BarrettHandGroupPosition::setBreakAwayCB_, this);
		
		#if TRACE_ACTIVATED
			ROS_INFO("Finish BarrettHandGroupPosition::init !");
		#endif
		
		return true;
	}
	void BarrettHandGroupPosition::checkLimit_(double &angleValue, const Fingers& index) const noexcept
	{
		int limit_index = enum_value(index);
		
		if (angleValue < joint_limits_.min(limit_index))
		{
			angleValue = joint_limits_.min(limit_index);
		}
		
		if (angleValue > joint_limits_.max(limit_index))
		{
			angleValue = joint_limits_.max(limit_index);
		}
	}
	
	void BarrettHandGroupPosition::update(const ros::Time& time, const ros::Duration& period)
	{
		#if TRACE_ACTIVATED
			ROS_INFO("BarrettHandGroupPosition -> (update) Updating of robot %s !",robot_namespace_.c_str());
		#endif
		
		std::vector<double> & commands = *commands_buffer_.readFromRT();
		GraspingMode & currentGrapingMode = *grasping_mode_buffer_.readFromRT();
		
		
		// joint_handles_[0] -> joint_limits_.min(0) -> j11_joint
		// joint_handles_[1] -> joint_limits_.min(1) -> j12_joint
		// joint_handles_[2] -> joint_limits_.min(2) -> j13_joint
		// joint_handles_[3] -> joint_limits_.min(3) -> j21_joint
		// joint_handles_[4] -> joint_limits_.min(4) -> j22_joint
		// joint_handles_[5] -> joint_limits_.min(5) -> j23_joint
		// joint_handles_[6] -> joint_limits_.min(6) -> j32_joint
		// joint_handles_[7] -> joint_limits_.min(7) -> j33_joint
		
		// commands[0] -> j11_joint, mimic j21_joint multiply by 1
		// commands[1] -> j12_joint, mimic j13_joint multiply by 0.344
		// commands[2] -> j22_joint, mimic j23_joint multiply by 0.344
		// commands[3] -> j32_joint, mimic j33_joint multiply by 0.344
		
		
		
		// mimic j11_joint and j21_joint
		checkLimit_(commands[0], Fingers::Finger1_KNUCKLE);
		
		joint_handles_[0].setCommand(commands[0]); // j11_joint
		joint_handles_[3].setCommand(commands[0]); // j21_joint
		
		switch (currentGrapingMode)
		{
			case GraspingMode::Break_Away_Inactive:
			
				// mimic j12_joint and j13_joint
				checkLimit_(commands[1], Fingers::Finger1_PROXIMAL);
				
				joint_handles_[1].setCommand(commands[1]); // j12_joint
				joint_handles_[2].setCommand(commands[1]*0.344); // j13_joint
				
				// mimic j22_joint and j23_joint
				checkLimit_(commands[2], Fingers::Finger2_PROXIMAL);
				
				joint_handles_[4].setCommand(commands[2]); // j22_joint
				joint_handles_[5].setCommand(commands[2]*0.344); // j23_joint
				
				// mimic j32_joint and j33_joint
				checkLimit_(commands[3], Fingers::Finger3_PROXIMAL);
				
				joint_handles_[6].setCommand(commands[3]); // j32_joint
				joint_handles_[7].setCommand(commands[3]*0.344); // j33_joint
				
				break;
				
			case GraspingMode::Break_Away_Active:
			
				// move only j13_joint, j12_joint is blocked
				checkLimit_(commands[1], Fingers::Finger1_DISTAL);
				joint_handles_[1].setCommand(joint_handles_[1].getPosition()); // j12_joint
				joint_handles_[2].setCommand(commands[1]); // j13_joint
				
				// move only j23_joint, j22_joint is blocked
				checkLimit_(commands[2], Fingers::Finger2_DISTAL);
				joint_handles_[4].setCommand(joint_handles_[4].getPosition()); // j22_joint
				joint_handles_[5].setCommand(commands[2]); // j23_joint
				
				// move only j33_joint, j32_joint is blocked
				checkLimit_(commands[3], Fingers::Finger3_DISTAL);
				joint_handles_[6].setCommand(joint_handles_[6].getPosition()); // j32_joint
				joint_handles_[7].setCommand(commands[3]); // j33_joint
				
				break;
		};
		
	}
	
	bool BarrettHandGroupPosition::setBreakAwayCB_(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
	{
		#if TRACE_ACTIVATED
			ROS_INFO("BarrettHandGroupPosition -> (setBreakAwayCB_) Start commandCB of robot %s!  with request data = %d !",robot_namespace_.c_str(),req.data);
		#endif
		
		if (req.data == true)
		{
			res.message = "Mode Break Away Active !";
			grasping_mode_buffer_.writeFromNonRT(GraspingMode::Break_Away_Active);
		}
		else
		{
			res.message = "Mode Break Away Inactive !";
			grasping_mode_buffer_.writeFromNonRT(GraspingMode::Break_Away_Inactive);
		}
		
		res.success = true;
		
		return true;
	}
	
	void BarrettHandGroupPosition::commandCB_(const std_msgs::Float64MultiArrayConstPtr& msg) 
	{
		 #if TRACE_ACTIVATED
			ROS_INFO("BarrettHandGroupPosition -> (commandCB) Start commandCB of robot %s!",robot_namespace_.c_str());
		 #endif
		 
		if (msg->data.size()!=n_dof_)
		{ 
			ROS_ERROR_STREAM("BarrettHandGroupPosition -> (commandCB) Dimension of command (" << msg->data.size() << ") does not match number of degrees of freedom (" << n_dof_ << ")! Not executing!");
			return; 
		}
		
		GraspingMode & currentGrapingMode = *grasping_mode_buffer_.readFromRT();
		
		if (msg->data[0] < joint_limits_.min(0))
		{
			ROS_ERROR_STREAM("BarrettHandGroupPosition -> (commandCB) angle value of index 0 does not match the min limit Finger 1 Knuckle :" << joint_limits_.min(0));
			return;
		}
		
		if (msg->data[0] > joint_limits_.max(0))
		{
			ROS_ERROR_STREAM("BarrettHandGroupPosition -> (commandCB) angle value of index 0 does not match the max limit Finger 1 Knuckle :" << joint_limits_.max(0));
			return;
		}
		
		
		switch (currentGrapingMode)
		{
			case GraspingMode::Break_Away_Inactive:
				if (msg->data[1] < joint_limits_.min(1))
				{
					ROS_ERROR_STREAM("BarrettHandGroupPosition -> (commandCB) angle value of index 1 does not match the min limit Finger 1 Proximal :" << joint_limits_.min(1));
					return;
				}
				
				if (msg->data[1] > joint_limits_.max(1))
				{
					ROS_ERROR_STREAM("BarrettHandGroupPosition -> (commandCB) angle value of index 1 does not match the max limit Finger 1 Proximal :" << joint_limits_.max(1));
					return;
				}
				
				if (msg->data[2] < joint_limits_.min(4))
				{
					ROS_ERROR_STREAM("BarrettHandGroupPosition -> (commandCB) angle value of index 2 does not match the min limit Finger 2 Proximal :" << joint_limits_.min(4));
					return;
				}
				
				if (msg->data[2] > joint_limits_.max(4))
				{
					ROS_ERROR_STREAM("BarrettHandGroupPosition -> (commandCB) angle value of index 2 does not match the max limit Finger 2 Proximal :" << joint_limits_.max(4));
					return;
				}
				
				if (msg->data[3] < joint_limits_.min(6))
				{
					ROS_ERROR_STREAM("BarrettHandGroupPosition -> (commandCB) angle value of index 3 does not match the min limit Finger 3 Proximal :" << joint_limits_.min(6));
					return;
				}
				
				if (msg->data[3] > joint_limits_.max(6))
				{
					ROS_ERROR_STREAM("BarrettHandGroupPosition -> (commandCB) angle value of index 3 does not match the max limit Finger 3 Proximal :" << joint_limits_.max(6));
					return;
				}
			break;
			
			case GraspingMode::Break_Away_Active:
				if (msg->data[1] < joint_limits_.min(2))
				{
					ROS_ERROR_STREAM("BarrettHandGroupPosition -> (commandCB) angle value of index 1 does not match the min limit Finger 1 Distal :" << joint_limits_.min(2));
					return;
				}
				
				if (msg->data[1] > joint_limits_.max(2))
				{
					ROS_ERROR_STREAM("BarrettHandGroupPosition -> (commandCB) angle value of index 1 does not match the max limit Finger 1 Distal :" << joint_limits_.max(2));
					return;
				}
				
				if (msg->data[2] < joint_limits_.min(5))
				{
					ROS_ERROR_STREAM("BarrettHandGroupPosition -> (commandCB) angle value of index 2 does not match the min limit Finger 2 Distal :" << joint_limits_.min(5));
					return;
				}
				
				if (msg->data[2] > joint_limits_.max(5))
				{
					ROS_ERROR_STREAM("BarrettHandGroupPosition -> (commandCB) angle value of index 2 does not match the max limit Finger 2 Distal :" << joint_limits_.max(5));
					return;
				}
				
				if (msg->data[3] < joint_limits_.min(7))
				{
					ROS_ERROR_STREAM("BarrettHandGroupPosition -> (commandCB) angle value of index 3 does not match the min limit Finger 3 Distal :" << joint_limits_.min(7));
					return;
				}
				
				if (msg->data[3] > joint_limits_.max(7))
				{
					ROS_ERROR_STREAM("BarrettHandGroupPosition -> (commandCB) angle value of index 3 does not match the max limit Finger 3 Distal :" << joint_limits_.max(7));
					return;
				}
			
			break;
		}
		
		commands_buffer_.writeFromNonRT(msg->data);
		
		 #if TRACE_ACTIVATED
			ROS_INFO("BarrettHandGroupPosition -> (commandCB) Finish commandCB of robot %s !",robot_namespace_.c_str());
		 #endif
	}
}

PLUGINLIB_EXPORT_CLASS(barrett_hand_controllers::BarrettHandGroupPosition, controller_interface::ControllerBase)
