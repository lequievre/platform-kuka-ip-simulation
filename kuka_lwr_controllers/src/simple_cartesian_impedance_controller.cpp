/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@univ-bpclermont.fr
 * 
*/

#include <simple_cartesian_impedance_controller.h>

// For plugin
#include <pluginlib/class_list_macros.h>

namespace kuka_lwr_controllers 
{
    SimpleCartesianImpedanceController::SimpleCartesianImpedanceController() {}
    SimpleCartesianImpedanceController::~SimpleCartesianImpedanceController() {}
    
     bool SimpleCartesianImpedanceController::init(hardware_interface::KUKACartesianInterface *robot, ros::NodeHandle &n)
     {
		robot_namespace_ = n.getNamespace();
		// robot_namespace_ contains the namespace concatenate with the name of this controller
		// example : /kuka_lwr_left/kuka_simple_cartesian_impedance_controller
		// Only need to get the namespace --------------------------------------------
		int pos_found = robot_namespace_.find("/",1);
		std::string robot_namespace_only = robot_namespace_.substr(1,pos_found-1);
		// ----------------------------------------------------------------------------
		 
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("SimpleCartesianImpedanceController: Start init of robot %s !",robot_namespace_.c_str());
			ROS_INFO("SimpleCartesianImpedanceController: Start init of robot ns only %s !",robot_namespace_only.c_str());
		#endif
		
			
        if( !(KinematicChainControllerBase<hardware_interface::KUKACartesianInterface>::init(robot, n)) )
        {
            ROS_ERROR("SimpleCartesianImpedanceController: Couldn't initilize SimpleCartesianImpedanceController controller of robot %s!",robot_namespace_.c_str());
            return false;
        }
        
        // Get Cartesian Stiffness and Damping Interfaces Handles
        kuka_cart_stiff_handle_ = robot->getHandle(robot_namespace_only + "_cart_stiffness");
        kuka_cart_damp_handle_ = robot->getHandle(robot_namespace_only + "_cart_damping");
        
        // Get Cartesian Pose and Wrench Interfaces Handles
        kuka_cart_pose_handle_ = robot->getHandle(robot_namespace_only + "_cart_pose");
        kuka_cart_wrench_handle_ = robot->getHandle(robot_namespace_only + "_cart_wrench");
        
        sub_cart_stiffness_command_ = n.subscribe("setCartesianStiffness", 1, &SimpleCartesianImpedanceController::setCartesianStiffness, this); 
        sub_cart_damping_command_ = n.subscribe("setCartesianDamping", 1, &SimpleCartesianImpedanceController::setCartesianDamping, this); 
        
        sub_cart_pose_command_ = n.subscribe("setCartesianPose", 1, &SimpleCartesianImpedanceController::setCartesianPose, this); 
        sub_cart_wrench_command_ = n.subscribe("setCartesianWrench", 1, &SimpleCartesianImpedanceController::setCartesianWrench, this); 
        
        realtime_pose_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(n, "cartesianPose", 4));
       
        cur_Pose_FRI_.resize(NUMBER_OF_FRAME_ELEMENTS);
       
        return true;
        
	 }
	 
	 void SimpleCartesianImpedanceController::starting(const ros::Time& time)
     {
        
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("SimpleCartesianImpedanceController: Starting of robot %s !",robot_namespace_.c_str());
		#endif
		
		trace_count_ = 0;
		
		// Get current Pose (Rotation && Translation matrix values)
		getCurrentPose_(pose_cur_);
		
		// Initial Cartesian stiffness
		KDL::Stiffness init_stiffness( 800.0, 800.0, 800.0, 50.0, 50.0, 50.0 );
		stiff_cur_ = init_stiffness;
		
		// Initial Cartesian damping
		KDL::Stiffness init_damping( 0.8, 0.8, 0.8, 0.8, 0.8, 0.8 );
		damp_cur_ = init_damping;
		
		// Initial force/torque wrench
		KDL::Wrench init_wrench(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
		wrench_cur_ = init_wrench;
		
		// forward initial commands to HW
		forwardCmdFRI_(pose_cur_,stiff_cur_,damp_cur_,wrench_cur_);
		
     }
    
     void SimpleCartesianImpedanceController::stopping(const ros::Time& time)
	 {
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO_NAMED("SimpleCartesianImpedanceController: Stopping of robot %s !",robot_namespace_.c_str());
		#endif
		
	 }
	 
	 void SimpleCartesianImpedanceController::update(const ros::Time& time, const ros::Duration& period)
     {
		// Get current Pose (Rotation && Translation matrix values)
		getCurrentPose_(pose_cur_);
		
		// Publish current cartesian pose
		publishCurrentPose(pose_cur_);
        
		// forward initial commands to HW
		forwardCmdFRI_(pose_cur_,stiff_cur_,damp_cur_,wrench_cur_);
		 
		 #if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			 trace_count_++;
			 if (trace_count_%10000)
			 {
				ROS_INFO("-> stiffness -> X = %f, Y = %f, Z = %f, A = %f, B = %f, C = %f", kuka_cart_stiff_handle_.getX(), kuka_cart_stiff_handle_.getY(), kuka_cart_stiff_handle_.getZ(), kuka_cart_stiff_handle_.getA(), kuka_cart_stiff_handle_.getB(), kuka_cart_stiff_handle_.getC());
				/*for (int i=0; i<7; i++)
				{
					ROS_INFO("Joint Pos(%d) = %f", i, joint_handles_[i].getPosition());
				}*/
			 }
		 #endif
	 }
	 
	 
	 
	 void SimpleCartesianImpedanceController::setCartesianPose(const std_msgs::Float64MultiArrayConstPtr& msg)
	 {
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("SimpleCartesianImpedanceController: Start setCartesianPose of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=NUMBER_OF_FRAME_ELEMENTS)
		{ 
			ROS_ERROR_STREAM("SimpleCartesianImpedanceController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of DOF(" << NUMBER_OF_FRAME_ELEMENTS << ")! Not executing!");
			return; 
		}
		
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("Get SimpleCartesianImpedanceController/setCartesianPose :");
			ROS_INFO("[%.3f, %.3f, %.3f, %.3f",msg->data[0],msg->data[1],msg->data[2],msg->data[3]);
			ROS_INFO("%.3f, %.3f, %.3f, %.3f",msg->data[4],msg->data[5],msg->data[6],msg->data[7]);
			ROS_INFO("%.3f, %.3f, %.3f, %.3f",msg->data[8],msg->data[9],msg->data[10],msg->data[11]);
			ROS_INFO("0, 0, 0, 1]");
		#endif
		
		// 0: RXX, 1: RXY, 2: RXZ, 3: TX, 4: RYX, 5: RYY, 6: RYZ, 7: TY, 8: RZX, 9: RZY, 10: RZZ, 11: TZ
		pose_cur_.M = KDL::Rotation(msg->data[0],msg->data[1],msg->data[2],msg->data[4],msg->data[5],msg->data[6],msg->data[8],msg->data[9],msg->data[10]);
		pose_cur_.p = KDL::Vector(msg->data[3],msg->data[7],msg->data[11]);
		
	 }
	 
	 
	 
	  void SimpleCartesianImpedanceController::setCartesianWrench(const std_msgs::Float64MultiArrayConstPtr& msg)
	  {
		  #if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("SimpleCartesianImpedanceController: Start setCartesianWrench of robot %s!",robot_namespace_.c_str());
		  #endif

		 if(msg->data.size()!=NUMBER_OF_CART_DOFS)
		 { 
			ROS_ERROR_STREAM("SimpleCartesianImpedanceController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of DOF(" << NUMBER_OF_CART_DOFS << ")! Not executing!");
			return; 
		 }
		 
		 // 0: X, 1: Y, 2: Z, 3: A, 4: B, 5: C
		 wrench_cur_.force = KDL::Vector(msg->data[0],msg->data[1],msg->data[2]);
		 wrench_cur_.torque = KDL::Vector(msg->data[3],msg->data[4],msg->data[5]);
		  
	  }
	 
	 
	 
	 
	 
	 void SimpleCartesianImpedanceController::setCartesianStiffness(const std_msgs::Float64MultiArrayConstPtr& msg)
	 {
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("SimpleCartesianImpedanceController: Start setCartesianStiffness of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=NUMBER_OF_CART_DOFS)
		{ 
			ROS_ERROR_STREAM("SimpleCartesianImpedanceController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of DOF(" << NUMBER_OF_CART_DOFS << ")! Not executing!");
			return; 
		}
		
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("Get SimpleCartesianImpedanceController/setCartesianStiffness X=%f, Y=%f, Z=%f, A=%f, B=%f, C=%f",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]);
		#endif
		
		// 0: X, 1: Y, 2: Z, 3: A, 4: B, 5: C
		for (int i=0; i<NUMBER_OF_CART_DOFS; i++)
		{
			stiff_cur_[i] = msg->data[i];
		}
		
	 }
	 
	 void SimpleCartesianImpedanceController::setCartesianDamping(const std_msgs::Float64MultiArrayConstPtr& msg)
	 {
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("SimpleCartesianImpedanceController: Start setCartesianDamping of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=NUMBER_OF_CART_DOFS)
		{ 
			ROS_ERROR_STREAM("SimpleCartesianImpedanceController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of DOF(" << NUMBER_OF_CART_DOFS << ")! Not executing!");
			return; 
		}
		
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("Get SimpleCartesianImpedanceController/setCartesianDamping X=%f, Y=%f, Z=%f, A=%f, B=%f, C=%f",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]);
		#endif
		
		// 0: X, 1: Y, 2: Z, 3: A, 4: B, 5: C
		// 0: X, 1: Y, 2: Z, 3: A, 4: B, 5: C
		for (int i=0; i<NUMBER_OF_CART_DOFS; i++)
		{
			damp_cur_[i] = msg->data[i];
		}
		
	 }
	 
	void SimpleCartesianImpedanceController::getCurrentPose_(KDL::Frame& frame)
	{
		 KDL::Rotation cur_R(kuka_cart_pose_handle_.getRXX(),
							kuka_cart_pose_handle_.getRXY(),
							kuka_cart_pose_handle_.getRXZ(),
							kuka_cart_pose_handle_.getRYX(),
							kuka_cart_pose_handle_.getRYY(),
							kuka_cart_pose_handle_.getRYZ(),
							kuka_cart_pose_handle_.getRZX(),
							kuka_cart_pose_handle_.getRZY(),
							kuka_cart_pose_handle_.getRZZ());
							
		KDL::Vector cur_T(kuka_cart_pose_handle_.getTX(),
						  kuka_cart_pose_handle_.getTY(),
						  kuka_cart_pose_handle_.getTZ());
						  
		frame = KDL::Frame(cur_R, cur_T);
	 
	}
	 
	 
	void SimpleCartesianImpedanceController::fromKDLtoFRI_(const KDL::Frame& frame_in, std::vector<double>& vect_Pose_FRI_out)
	{
		assert(vect_Pose_FRI_out.size() == NUMBER_OF_FRAME_ELEMENTS);
		vect_Pose_FRI_out[0] = frame_in.M.UnitX().x(); // RXX
		vect_Pose_FRI_out[1] = frame_in.M.UnitY().x(); // RXY
		vect_Pose_FRI_out[2] = frame_in.M.UnitZ().x(); // RXZ
		vect_Pose_FRI_out[3] = frame_in.p.x(); // TX
		vect_Pose_FRI_out[4] = frame_in.M.UnitX().y(); // RYX
		vect_Pose_FRI_out[5] = frame_in.M.UnitY().y(); // RYY
		vect_Pose_FRI_out[6] = frame_in.M.UnitZ().y(); // RYZ
		vect_Pose_FRI_out[7] = frame_in.p.y(); // TY
		vect_Pose_FRI_out[8] = frame_in.M.UnitX().z(); // RZX
		vect_Pose_FRI_out[9] = frame_in.M.UnitY().z(); // RZY
		vect_Pose_FRI_out[10] = frame_in.M.UnitZ().z(); // RZZ
		vect_Pose_FRI_out[11] = frame_in.p.z(); // TZ

	}
	 
	 
	void SimpleCartesianImpedanceController::forwardCmdFRI_(const KDL::Frame& frame, const KDL::Stiffness& stiffness, const KDL::Stiffness& damping, const KDL::Wrench& wrench)
	{
	 // Transform a KDL frame to a 'FRI Pose vector' (Rotation && Translation)
	 fromKDLtoFRI_(frame, cur_Pose_FRI_);
	 
	 // Set Cartesian Pose command
	 kuka_cart_pose_handle_.setCommandRXX(cur_Pose_FRI_[0]);
	 kuka_cart_pose_handle_.setCommandRXY(cur_Pose_FRI_[1]);
	 kuka_cart_pose_handle_.setCommandRXZ(cur_Pose_FRI_[2]);
	 kuka_cart_pose_handle_.setCommandTX(cur_Pose_FRI_[3]);
	 
	 kuka_cart_pose_handle_.setCommandRYX(cur_Pose_FRI_[4]);
	 kuka_cart_pose_handle_.setCommandRYY(cur_Pose_FRI_[5]);
	 kuka_cart_pose_handle_.setCommandRYZ(cur_Pose_FRI_[6]);
	 kuka_cart_pose_handle_.setCommandTY(cur_Pose_FRI_[7]);
	 
	 kuka_cart_pose_handle_.setCommandRZX(cur_Pose_FRI_[8]);
	 kuka_cart_pose_handle_.setCommandRZY(cur_Pose_FRI_[9]);
	 kuka_cart_pose_handle_.setCommandRZZ(cur_Pose_FRI_[10]);
	 kuka_cart_pose_handle_.setCommandTZ(cur_Pose_FRI_[11]);
	 
	 // Set Cartesian Stiffness command
	 kuka_cart_stiff_handle_.setCommandX(stiffness[0]); 
	 kuka_cart_stiff_handle_.setCommandY(stiffness[1]);
	 kuka_cart_stiff_handle_.setCommandZ(stiffness[2]);
	 kuka_cart_stiff_handle_.setCommandA(stiffness[3]); 
	 kuka_cart_stiff_handle_.setCommandB(stiffness[4]);
	 kuka_cart_stiff_handle_.setCommandC(stiffness[5]);
	 
	 // Set Cartesian Damping command
	 kuka_cart_damp_handle_.setCommandX(damping[0]); 
	 kuka_cart_damp_handle_.setCommandY(damping[1]);
	 kuka_cart_damp_handle_.setCommandZ(damping[2]);
	 kuka_cart_damp_handle_.setCommandA(damping[3]); 
	 kuka_cart_damp_handle_.setCommandB(damping[4]);
	 kuka_cart_damp_handle_.setCommandC(damping[5]);
	 
	 // Set Cartesian Wrench command 
	 kuka_cart_wrench_handle_.setCommandX(wrench.force.x());
	 kuka_cart_wrench_handle_.setCommandY(wrench.force.y());
	 kuka_cart_wrench_handle_.setCommandZ(wrench.force.z());
	 kuka_cart_wrench_handle_.setCommandA(wrench.torque.x());
	 kuka_cart_wrench_handle_.setCommandB(wrench.torque.y());
	 kuka_cart_wrench_handle_.setCommandC(wrench.torque.z());
		 
	}
	
	
	void SimpleCartesianImpedanceController::publishCurrentPose(const KDL::Frame& f)
    {
        if (realtime_pose_pub_->trylock()) 
        {
            realtime_pose_pub_->msg_.header.stamp = ros::Time::now();
            tf::poseKDLToMsg(f, realtime_pose_pub_->msg_.pose);
            realtime_pose_pub_->unlockAndPublish();
        }
	}
	 
}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::SimpleCartesianImpedanceController, controller_interface::ControllerBase)
