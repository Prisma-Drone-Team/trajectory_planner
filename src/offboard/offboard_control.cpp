#include "offboard_control.hpp"

#include <cmath>

#define START_FROM_LAST_MEAS 0

OffboardControl::OffboardControl() : rclcpp::Node("offboard_control"), _state(STOPPED) {


	/*Visualization*/
	_path_publisher = this->create_publisher<nav_msgs::msg::Path>("rrt/path", 10);

	rcl_interfaces::msg::ParameterDescriptor traj_points_descriptor;
	traj_points_descriptor.description = "Trajectory points";
	this->declare_parameter<std::vector<double>>("traj_points", {}, traj_points_descriptor);
	std::vector<double> traj_param;
	_traj_present = this->get_parameter("traj_points", traj_param);
	if(_traj_present) {
		_traj_points = traj_param; //.as_double_array();
		if(_traj_points.size() % 5 != 0)
			_traj_present = false;
	}


	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	_command_client = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

	_setpoint_pub = this->create_publisher<mavros_msgs::msg::PositionTarget>( "/mavros/setpoint_raw/local", 10);

	_local_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>( "/mavros/local_position/pose", qos, std::bind(&OffboardControl::mavros_pose_cb, this, std::placeholders::_1));


	_cmd_sub = this->create_subscription<trajectory_planner::msg::MoveCmd>("move_cmd", qos, std::bind(&OffboardControl::move_command_callback, this, std::placeholders::_1));

	_octo_sub = this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_binary", qos, std::bind(&OffboardControl::octomap_callback, this, std::placeholders::_1));
	_map_set = false;	

	_offboard_setpoint_counter = 0;


	_timer = this->create_wall_timer(_timer_period, std::bind(&OffboardControl::timer_callback, this));

	boost::thread key_input_t( &OffboardControl::key_input, this );

	//---Init planner
	this->declare_parameter("max_yaw_rate", .5);
	_max_yaw_rate = this->get_parameter("max_yaw_rate").as_double();
	RCLCPP_INFO(get_logger(), "max_yaw_rate: %f", _max_yaw_rate);
	this->declare_parameter("max_velocity", .25);
	_max_velocity = this->get_parameter("max_velocity").as_double();
	RCLCPP_INFO(get_logger(), "max_velocity: %f", _max_velocity);
	
	this->declare_parameter("robot_radius",0.8);
	_robot_radius = this->get_parameter("robot_radius").as_double();
	RCLCPP_INFO(get_logger(), "robot_radius: %f", _robot_radius);

	this->declare_parameter("x_lower_bound", -5.0);
	_xbounds[0] = this->get_parameter("x_lower_bound").as_double();
	RCLCPP_INFO(get_logger(), "x_lower_bound: %f", _xbounds[0]);
	
	this->declare_parameter("x_upper_bound",21.0);
	_xbounds[1] = this->get_parameter("x_upper_bound").as_double();
	RCLCPP_INFO(get_logger(), "x_upper_bound: %f", _xbounds[1] );

	this->declare_parameter("y_lower_bound", -5.0);
	_ybounds[0] = this->get_parameter("y_lower_bound").as_double();
	RCLCPP_INFO(get_logger(), "y_lower_bound: %f", _ybounds[0]);
	
	this->declare_parameter("y_upper_bound",21.0);
	_ybounds[1] = this->get_parameter("y_upper_bound").as_double();
	RCLCPP_INFO(get_logger(), "y_upper_bound: %f", _ybounds[1] );

	this->declare_parameter("z_lower_bound", -5.0);
	_zbounds[0] = this->get_parameter("z_lower_bound").as_double();
	RCLCPP_INFO(get_logger(), "z_lower_bound: %f", _ybounds[0]);
	
	this->declare_parameter("z_upper_bound",21.0);
	_zbounds[1] = this->get_parameter("z_upper_bound").as_double();
	RCLCPP_INFO(get_logger(), "z_upper_bound: %f", _ybounds[1] );

	this->declare_parameter("x_valid_min",-10.0);
	_x_valid_min = this->get_parameter("x_valid_min").as_double();
	RCLCPP_INFO(get_logger(), "x_valid_min: %f", _x_valid_min);

	this->declare_parameter("y_valid_min",-10.0);
	_y_valid_min = this->get_parameter("y_valid_min").as_double();
	RCLCPP_INFO(get_logger(), "y_valid_min: %f", _y_valid_min);

	this->declare_parameter("x_valid_max",10.0);
	_x_valid_max = this->get_parameter("x_valid_max").as_double();
	RCLCPP_INFO(get_logger(), "x_valid_max: %f", _x_valid_max);

	this->declare_parameter("y_valid_max",10.0);
	_y_valid_max = this->get_parameter("y_valid_max").as_double();
	RCLCPP_INFO(get_logger(), "y_valid_max: %f", _y_valid_max);

	this->declare_parameter("z_motion_threshold",1.0);
	_z_motion_threshold = this->get_parameter("z_motion_threshold").as_double();
	RCLCPP_INFO(get_logger(), "z_motion_threshold: %f", _z_motion_threshold);

	this->declare_parameter("use_octomap",1.0);
	_use_octomap = this->get_parameter("use_octomap").as_double(); // as_bool not working
	RCLCPP_INFO(get_logger(), "use_octomap: %f", _use_octomap);

	this->declare_parameter("rviz_output",1.0);
	_rviz_output = this->get_parameter("rviz_output").as_double(); // as_bool not working
	RCLCPP_INFO(get_logger(), "rviz_output: %f", _rviz_output);	

    _pp = new PATH_PLANNER();
    _pp->init( _xbounds, _ybounds, _zbounds);
    _pp->set_robot_geometry(_robot_radius);


}

void OffboardControl::mavros_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg){

	_first_pose = true;
				
	_attitude = matrix::Quaternionf(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
	_position = matrix::Vector3f(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

	if(isnanf(_position(0)) || 
			isnanf(_position(1)) ||
			isnanf(_position(2))) {
		RCLCPP_WARN(rclcpp::get_logger("OFFBOARD"), "INVALID POSITION: %10.5f, %10.5f, %10.5f",
			_position(0), _position(1), _position(2));
	}
	if(!_first_traj){
		_prev_sp = _position;
		_prev_yaw_sp = matrix::Eulerf(_attitude).psi();
	}

}
void OffboardControl::octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr octo_msg ) {
	octomap::AbstractOcTree* tect = octomap_msgs::binaryMsgToMap(*octo_msg);
    octomap::OcTree* tree_oct = (octomap::OcTree*)tect;
	_pp->set_octo_tree(tree_oct);
	_map_set = true;
	RCLCPP_INFO(get_logger(), "Map got");
}

void OffboardControl::timer_callback() {
	if(!_first_pose)
		return;

	if(!_first_traj){
		// _prev_sp = _position;
		// _prev_att_sp = _attitude;
		// startTraj(_position, matrix::Eulerf(_attitude).psi(), 0.1);
		//_first_traj = true;
		holdTraj();
		_first_traj = true;
	}

	if (_offboard_setpoint_counter == 10) {
		publish_offboard_control_mode();
	}


	_trajectory.getNext(_x, _xd, _xdd);
	
	// offboard_control_mode needs to be paired with trajectory_setpoint
	publish_offboard_control_mode();
	publish_trajectory_setpoint();

	// stop the counter after reaching 101
	if (_offboard_setpoint_counter < 11) {
		_offboard_setpoint_counter++;
	}

}

void OffboardControl::move_command_callback(const trajectory_planner::msg::MoveCmd::SharedPtr msg) {

	std::string cmd = msg->command.data;
	matrix::Vector3f sp;
	sp(0) = msg->pose.position.x;
	sp(1) = msg->pose.position.y;
	sp(2) = msg->pose.position.z;	

	double duration;
	double yaw_time;
	float  yaw_d;

	if(cmd == "go") {
	
		yaw_d = atan2(sp(1)-_prev_sp(1),sp(0)-_prev_sp(0)); 
		
		yaw_d = std::isnan(yaw_d) ? _prev_yaw_sp : yaw_d;

		yaw_time = 0.1 + std::abs(_prev_yaw_sp - yaw_d)/_max_yaw_rate;
		duration = 0.5 + std::sqrt(pow(sp(0) - _prev_sp(0),2)+pow(sp(1) - _prev_sp(1),2)+pow(sp(2) - _prev_sp(2),2))/_max_velocity;

		startTraj(_prev_sp, yaw_d, yaw_time); 
		startTraj(sp, yaw_d, duration);

		_prev_sp = sp;
		_prev_yaw_sp = yaw_d;

	}
	else if(cmd=="nav"){
		Eigen::Vector3d wp;
		wp[0] = sp(0);
		wp[1] = sp(1);
		wp[2] = sp(2);

		auto opt_poses = std::make_shared<std::vector<POSE>>();
		plan(wp,opt_poses);

		// auto id_wp_ptr = std::shared_ptr<int>(new int(0));
		// auto id_wp = 0;
		// boost::thread check_path_t( &OffboardControl::check_path, this, opt_poses, id_wp_ptr);
	
		for(int i = 0; i<int(opt_poses->size()); i++) {
			sp(0) = (*opt_poses)[i].position.x;
			sp(1) = (*opt_poses)[i].position.y;
			sp(2) = (*opt_poses)[i].position.z; 


			yaw_d = atan2(sp(1)-_prev_sp(1),sp(0)-_prev_sp(0)); 
			yaw_d = std::isnan(yaw_d) ? _prev_yaw_sp : yaw_d;
			yaw_time = 0.1 + std::abs(_prev_yaw_sp - yaw_d)/_max_yaw_rate;
			duration = 0.5 + std::sqrt(pow(sp(0) - _prev_sp(0),2)+pow(sp(1) - _prev_sp(1),2)+pow(sp(2) - _prev_sp(2),2))/_max_velocity;
			

			startTraj(_prev_sp, yaw_d, yaw_time); 
			startTraj(sp, yaw_d, duration);

			_prev_sp = sp;
			_prev_yaw_sp = yaw_d;

			// ROS_INFO("")
            //if(  (wp-_cmd_p).norm()  < 0.1 ) 
			// id_wp++;
            // *id_wp_ptr = id_wp;
		}

	}
	else if(cmd == "takeoff") {
		sp(0) = _position(1);   
		sp(1) = _position(0);
		
		yaw_d = matrix::Eulerf(_attitude).psi(); 
		
		yaw_time = std::abs(_prev_yaw_sp - yaw_d)/_max_yaw_rate;
		duration = std::sqrt(pow(sp(0) - _prev_sp(0),2)+pow(sp(1) - _prev_sp(1),2)+pow(sp(2) - _prev_sp(2),2))/(2*_max_velocity);

		this->arm(true);

		//startTraj(_prev_sp, yaw_d, yaw_time); 
		startTraj(sp, yaw_d, duration);

		_prev_sp = sp;
		_prev_yaw_sp = yaw_d;                 
	}
	else if(cmd == "land") {
		_prev_sp(2) = -0.5; 
		//yaw_d = matrix::Eulerf(_attitude).psi();
		startTraj(_prev_sp, yaw_d, 15);	 // TODO tune time
	}
	else if(cmd == "arm") {
		_first_traj = false;
		this->flight_termination(false);
		this->arm(true);
	}
	else if(cmd == "term") {
		this->flight_termination(true);		 // TODO unire term a land (check odometry feedback)
	}

}

void OffboardControl::key_input() {
	bool exit = false;
	std::string cmd;
	matrix::Vector3f sp;
	double duration;
	double yaw_time;
	float  yaw_d;

	while(!exit && rclcpp::ok()) {
		std::cout << "Enter command [arm | takeoff | go | nav | land | term | stop]: \n"; 
		std::cin >> cmd;

		if(cmd == "go") {
			std::cout << "Enter X coordinate (ENU frame): "; 
			std::cin >> sp(0);
			std::cout << "Enter Y coordinate (ENU frame): "; 
			std::cin >> sp(1);
			std::cout << "Enter Z coordinate (ENU frame): "; 
			std::cin >> sp(2);


			yaw_d = atan2(sp(1)-_prev_sp(1),sp(0)-_prev_sp(0)); 
			yaw_d = std::isnan(yaw_d) ? _prev_yaw_sp : yaw_d;
			yaw_time = 0.2 + std::abs(_prev_yaw_sp - yaw_d)/_max_yaw_rate;
			duration = 0.5 + std::sqrt(pow(sp(0) - _prev_sp(0),2)+pow(sp(1) - _prev_sp(1),2)+pow(sp(2) - _prev_sp(2),2))/_max_velocity;
			
			startTraj(_prev_sp, yaw_d, yaw_time); 
			startTraj(sp, yaw_d, duration);

			_prev_sp = sp;
			_prev_yaw_sp = yaw_d;

		}
		else if(cmd=="nav"){
			Eigen::Vector3d wp;
	
			std::cout << "Enter X coordinate (ENU frame): "; 
			std::cin >> wp[0];
			std::cout << "Enter Y coordinate (ENU frame): "; 
			std::cin >> wp[1];
			std::cout << "Enter Z coordinate (ENU frame): "; 
			std::cin >> wp[2];
			
			auto opt_poses = std::make_shared<std::vector<POSE>>();
			plan(wp,opt_poses);

			int wp_index = 1;

			auto id_wp_ptr = std::shared_ptr<int>(new int(0));

			std::vector<POSE> poses_to_check = *opt_poses;
			boost::thread check_path_t( &OffboardControl::check_path, this, poses_to_check, id_wp_ptr);

			while(wp_index<int(opt_poses->size()) && _plan_is_valid ){	
				
				sp(0) = (*opt_poses)[wp_index].position.x;
				sp(1) = (*opt_poses)[wp_index].position.y;
				sp(2) = (*opt_poses)[wp_index].position.z;    // TODO proper conversion

				yaw_d = atan2(sp(1)-_prev_sp(1),sp(0)-_prev_sp(0)); 
				yaw_d = std::isnan(yaw_d) ? _prev_yaw_sp : yaw_d;
				yaw_time = 0.2 + std::abs(_prev_yaw_sp - yaw_d)/_max_yaw_rate;

				duration = 0.5 + std::sqrt(pow(sp(0) - _prev_sp(0),2)+pow(sp(1) - _prev_sp(1),2)+pow(sp(2) - _prev_sp(2),2))/_max_velocity;

		
				startTraj(_prev_sp, yaw_d, yaw_time); 
			 	startTraj(sp, yaw_d, duration);	

			 	_prev_sp = sp;
			 	_prev_yaw_sp = yaw_d;

				wp_index++;
				*id_wp_ptr = wp_index;
			}

			//startWPTraj(opt_poses);


		}
		else if(cmd == "takeoff") {
			sp(0) = _position(1);   
			sp(1) = _position(0);
			std::cout << "Enter takeoff altitude (ENU frame): "; 
			std::cin >> sp(2);
			

			yaw_d = matrix::Eulerf(_attitude).psi(); 
			
			yaw_time = 0.1 + std::abs(_prev_yaw_sp - yaw_d)/_max_yaw_rate;
			duration = 0.1 + std::sqrt(pow(sp(0) - _prev_sp(0),2)+pow(sp(1) - _prev_sp(1),2)+pow(sp(2) - _prev_sp(2),2))/(_max_velocity);

			this->arm(1);

			//startTraj(_prev_sp, yaw_d, yaw_time); 
			startTraj(sp, yaw_d, duration);

			_prev_sp = sp;
			_prev_yaw_sp = yaw_d;    
		
		}
		else if(cmd == "land") {
			std::cout << "Landing procedure triggered... \nRemember to kill disarm manually after landed.\n";
			_prev_sp(2) = 0.5; 
			startTraj(_prev_sp, yaw_d, 15);	 // TODO tune time
		}
		else if(cmd == "arm") {
			_first_traj = false;
			this->flight_termination(0);
			this->arm(1);
		}
		else if(cmd == "term") {
			this->flight_termination(1);		 // TODO unire term a land (check odometry feedback)
		}
		else if(cmd == "stop") {
			_stop_trajectory = true;
			_first_traj = false;
		}
		else {
			std::cout << "Unknown command;\n";
		}

	}

}

void OffboardControl::holdTraj() {
	std::vector<geometry_msgs::msg::PoseStamped> poses;
	std::vector<double> times;
	geometry_msgs::msg::PoseStamped p;
	double t;

	_prev_sp = _position;
	_prev_att_sp = _attitude;

	p.pose.position.x = _prev_sp(0);
	p.pose.position.y = _prev_sp(1);
	p.pose.position.z = _prev_sp(2); 

	p.pose.orientation.w = _prev_att_sp(0);
	p.pose.orientation.x = _prev_att_sp(1);
	p.pose.orientation.y = _prev_att_sp(2);
	p.pose.orientation.z = _prev_att_sp(3);
	t = 0.0f;

	poses.push_back(p);
	times.push_back(t);
	
	poses.push_back(p);
	times.push_back(0.1);

	_trajectory.set_waypoints(poses, times);
	_trajectory.compute();

}
void OffboardControl::send_command(const std::shared_ptr<mavros_msgs::srv::CommandLong::Request> &request, const std::string &command_name){
	// Wait for the service to be available
	// while (!_command_client->wait_for_service(std::chrono::seconds(1)))
	// {
	// 	RCLCPP_WARN(this->get_logger(), "Waiting for CommandLong service to be available...");
	// }

	// Call the service
	auto result = _command_client->async_send_request(request);
	// result.wait();

	// // Check if the command was successful
	// if (result.get()->success)
	// {
	// 	RCLCPP_INFO(this->get_logger(), "%s command sent successfully!", command_name.c_str());
	// }
	// else
	// {
	// 	RCLCPP_ERROR(this->get_logger(), "Failed to send %s command.", command_name.c_str());
	// }
}
void OffboardControl::arm(bool arm){

	auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
	request->command = 400;  // MAV_CMD_COMPONENT_ARM_DISARM
	request->param1 = arm ? 1 : 0;  // 1 to arm, 0 to disarm
	request->broadcast = false;

	// Send the command
	RCLCPP_INFO(this->get_logger(), "Arm command send");
	this->send_command(request, arm ? "Arm" : "Disarm");
;
}

void OffboardControl::flight_termination(bool terminate){
	auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
	request->command = 185;  // MAV_CMD_DO_FLIGHTTERMINATION
	request->param1 = terminate ? 1 : 0;  // 1 to terminate, 0 to cancel
	request->broadcast = false;
	this->send_command(request, terminate ? "Flight Termination: Active" : "Flight Termination: Unactive");

	RCLCPP_INFO(this->get_logger(), "Flight Termination command send");
}

void OffboardControl::publish_offboard_control_mode(){
	auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
	request->command = 176;  // MAV_CMD_DO_SET_MODE
	request->param1 = 1;     // Base mode (custom)
	request->param2 = 6;     // Custom mode (6 = Offboard mode)
	request->broadcast = false;

	// Send the command
	this->send_command(request, "Offboard Mode");
}

void OffboardControl::publish_trajectory_setpoint(){

	auto setpoint_msg = mavros_msgs::msg::PositionTarget();

	// Set the coordinate frame 
	setpoint_msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
	setpoint_msg.header.stamp = this->get_clock()->now();

	// Remove the type mask for the fields we want to publish (e.g., position, velocity, acceleration, and yaw)
	// setpoint_msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
	// 							mavros_msgs::msg::PositionTarget::IGNORE_PY |
	// 							mavros_msgs::msg::PositionTarget::IGNORE_PZ |
	// 							mavros_msgs::msg::PositionTarget::IGNORE_VX |
	// 							mavros_msgs::msg::PositionTarget::IGNORE_VY |
	// 							mavros_msgs::msg::PositionTarget::IGNORE_VZ |
	// 							mavros_msgs::msg::PositionTarget::IGNORE_AFX |
	// 							mavros_msgs::msg::PositionTarget::IGNORE_AFY |
	// 							mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
	// 							mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

	setpoint_msg.position.x = _x.pose.position.x;
	setpoint_msg.position.y = _x.pose.position.y;
	setpoint_msg.position.z = _x.pose.position.z;

	setpoint_msg.velocity.x = _xd.twist.linear.x;  
	setpoint_msg.velocity.y = _xd.twist.linear.y;  
	setpoint_msg.velocity.z = _xd.twist.linear.z; 

	setpoint_msg.acceleration_or_force.x = _xdd.accel.linear.x;
	setpoint_msg.acceleration_or_force.y = _xdd.accel.linear.y; 
	setpoint_msg.acceleration_or_force.z = _xdd.accel.linear.z; 

	matrix::Quaternionf des_att(_x.pose.orientation.w, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z);
	setpoint_msg.yaw = matrix::Eulerf(des_att).psi();;  
	setpoint_msg.yaw_rate = 0.0f;
	
	_setpoint_pub->publish(setpoint_msg);
}

void OffboardControl::startWPTraj(std::shared_ptr<std::vector<POSE>> opt_poses) {

	matrix::Vector3f sp;
	double duration;
	double yaw_time;
	float yaw_d;

	std::vector<geometry_msgs::msg::PoseStamped> poses;
	std::vector<double> times;
	geometry_msgs::msg::PoseStamped p;
	double t;

	for(int i = 0; i<int(opt_poses->size()); i++) {
		sp(0) = (*opt_poses)[i].position.x;
		sp(1) = (*opt_poses)[i].position.y;
		sp(2) = (*opt_poses)[i].position.z; 

		yaw_d = atan2(sp(1)-_prev_sp(1),sp(0)-_prev_sp(0)); 
		yaw_d = std::isnan(yaw_d) ? _prev_yaw_sp : yaw_d;
		matrix::Quaternionf att(matrix::Eulerf(0, 0, yaw_d));

		yaw_time = 0.1 + std::abs(_prev_yaw_sp - yaw_d)/_max_yaw_rate;
		duration = 0.5 + std::sqrt(pow(sp(0) - _prev_sp(0),2)+pow(sp(1) - _prev_sp(1),2)+pow(sp(2) - _prev_sp(2),2))/_max_velocity;

		p.pose.position.x = _prev_sp(0);
		p.pose.position.y = _prev_sp(1);
		p.pose.position.z = _prev_sp(2); 

		p.pose.orientation.w = _prev_att_sp(0);
		p.pose.orientation.x = _prev_att_sp(1);
		p.pose.orientation.y = _prev_att_sp(2);
		p.pose.orientation.z = _prev_att_sp(3);

		t = 0.0f;
		
		poses.push_back(p);
		times.push_back(t);
		
		/* */
		p.pose.position.x = sp(0);
		p.pose.position.y = sp(1);
		p.pose.position.z = sp(2); 

		p.pose.orientation.w = att(0);
		p.pose.orientation.x = att(1);
		p.pose.orientation.y = att(2);
		p.pose.orientation.z = att(3);


		_prev_sp = sp;
		_prev_att_sp = att;

		poses.push_back(p);
		times.push_back(duration);

	}
	_trajectory.set_waypoints(poses, times);

	_trajectory.compute();
}

void OffboardControl::startTraj(matrix::Vector3f pos, float yaw, double d) {

	while(_trajectory.isReady()) {
		usleep(0.1e6);
	}
	
	std::vector<geometry_msgs::msg::PoseStamped> poses;
	std::vector<double> times;
	geometry_msgs::msg::PoseStamped p;
	double t;

	matrix::Quaternionf att(matrix::Eulerf(0, 0, yaw));

	if(START_FROM_LAST_MEAS){
		_prev_sp = _position;
		_prev_att_sp = _attitude;
		_prev_yaw_sp = matrix::Eulerf(_attitude).psi();
	}


	/* */
	p.pose.position.x = _prev_sp(0);
	p.pose.position.y = _prev_sp(1);
	p.pose.position.z = _prev_sp(2); 

	p.pose.orientation.w = _prev_att_sp(0);
	p.pose.orientation.x = _prev_att_sp(1);
	p.pose.orientation.y = _prev_att_sp(2);
	p.pose.orientation.z = _prev_att_sp(3);

	t = 0.0f;
	
	poses.push_back(p);
	times.push_back(t);
	
	/* */
	p.pose.position.x = pos(0);
	p.pose.position.y = pos(1);
	p.pose.position.z = pos(2); 

	p.pose.orientation.w = att(0);
	p.pose.orientation.x = att(1);
	p.pose.orientation.y = att(2);
	p.pose.orientation.z = att(3);


	_prev_sp = pos;
	_prev_att_sp = att;

	poses.push_back(p);
	times.push_back(d);


	_trajectory.set_waypoints(poses, times);

	_trajectory.compute();

}

void OffboardControl::plan(Eigen::Vector3d wp, std::shared_ptr<std::vector<POSE>> opt_poses) {

    if( _use_octomap == 1.0) {
        while(!_map_set) usleep(0.1*1e6);
    }

    POSE s;
    POSE g;
    

    s.position.x = _prev_sp(0); 
    s.position.y = _prev_sp(1);
    s.position.z = _prev_sp(2);
    s.orientation.w = 1.0; // _last_att_sp(0); 
    s.orientation.x = 0.0; // _last_att_sp(1);
    s.orientation.y = 0.0; // _last_att_sp(2);
    s.orientation.z = 0.0; // _last_att_sp(3);

    g.position.x = wp[0];
    g.position.y = wp[1];
    g.position.z = wp[2];
    g.orientation.w = 1.0;
    g.orientation.x = 0.0;
    g.orientation.y = 0.0;
    g.orientation.z = 0.0;

	geometry_msgs::msg::PoseStamped p;
	nav_msgs::msg::Path generated_path;
	generated_path.header.frame_id = "map";
	
    // geometry_msgs::PoseStamped p;
    // p.header.frame_id = "drone/map";

    // Eigen::Vector3d lp;
    // Eigen::Vector3d gp;

    // nav_msgs::Path generated_path_opt;
    // generated_path_opt.header.frame_id = "drone/map";

    _pp->set_start_state(s);
    _pp->set_goal_state(g);


    std::vector<POSE> nav_poses;
    //std::vector<POSE> opt_poses;


    double xbounds[2];
    double ybounds[2];
    double zbounds[2];
    double bz_min = 0.0; //( _w_p[2] < wp[2] ) ?   
    double bz_max = 0.0; 

	Eigen::Vector3f _w_p;
	_w_p << _position(0), _position(1), _position(2);

    if (  _w_p[2] < wp[2]  ) {
        bz_min = _w_p[2];
        bz_max = wp[2];
    }
    else {
        bz_min = wp[2];
        bz_max = _w_p[2];
    }

    xbounds[0] = _x_valid_min;
    ybounds[0] = _y_valid_min;
    zbounds[0] = bz_min - _z_motion_threshold; 

    xbounds[1] = _x_valid_max;
    ybounds[1] = _y_valid_max;
    zbounds[1] = bz_max + _z_motion_threshold; 

    int ret = _pp->plan(2, xbounds, ybounds, zbounds, nav_poses, *opt_poses);

	if( ret == -3 ) {
        RCLCPP_ERROR(get_logger(),"Goal state not valid!");
    }
    else {
		if( ret < 0 || nav_poses.size() < 2 ) 
				RCLCPP_ERROR(get_logger(),"Planner not correctly initialized"); 
		else {

			for( int i=0; i<nav_poses.size(); i++ ) {
				p.pose.position.x = nav_poses[i].position.x;
				p.pose.position.y = nav_poses[i].position.y;
				p.pose.position.z = nav_poses[i].position.z;

				p.pose.orientation.x = nav_poses[i].orientation.x;
				p.pose.orientation.y = nav_poses[i].orientation.y;
				p.pose.orientation.z = nav_poses[i].orientation.y;
				p.pose.orientation.w = nav_poses[i].orientation.z;

				generated_path.poses.push_back( p );
				std::cout<<i<<": p("<<p.pose.position.x<<", "<<p.pose.position.y<<", "<<p.pose.position.z<<")\n";

			}
			std::cout<<"Publish path\n";
			_path_publisher->publish( generated_path );

			std::cout << "Solution: " << std::endl;
			
			for(int i=0; i<nav_poses.size(); i++ ) {
				std::cout << "Pose: [" << i << "]: " << "(" << nav_poses[i].position.x << " " << nav_poses[i].position.y << " " << nav_poses[i].position.z << ")" << std::endl;
			}
		}
	}
}

void OffboardControl::check_path(const std::vector<POSE> & poses, const std::shared_ptr<int> wp) {
    
    // Eigen::Vector3d dir;
    // Eigen::Vector3d v0;
    // Eigen::Vector3d v1;
    // Eigen::Vector3d pt_check;
    // dir << 0.0, 0.0, 0.0;
    // bool valid_path = true;
    // double s[3]; //state
    
    // for(int k=0; k<poses.size(); k++) {
    //     std::cout<<"\t"<<poses[k].position.x<<"---"<<poses[k].position.y<<"---"<<poses[k].position.z<<std::endl;
	// 	if( k == 0 ) v0 = _w_p;
	// 	else v0 <<  poses[k-1].position.x,  poses[k-1].position.y,  poses[k-1].position.z;

	// 	v1 << poses[k].position.x, poses[k].position.y, poses[k].position.z;

	// 	dir = v1 - v0;
		
	// 	dir /= dir.norm();
	// 	std::cout<<"\t\t"<<dir<<std::endl;
    // }
    
    // while( valid_path && (!_stop_trajectory && _trajectory_execution && *wp < poses.size()) ) {
    
    //     for( int i=*wp; i<poses.size(); i++ ) {

    //         while(*wp == 0){
    //             usleep(0.1e6);     
    //         }
                   
    //         if( i == *wp ){
    //             v0 = _w_p;
    //             v1 << poses[i].position.x, poses[i].position.y, poses[i].position.z;
    //         } 
    //         else{
    //             v0 <<  poses[i-1].position.x,  poses[i-1].position.y,  poses[i-1].position.z;
    //             v1 << poses[i].position.x, poses[i].position.y, poses[i].position.z;
    //         } 

    //         dir = v1 - v0;
            
    //         dir /= dir.norm();
    //         pt_check = v0;

    //         bool segment_checked = false;

    //         double step = 0.2; //a param?

    //         while( !segment_checked && valid_path) {

    //             pt_check += dir*step;
    //             check_m.pose.position.x = pt_check[0];
    //             check_m.pose.position.y = pt_check[1];
    //             check_m.pose.position.z = pt_check[2];

    //             s[0] = pt_check[0];
    //             s[1] = pt_check[1];
    //             s[2] = pt_check[2];

    //             segment_checked = ((pt_check-v1).norm() < 0.1);
    //             valid_path = _pp->check_state(s);
    //         }
    //     }   
       
    // }

    // if( !valid_path ) {
	// 	RCLCPP_WARN(rclcpp::get_logger("OFFBOARD"), "New obstacle detected! Replan");
    //     _replan = true;
    // }

	Eigen::Vector3d pt_check;
	Eigen::Vector3d dir;
	Eigen::Vector3d pt_i;
	Eigen::Vector3d pt_f;
	bool valid_path = true;
	double s[3]; //state
	double step = 0.2;
	bool segment_checked = false;

	while( valid_path && !_stop_trajectory && *wp < poses.size()){

		if(*wp != 0){

			pt_i << _position(0), _position(1), _position(2);

			for(int i=*wp + 1; i<poses.size(); i++ ) {

				pt_f << poses[i].position.x, poses[i].position.y, poses[i].position.z;
				
				dir = (pt_f - pt_i);
				if (dir.norm() > 0.0) dir /= dir.norm();
				segment_checked = false;

    	        while( !segment_checked && valid_path) {

					pt_check += dir*step;

					s[0] = pt_check[0];
					s[1] = pt_check[1];
					s[2] = pt_check[2];

					valid_path = _pp->check_state(s);

					segment_checked = ((pt_check-pt_f).norm() < step);

				}
				pt_i = pt_f;

				if(!valid_path) break;
			}
		}
	}
	if( !valid_path ) {
		RCLCPP_WARN(get_logger(), "New obstacle detected! Replan");
		_replan = true;
		_first_traj = false;
	}
}   


int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	auto offboardCtrlPtr = std::make_shared<OffboardControl>();
	
	rclcpp::spin(offboardCtrlPtr);

	rclcpp::shutdown();
	return 0;
}
