#include "offboard_control.hpp"

#include <cmath>

#define START_FROM_LAST_MEAS 1

OffboardControl::OffboardControl() : rclcpp::Node("offboard_control"), _state(STOPPED) {

	_offboard_control_mode_publisher =
		this->create_publisher<OffboardControlMode>("fmu/in/offboard_control_mode", 10);
	_trajectory_setpoint_publisher =
		this->create_publisher<TrajectorySetpoint>("fmu/in/trajectory_setpoint", 10);
	_vehicle_command_publisher =
		this->create_publisher<VehicleCommand>("fmu/in/vehicle_command", 10);

	/*Visualization*/
	_path_publisher = this->create_publisher<nav_msgs::msg::Path>("rrt/path", 1);
	_check_path_pub = this->create_publisher<visualization_msgs::msg::Marker>("/leo/drone/check_path", 1);

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

	// get common timestamp
	_timesync_sub =
		this->create_subscription<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", qos,
			[this](const px4_msgs::msg::TimesyncStatus::UniquePtr msg) {
				_timestamp.store(msg->timestamp);
			});

	_odom_sub =
		this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/out/vehicle_odometry", qos,
			[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
				_first_odom = true;
				
				_attitude = matrix::Quaternionf(msg->q.data()[0], msg->q.data()[1], msg->q.data()[2], msg->q.data()[3]);
				_position = matrix::Vector3f(msg->position[0], msg->position[1], msg->position[2]);
				if(isnanf(_position(0)) || 
						isnanf(_position(1)) ||
						isnanf(_position(2))) {
					RCLCPP_WARN(rclcpp::get_logger("OFFBOARD"), "INVALID POSITION: %10.5f, %10.5f, %10.5f",
						_position(0), _position(1), _position(2));
				}
				if(!_first_traj){
					_prev_sp = _position;
					_prev_att_sp = _attitude;
					_prev_yaw_sp = matrix::Eulerf(_attitude).psi();
				}
			});

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
    
	_T_enu_to_ned.setZero();
	_T_enu_to_ned(0,1) =  1.0;
	_T_enu_to_ned(1,0) =  1.0;
	_T_enu_to_ned(2,2) = -1.0;

}

void OffboardControl::octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr octo_msg ) {
	octomap::AbstractOcTree* tect = octomap_msgs::binaryMsgToMap(*octo_msg);
    octomap::OcTree* tree_oct = (octomap::OcTree*)tect;
	_pp->set_octo_tree(tree_oct);
	_map_set = true;
	//RCLCPP_INFO(get_logger(), "Map got");
}

void OffboardControl::timer_callback() {
	if(!_first_odom)
		return;

	if(!_first_traj) {
		// _prev_sp = _position;
		// _prev_att_sp = _attitude;
		// startTraj(_position, matrix::Eulerf(_attitude).psi(), 0.1);
		//_first_traj = true;
		holdTraj();
		_first_traj = true;
		_stop_trajectory = false;
	}

	if (_offboard_setpoint_counter == 10) {
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	}

	_trajectory.getNext(_x, _xd, _xdd);

	if(_stop_trajectory){	
		holdTraj();
		_x.pose.position.x = _stop_sp(0);
		_x.pose.position.y = _stop_sp(1);
		_x.pose.position.z = _stop_sp(2);

		_xd.twist.linear.x = 0.0;
		_xd.twist.linear.y = 0.0;
		_xd.twist.linear.z = 0.0;

		_xdd.accel.linear.x = 0.0;
		_xdd.accel.linear.y = 0.0;
		_xdd.accel.linear.z = 0.0;

	}

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
	
		sp = _T_enu_to_ned*sp;

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
			
			sp = _T_enu_to_ned*sp;

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
		
		
		sp = _T_enu_to_ned*sp;
		yaw_d = matrix::Eulerf(_attitude).psi(); 
		
		yaw_time = std::abs(_prev_yaw_sp - yaw_d)/_max_yaw_rate;
		duration = std::sqrt(pow(sp(0) - _prev_sp(0),2)+pow(sp(1) - _prev_sp(1),2)+pow(sp(2) - _prev_sp(2),2))/(2*_max_velocity);

		this->arm();

		//startTraj(_prev_sp, yaw_d, yaw_time); 
		startTraj(sp, yaw_d, duration);

		_prev_sp = sp;
		_prev_yaw_sp = yaw_d;                 
	}
	else if(cmd == "land") {
		_prev_sp(2) = 0.5; 
		//yaw_d = matrix::Eulerf(_attitude).psi();
		startTraj(_prev_sp, yaw_d, 15);	 // TODO tune time
	}
	else if(cmd == "arm") {
		_first_traj = false;
		this->flight_termination(0);
		this->arm();
	}
	else if(cmd == "term") {
		this->flight_termination(1);		 // TODO unire term a land (check odometry feedback)
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
			
			sp = _T_enu_to_ned*sp;

			yaw_d = atan2(sp(1)-_prev_sp(1),sp(0)-_prev_sp(0)); 
			yaw_d = std::isnan(yaw_d) ? _prev_yaw_sp : yaw_d;
			yaw_time = 0.2 + std::abs(_prev_yaw_sp - yaw_d)/_max_yaw_rate;
			duration = 0.5 + std::sqrt(pow(sp(0) - _prev_sp(0),2)+pow(sp(1) - _prev_sp(1),2)+pow(sp(2) - _prev_sp(2),2))/_max_velocity;
			_replan = false;
			startTraj(_prev_sp, yaw_d, yaw_time); 
			startTraj(sp, yaw_d, duration);

			_prev_sp = sp;
			_prev_yaw_sp = yaw_d;

		}
		else if(cmd=="nav"){
			Eigen::Vector3d wp;
			
			//_wp_traj_completed = true; // Otherwise the last check thread keeps running - the new nav overrides the previous
			std::cout << "Enter X coordinate (ENU frame): "; 
			std::cin >> wp[0];
			std::cout << "Enter Y coordinate (ENU frame): "; 
			std::cin >> wp[1];
			std::cout << "Enter Z coordinate (ENU frame): "; 
			std::cin >> wp[2];

			_replan = true;
			_wp_traj_completed = false;

			auto opt_poses = std::make_shared<std::vector<POSE>>();
			plan(wp,opt_poses);
			
			_replan = false;

			int wp_index = 1;

			auto id_wp_ptr = std::shared_ptr<int>(new int(0));
			//*id_wp_ptr = wp_index;

			std::vector<POSE> poses_to_check = *opt_poses;
			boost::thread check_path_t( &OffboardControl::check_path, this, poses_to_check, id_wp_ptr);
			
			_stop_trajectory = false;

			while(wp_index<int(opt_poses->size()) && _stop_trajectory == false) {	
				
				sp(0) = (*opt_poses)[wp_index].position.x;
				sp(1) = (*opt_poses)[wp_index].position.y;
				sp(2) = (*opt_poses)[wp_index].position.z;    // TODO proper conversion
				
				sp = _T_enu_to_ned*sp;

				yaw_d = atan2(sp(1)-_prev_sp(1),sp(0)-_prev_sp(0)); 
				yaw_d = std::isnan(yaw_d) ? _prev_yaw_sp : yaw_d;
				yaw_time = 0.2 + std::abs(_prev_yaw_sp - yaw_d)/_max_yaw_rate;

				duration = 0.5 + std::sqrt(pow(sp(0) - _prev_sp(0),2)+pow(sp(1) - _prev_sp(1),2)+pow(sp(2) - _prev_sp(2),2))/_max_velocity;
		
				startTraj(_prev_sp, yaw_d, yaw_time);  //Blocking
			 	startTraj(sp, yaw_d, duration);	

			 	// _prev_sp = sp;
			 	// _prev_yaw_sp = yaw_d;
				
				*id_wp_ptr = wp_index;
				wp_index++;
	
			}

		}
		else if(cmd == "takeoff") {
			sp(0) = _position(1);   
			sp(1) = _position(0);
			std::cout << "Enter takeoff altitude (ENU frame): "; 
			std::cin >> sp(2);
			
			sp = _T_enu_to_ned*sp;
			yaw_d = matrix::Eulerf(_attitude).psi(); 
			
			yaw_time = 0.1 + std::abs(_prev_yaw_sp - yaw_d)/_max_yaw_rate;
			duration = 0.0 + std::sqrt(pow(sp(0) - _prev_sp(0),2)+pow(sp(1) - _prev_sp(1),2)+pow(sp(2) - _prev_sp(2),2))/(2*_max_velocity);

			this->arm();

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
			this->arm();
		}
		else if(cmd == "term") {
			this->flight_termination(1);		 // TODO unire term a land (check odometry feedback)
		}
		else if(cmd == "stop") {
			_stop_trajectory = true;
			_first_traj = false;
			_stop_sp = _position;
		}
		else {
			std::cout << "Unknown command;\n";
		}

	}

}

void OffboardControl::holdTraj(){

	//RCLCPP_INFO(this->get_logger(),"Hold Trajectory Set");

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

void OffboardControl::arm() {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::disarm() {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void OffboardControl::flight_termination(float value){
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION, value);

	RCLCPP_INFO(this->get_logger(), "Flight Termination command send");
}

void OffboardControl::publish_offboard_control_mode() {
	OffboardControlMode msg{};
	rclcpp::Time now = this->get_clock()->now();

	msg.timestamp = now.nanoseconds() / 1000.0;

	msg.position = true;
	msg.velocity = true;
	msg.acceleration = true;
	msg.attitude = true;
	msg.body_rate = false;

	_offboard_control_mode_publisher->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint() {
	TrajectorySetpoint msg{};
	rclcpp::Time now = this->get_clock()->now();

	msg.timestamp = now.nanoseconds() / 1000.0;


	msg.position[0] = _x.pose.position.x;
	msg.position[1] = _x.pose.position.y;
	msg.position[2] = _x.pose.position.z;

	msg.velocity[0] = _xd.twist.linear.x;
	msg.velocity[1] = _xd.twist.linear.y;
	msg.velocity[2] = _xd.twist.linear.z;

	msg.acceleration[0] = _xdd.accel.linear.x;
	msg.acceleration[1] = _xdd.accel.linear.y;
	msg.acceleration[2] = _xdd.accel.linear.z;

	matrix::Quaternionf des_att(_x.pose.orientation.w, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z);
	msg.yaw = matrix::Eulerf(des_att).psi();

	if (isnanf(msg.position[0]) || isnanf(msg.position[1]) || isnanf(msg.position[2]))
		RCLCPP_INFO(this->get_logger(),"NAN in trajectory setpoint");

	_trajectory_setpoint_publisher->publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2) {
	VehicleCommand msg{};
	rclcpp::Time now = this->get_clock()->now();

	msg.timestamp = now.nanoseconds() / 1000.0;

	msg.param1 = param1;
	msg.param2 = param2;

	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	_vehicle_command_publisher->publish(msg);
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
		
		sp = _T_enu_to_ned*sp;

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
	
	while(_trajectory.isReady()){ 
		usleep(0.1e6);
		if(_replan) return;	
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
	_prev_yaw_sp = matrix::Eulerf(att).psi();

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
	
	if(START_FROM_LAST_MEAS){
		_prev_sp = _position;
		_prev_att_sp = _attitude;
		_prev_yaw_sp = matrix::Eulerf(_attitude).psi();
	}

	matrix::Vector3f prev_sp = _T_enu_to_ned*_prev_sp; 
	//matrix::Vector3f prev_sp = _T_enu_to_ned*_last_pos_sp;
    s.position.x = prev_sp(0); 
    s.position.y = prev_sp(1);
    s.position.z = prev_sp(2);
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

	usleep(10);

    visualization_msgs::msg::Marker check_m;
	// Set the frame, timestamp, and namespace
	check_m.header.frame_id = "map";
	check_m.header.stamp = this->get_clock()->now();
	check_m.ns = "check";

	// Set marker properties
	check_m.type = visualization_msgs::msg::Marker::CUBE;
	check_m.action = visualization_msgs::msg::Marker::ADD;

	// Set the scale of the marker
	check_m.scale.x = 0.15;
	check_m.scale.y = 0.15;
	check_m.scale.z = 0.15;

	// Set the color of the marker
	check_m.color.r = 1.0f;
	check_m.color.g = 0.0f;
	check_m.color.b = 0.0f;
	check_m.color.a = 1.0;

	// Marker lifetime
	check_m.lifetime = rclcpp::Duration(0, 0); // Infinite lifetime

	// Set marker ID
	check_m.id = 97;

	Eigen::Vector3d pt_check;
	Eigen::Vector3d dir;
	Eigen::Vector3d pt_i;
	Eigen::Vector3d pt_f;
	bool valid_path = true;
	double s[3]; //state
	double step = 0.2;
	bool segment_checked = false;
	bool trajetory_is_completed = false;
	bool is_last_sp = false;

	while( valid_path && !_stop_trajectory && *wp < poses.size() && !_wp_traj_completed && !_replan){	 // continue checking while executing

		if(*wp != 0){
			pt_i << _x.pose.position.y, _x.pose.position.x, -_x.pose.position.z;  // NED TO ENU !!

			for(int i=*wp ; i<poses.size(); i++ ) {
				//RCLCPP_WARN(get_logger(), "Checking wp %f", i);
				pt_f << poses[i].position.x, poses[i].position.y, poses[i].position.z;
				
				dir = (pt_f - pt_i);
				if (dir.norm() > 0.0) dir /= dir.norm();
				pt_check = pt_i;
				segment_checked = false;

    	        while( !segment_checked && valid_path) {

					pt_check += dir*step;
					check_m.pose.position.x = pt_check[0];
					check_m.pose.position.y = pt_check[1];
					check_m.pose.position.z = pt_check[2];
					s[0] = pt_check[0];
					s[1] = pt_check[1];
					s[2] = pt_check[2];

					valid_path = _pp->check_state(s);

					segment_checked = ((pt_check-pt_f).norm() < 2*step);
					if( _rviz_output ) 
						_check_path_pub->publish( check_m );
				}
				
				pt_i = pt_f;

				if(!valid_path) break;
			}
		}
		
		is_last_sp = (!_trajectory.isReady() && *wp == poses.size()-1);
	
		if( is_last_sp ){
			_wp_traj_completed = true;
			RCLCPP_WARN(get_logger(), "Checking path ENDED");
		}
	}
	if( !valid_path ) {
		RCLCPP_WARN(get_logger(), "New obstacle detected! Replan");
		_replan = true;
		_stop_trajectory = true;
		_stop_sp = _position;
		//_first_traj = false;
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
