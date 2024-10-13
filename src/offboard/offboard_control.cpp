#include "offboard_control.hpp"
#include <random>

#include <cmath>

#define START_FROM_LAST_MEAS 0
#define SIMULATION 1


OffboardControl::OffboardControl() : rclcpp::Node("offboard_control"), _state(STOPPED){

	_offboard_control_mode_publisher =
		this->create_publisher<OffboardControlMode>("fmu/in/offboard_control_mode", 10);
	_trajectory_setpoint_publisher =
		this->create_publisher<TrajectorySetpoint>("fmu/in/trajectory_setpoint", 10);
	_vehicle_command_publisher =
		this->create_publisher<VehicleCommand>("fmu/in/vehicle_command", 10);

	/*Visualization*/
	_path_publisher = this->create_publisher<nav_msgs::msg::Path>("rrt/path", 1);
	_check_path_pub = this->create_publisher<visualization_msgs::msg::Marker>("/leo/drone/check_path", 1);

	// Set up a reliable QoS profile
	// rclcpp::QoS qos_profile(10);
	// qos_profile.reliable(); // Set reliability to reliable
	// qos_profile.keep_last(1); // Keep only the last message
	_plan_state_publisher = this->create_publisher<std_msgs::msg::String>("/leo/drone/plan_status", 1);


	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	// // get common timestamp
	// _timesync_sub =
	// 	this->create_subscription<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", qos,
	// 		[this](const px4_msgs::msg::TimesyncStatus::UniquePtr msg) {
	// 			_timestamp.store(msg->timestamp);
	// 		});

	_odom_sub =
		this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/out/vehicle_odometry", qos,
			[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
				
				
				_attitude = matrix::Quaternionf(msg->q.data()[0], msg->q.data()[1], msg->q.data()[2], msg->q.data()[3]);
				_position = matrix::Vector3f(msg->position[0], msg->position[1], msg->position[2]);
				if(isnanf(_position(0)) || 
						isnanf(_position(1)) ||
						isnanf(_position(2))) {
					RCLCPP_WARN(rclcpp::get_logger("OFFBOARD"), "INVALID POSITION: %10.5f, %10.5f, %10.5f",
						_position(0), _position(1), _position(2));
				}

				if(!_first_odom){
					_x.pose.position.x = _position(0);
					_x.pose.position.y = _position(1);
					_x.pose.position.z = _position(2);
					_x.pose.orientation.x = _attitude(1); 
					_x.pose.orientation.y = _attitude(2);
					_x.pose.orientation.z = _attitude(3);
					_x.pose.orientation.w = _attitude(0);
					_prev_sp = _position;
					_prev_att_sp = _attitude;
					_prev_yaw_sp = matrix::Eulerf(_attitude).psi();
					std::cout << "------------------- Yaw: " << _prev_yaw_sp << std::endl;

					_trajectory._last_x.pose.position.x = _position(0);
					_trajectory._last_x.pose.position.y = _position(1);
					_trajectory._last_x.pose.position.z = _position(2);
					_trajectory._last_x.pose.orientation.x = _attitude(1); 
					_trajectory._last_x.pose.orientation.y = _attitude(2);
					_trajectory._last_x.pose.orientation.z = _attitude(3);
					_trajectory._last_x.pose.orientation.w = _attitude(0);
				}

				_first_odom = true;
			});

	_x = {};
	_xd = {};
	_xdd = {};

	
    //_px4_vehicle_status_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", qos, std::bind(&OffboardControl::status_callback, this, std::placeholders::_1));

	_octo_sub = this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_binary", qos, std::bind(&OffboardControl::octomap_callback, this, std::placeholders::_1));
	_map_set = false;	

	_offboard_setpoint_counter = 0;
	
	rmw_qos_profile_t cmd_qos_profile = rmw_qos_profile_sensor_data;
	cmd_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
	cmd_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE; 
	cmd_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;

	auto cmd_qos = rclcpp::QoS(rclcpp::QoSInitialization(cmd_qos_profile.history, 1), cmd_qos_profile);

	_cmd_sub = this->create_subscription<trajectory_planner::msg::MoveCmd>("move_cmd", cmd_qos, std::bind(&OffboardControl::move_command_callback, this, std::placeholders::_1));

	_timer = this->create_wall_timer(_timer_period, std::bind(&OffboardControl::offboard_callback, this));

	_status_timer = this->create_wall_timer(_status_timer_period, std::bind(&OffboardControl::status_update, this));


	//boost::thread key_input_t( &OffboardControl::key_input, this );
	boost::thread move_cmd_t( &OffboardControl::move_cmd, this );

	//---Init planner
	this->declare_parameter("max_yaw_rate", .1);
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
	
	this->declare_parameter("x_upper_bound",21.5);
	_xbounds[1] = this->get_parameter("x_upper_bound").as_double();
	RCLCPP_INFO(get_logger(), "x_upper_bound: %f", _xbounds[1] );

	this->declare_parameter("y_lower_bound", -5.0);
	_ybounds[0] = this->get_parameter("y_lower_bound").as_double();
	RCLCPP_INFO(get_logger(), "y_lower_bound: %f", _ybounds[0]);
	
	this->declare_parameter("y_upper_bound",11.0);
	_ybounds[1] = this->get_parameter("y_upper_bound").as_double();
	RCLCPP_INFO(get_logger(), "y_upper_bound: %f", _ybounds[1] );

	this->declare_parameter("z_lower_bound", 1.0);
	_zbounds[0] = this->get_parameter("z_lower_bound").as_double();
	RCLCPP_INFO(get_logger(), "z_lower_bound: %f", _ybounds[0]);
	
	this->declare_parameter("z_upper_bound",1.8);
	_zbounds[1] = this->get_parameter("z_upper_bound").as_double();
	RCLCPP_INFO(get_logger(), "z_upper_bound: %f", _ybounds[1] );

	this->declare_parameter("x_valid_min",-10.0);
	_x_valid_min = this->get_parameter("x_valid_min").as_double();
	RCLCPP_INFO(get_logger(), "x_valid_min: %f", _x_valid_min);

	this->declare_parameter("y_valid_min",-10.0);
	_y_valid_min = this->get_parameter("y_valid_min").as_double();
	RCLCPP_INFO(get_logger(), "y_valid_min: %f", _y_valid_min);

	this->declare_parameter("x_valid_max",19.5);
	_x_valid_max = this->get_parameter("x_valid_max").as_double();
	RCLCPP_INFO(get_logger(), "x_valid_max: %f", _x_valid_max);

	this->declare_parameter("y_valid_max",9.5);
	_y_valid_max = this->get_parameter("y_valid_max").as_double();
	RCLCPP_INFO(get_logger(), "y_valid_max: %f", _y_valid_max);

	this->declare_parameter("z_motion_threshold",0.2);
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
	_replan_cnt = 0;
    
	#ifdef SIMULATION
		_T_enu_to_ned.setZero();
		_T_enu_to_ned(0,1) =  1.0;
		_T_enu_to_ned(1,0) =  1.0;
		_T_enu_to_ned(2,2) = -1.0;
	#else
		_T_enu_to_ned.setZero();
		_T_enu_to_ned(0,0) = 1.0;
		_T_enu_to_ned(1,1) = -1.0;
		_T_enu_to_ned(2,2) = -1.0;
	#endif
}

// void OffboardControl::status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg){
// 	_armed = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
// }

void OffboardControl::status_update(){

	// Dummy state machine
	if(_replan_cnt >= _max_replan_iterations || _plan_has_result == false){
		_status = "FAILED";
	}
	else if(_stop_trajectory){_status = "STOPPED";}
	else if(!_trajectory.isReady()){_status = "IDLE";}
	else(_status = "RUNNING");

	// Publish 

	auto message = std_msgs::msg::String();
	message.data = _status;
	_plan_state_publisher->publish(message);
}

void OffboardControl::octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr octo_msg ) {
	octomap::AbstractOcTree* tect = octomap_msgs::binaryMsgToMap(*octo_msg);
    octomap::OcTree* tree_oct = (octomap::OcTree*)tect;
	_pp->set_octo_tree(tree_oct);
	_map_set = true;

}

void OffboardControl::offboard_callback() {

	if(!_first_odom)
		return;

	if (_offboard_setpoint_counter == 10) {
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	}


	_trajectory.getNext(_x,_xd,_xdd);

	// offboard_control_mode needs to be paired with trajectory_setpoint
	publish_offboard_control_mode();
	publish_trajectory_setpoint();

	// stop the counter after reaching 101
	if (_offboard_setpoint_counter < 11) {
		_offboard_setpoint_counter++;
	}

}

void OffboardControl::move_command_callback(const trajectory_planner::msg::MoveCmd::SharedPtr msg) {
	matrix::Vector3f sp;
	std::string cmd;
	cmd = msg->command.data;
	sp(0) = msg->pose.position.x;
	sp(1) = msg->pose.position.y;
	sp(2) = msg->pose.position.z;	
	
	_new_command = (cmd != _cmd || _cmd_sp != sp);
	_cmd = cmd;
	_cmd_sp = sp;
	if(cmd == "stop"){
		_stop_trajectory = true;
	}

	RCLCPP_INFO(get_logger(), "Command received: %s. Pose: %f,%f,%f", msg->command.data.c_str(),_cmd_sp(0),_cmd_sp(1),_cmd_sp(2));
	
}

void OffboardControl::move_cmd(){
	
	std::string cmd;
	matrix::Vector3f sp;
	Eigen::Vector3d wp;
	matrix::Vector3f current_sp;
	float duration;
	float yaw_time;
	float  yaw_d;
	bool plan_has_result = true;

	while(rclcpp::ok() && !_first_odom) {
		usleep(0.1e6);
	}

	while(rclcpp::ok()) {

		//status_update(); // Asyncronous status pub
		 
		if(_new_command) {
			
			RCLCPP_INFO(get_logger(), "Command accepted: %s. Pose: %f,%f,%f",_cmd.c_str(),_cmd_sp(0),_cmd_sp(1),_cmd_sp(2));
			_new_command = false;
			cmd = _cmd;
			current_sp = _cmd_sp;
			sp = current_sp;
			_replan_cnt = 0;

			if(cmd =="nav"){			

				do{
					_replan = true;
					_wp_traj_completed = false;
					wp[0] = current_sp(0);
					wp[1] = current_sp(1);
					wp[2] = current_sp(2);

					auto opt_poses = std::make_shared<std::vector<POSE>>();
					plan_has_result = plan(wp,opt_poses);

					_stop_trajectory = false;

					if(plan_has_result){

						_replan = false;

						int wp_index = 1;

						auto id_wp_ptr = std::shared_ptr<int>(new int(0));
						
						std::vector<POSE> poses_to_check = *opt_poses;

						boost::thread check_path_t( &OffboardControl::check_path, this, poses_to_check, id_wp_ptr); 

						while(_wp_traj_completed == false && !_stop_trajectory ) {	

							if(!_replan && wp_index<int(opt_poses->size())){
							
								sp(0) = (*opt_poses)[wp_index].position.x;
								sp(1) = (*opt_poses)[wp_index].position.y;
								sp(2) = (*opt_poses)[wp_index].position.z;    
								
								sp = _T_enu_to_ned*sp;
								
								compute_time_and_heading(sp, yaw_d, yaw_time, duration);
								
								start_traj(_prev_sp, yaw_d, yaw_time);  //Blocking
								start_traj(sp, yaw_d, duration);	
								*id_wp_ptr = wp_index;
								wp_index++;

							}
							if(_replan || _stop_trajectory){
								stop_traj();
								_replan_cnt++;
								break;
							}
						}
					}
					std::cout<<"Replan: "<<_replan_cnt<<std::endl;
				}while(_wp_traj_completed == false && _replan_cnt < _max_replan_iterations && !_stop_trajectory);

			}
			else if(cmd == "takeoff") {
				
				RCLCPP_INFO(this->get_logger(),"TAKEOFF command received: %f", sp(2));

				sp(0) = _position(0);
				sp(1) = _position(1);
				sp(2) = -current_sp(2);
			
				yaw_d = matrix::Eulerf(_attitude).psi(); 
				yaw_d = _prev_yaw_sp; 
				duration = 1.0 + std::abs(sp(2))/(_max_velocity);
				this->flight_termination(0);
				this->arm();

				_replan = false;
				_stop_trajectory = false;
				 
				start_traj(sp, yaw_d, duration);		

			}
			else if(cmd == "land") {
				RCLCPP_INFO(this->get_logger(),"LAND command received");
				sp(0) = _position(0);
				sp(1) = _position(1);
				sp(2) = 0.5; 
				current_sp = sp;
				_replan = false;
				_stop_trajectory = false; 
				start_traj(sp, yaw_d, 15);	 // TODO tune time
				
			}
			else if(cmd == "arm") {
				RCLCPP_INFO(this->get_logger(),"ARM command received");
				
				this->flight_termination(0);
				this->arm();
			}
			else if(cmd == "term") {
				RCLCPP_INFO(this->get_logger(),"TERM command received");
				this->flight_termination(1);

			}else if(cmd == "stop"){
				_stop_trajectory = true;

				#ifdef PLAN_FROM_LAST_MEAS
				_stop_sp = _position;
				_stop_att_sp = _attitude;
				_prev_yaw_sp = matrix::Eulerf(_attitude).psi();
				#endif
				stop_traj();
				_status = "STOPPED";
			}
			
		}
		usleep(0.1e6);
	}
}

void OffboardControl::key_input() {
	bool exit = false;
	bool plan_has_result = true;
	std::string cmd;
	matrix::Vector3f sp;
	float duration;
	float yaw_time;
	float  yaw_d;

	while(rclcpp::ok() && !_first_odom) {
		usleep(0.1e6);
	}

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

			compute_time_and_heading(sp, yaw_d, yaw_time, duration);

			_replan = false;
			
			_stop_trajectory = false; 
			start_traj(_prev_sp, yaw_d, yaw_time);
			
			start_traj(sp, yaw_d, duration);

		}
		else if(cmd =="nav"){
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
			plan_has_result = plan(wp,opt_poses);

			_stop_trajectory = false;

			if(plan_has_result){

				//_replan = false;

				// double s[3]; 
				//CARTESIAN_PLANNER trajectory{_timer_freq};
				// Eigen::Vector3d pt_check;
				// visualization_msgs::msg::Marker check_m;
				// // Set the frame, timestamp, and namespace
				// check_m.header.frame_id = "map";
				// check_m.header.stamp = this->get_clock()->now();
				// check_m.ns = "check";

				// // Set marker properties
				// check_m.type = visualization_msgs::msg::Marker::CUBE;
				// check_m.action = visualization_msgs::msg::Marker::ADD;

				// // Set the scale of the marker
				// check_m.scale.x = 0.15;
				// check_m.scale.y = 0.15;
				// check_m.scale.z = 0.15;

				// // Set the color of the marker
				// check_m.color.r = 1.0f;
				// check_m.color.g = 0.0f;
				// check_m.color.b = 0.0f;
				// check_m.color.a = 1.0;

				// // Marker lifetime
				// check_m.lifetime = rclcpp::Duration(0, 0); // Infinite lifetime

				// // Set marker ID
				// check_m.id = 97;
				// bool valid_path;

				// start_wp_traj(opt_poses, trajectory);

				// _trajectory = trajectory;

				// while(_replan == false && _trajectory.isReady() == true){
					
				// 	for(int i = _trajectory.getCounter(); i < trajectory._x.size()-1; i++){
				
				// 		s[0] = trajectory._x[i].pose.position.y;
				// 		s[1] = trajectory._x[i].pose.position.x;
				// 		s[2] = -trajectory._x[i].pose.position.z;
				// 		valid_path = _pp->check_state(s);
				// 		check_m.pose.position.x = s[0];
				// 		check_m.pose.position.y = s[1];
				// 		check_m.pose.position.z = s[2];
				// 		_check_path_pub->publish( check_m );

				// 		if(!valid_path) break;

				// 	}
					
				// 	if( !valid_path ) {
				// 		RCLCPP_WARN(get_logger(), "New obstacle detected! Replan");
				// 		_replan = true;
				// 		_stop_trajectory = true;
				// 		stop_traj();
				// 	}
				// }

				_replan = false;

				int wp_index = 1;

				auto id_wp_ptr = std::shared_ptr<int>(new int(0));
				//*id_wp_ptr = wp_index;

				std::vector<POSE> poses_to_check = *opt_poses;

				boost::thread check_path_t( &OffboardControl::check_path, this, poses_to_check, id_wp_ptr); 

				while(wp_index <= int(opt_poses->size()) && _wp_traj_completed == false) {	

					if(!_stop_trajectory && wp_index<int(opt_poses->size())){
					
						sp(0) = (*opt_poses)[wp_index].position.x;
						sp(1) = (*opt_poses)[wp_index].position.y;
						sp(2) = (*opt_poses)[wp_index].position.z;    
						
						sp = _T_enu_to_ned*sp;
						
						compute_time_and_heading(sp, yaw_d, yaw_time, duration);
						
						start_traj(_prev_sp, yaw_d, yaw_time);  //Blocking
						start_traj(sp, yaw_d, duration);	
						
						*id_wp_ptr = wp_index;
						wp_index++;

					}
					if(_stop_trajectory){
						stop_traj();
						break;
					}
					
				}

			}

		}
		else if(cmd == "takeoff") {

			sp = _position;
			std::cout << "Enter takeoff altitude (ENU frame): "; 
			std::cin >> sp(2);
			sp(2) = -sp(2);
			
			yaw_d = matrix::Eulerf(_attitude).psi(); 
			
			std::cout << yaw_d << std::endl;
			duration = 1.0 + std::sqrt(pow(sp(2) - _prev_sp(2),2))/(2*_max_velocity);

			this->flight_termination(0);
			this->arm();
			_stop_trajectory = false; 
		
			start_traj(sp, yaw_d, duration);
		
		}
		else if(cmd == "land") {
			std::cout << "Landing procedure triggered... \nRemember to kill disarm manually after landed.\n";
			sp = _prev_sp;
			sp(2) = 0.5; 
			_stop_trajectory = false; 
			yaw_d = _prev_yaw_sp; 
			start_traj(sp, yaw_d, 15);	 // TODO tune time
		}
		else if(cmd == "arm") {
			RCLCPP_INFO(this->get_logger(),"Arm command received");
			
			this->flight_termination(0);
			this->arm();
		}
		else if(cmd == "term") {
			this->flight_termination(1);		 // TODO unire term a land (check odometry feedback)
		}
		else if(cmd == "stop") {

			_stop_trajectory = true;

			#ifdef PLAN_FROM_LAST_MEAS
			_stop_sp = _position;
			_stop_att_sp = _attitude;
			_prev_yaw_sp = matrix::Eulerf(_attitude).psi();
			#endif
			stop_traj();
			_status = "STOPPED";

		}else {
			std::cout << "Unknown command;\n";

		}

	}

}

void OffboardControl::arm() {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO_ONCE(this->get_logger(), "Arm command send");
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

void OffboardControl::start_wp_traj(std::shared_ptr<std::vector<POSE>> opt_poses, CARTESIAN_PLANNER & trajectory) {

	matrix::Vector3f sp;
	matrix::Vector3f prev_sp;
	matrix::Quaternionf prev_att_sp;
	matrix::Quaternionf att;
	float prev_yaw_sp;
	double duration;
	double yaw_time;
	float yaw_d;

	std::vector<geometry_msgs::msg::PoseStamped> poses;
	std::vector<double> times;
	geometry_msgs::msg::PoseStamped p;
	double t = 0.0f;

	prev_sp = _prev_sp;
	prev_att_sp = _prev_att_sp;
	prev_yaw_sp = matrix::Eulerf(prev_att_sp).psi();

	p.pose.position.x = prev_sp(0);
	p.pose.position.y = prev_sp(1);
	p.pose.position.z = prev_sp(2); 

	p.pose.orientation.w = prev_att_sp(0);
	p.pose.orientation.x = prev_att_sp(1);
	p.pose.orientation.y = prev_att_sp(2);
	p.pose.orientation.z = prev_att_sp(3);

	poses.push_back(p);
	times.push_back(t);

	for(int i = 1; i<int(opt_poses->size()); i++) {
		sp(0) = (*opt_poses)[i].position.x;
		sp(1) = (*opt_poses)[i].position.y;
		sp(2) = (*opt_poses)[i].position.z; 
		
		sp = _T_enu_to_ned*sp;

		yaw_d = atan2(sp(1)-prev_sp(1),sp(0)-prev_sp(0)); 
		yaw_d = std::isnan(yaw_d) ? prev_yaw_sp : yaw_d;
		att = matrix::Eulerf(0, 0, yaw_d);

		yaw_time = 1.0 + std::abs(prev_yaw_sp - yaw_d)/_max_yaw_rate;
		duration = 1.0 + std::sqrt(pow(sp(0) - prev_sp(0),2)+pow(sp(1) - prev_sp(1),2)+pow(sp(2) - prev_sp(2),2))/_max_velocity;


		p.pose.position.x = prev_sp(0);
		p.pose.position.y = prev_sp(1);
		p.pose.position.z = prev_sp(2); 

		p.pose.orientation.w = att(0);
		p.pose.orientation.x = att(1);
		p.pose.orientation.y = att(2);
		p.pose.orientation.z = att(3);
		
		t = t + yaw_time;
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

		t = t + yaw_time + duration;
		poses.push_back(p);
		times.push_back(t);

		prev_sp = sp;
		prev_att_sp = att;

	}
	trajectory.set_waypoints(poses, times);

	trajectory.compute();

	_prev_sp = sp;
	_prev_att_sp = prev_att_sp;
	_prev_yaw_sp = matrix::Eulerf(prev_att_sp).psi();
}

void OffboardControl::start_traj(matrix::Vector3f pos, float yaw, double d) {
	
	while(_trajectory.isReady()){   //wait for the previous trajectory to complete, exit if replan or stop
		usleep(0.1e6);
		if(_replan || _stop_trajectory) return;	
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

bool OffboardControl::plan(Eigen::Vector3d wp, std::shared_ptr<std::vector<POSE>> opt_poses) {

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
		return false;
    }
    else {
		if( ret < 0 || (*opt_poses).size() < 2 ){
			RCLCPP_ERROR(get_logger(),"Planner not correctly initialized"); 
			return false;
		}else {

			for( int i=0; i<(*opt_poses).size(); i++ ) {
				p.pose.position.x = (*opt_poses)[i].position.x;
				p.pose.position.y = (*opt_poses)[i].position.y;
				p.pose.position.z = (*opt_poses)[i].position.z;

				p.pose.orientation.x = (*opt_poses)[i].orientation.x;
				p.pose.orientation.y = (*opt_poses)[i].orientation.y;
				p.pose.orientation.z = (*opt_poses)[i].orientation.y;
				p.pose.orientation.w = (*opt_poses)[i].orientation.z;

				generated_path.poses.push_back( p );
				std::cout<<i<<": p("<<p.pose.position.x<<", "<<p.pose.position.y<<", "<<p.pose.position.z<<")\n";

			}
			std::cout<<"Publish path\n";
			_path_publisher->publish( generated_path );

			std::cout << "Solution: " << std::endl;
			
			for(int i=0; i<(*opt_poses).size(); i++ ) {
				std::cout << "Pose: [" << i << "]: " << "(" << (*opt_poses)[i].position.x << " " << (*opt_poses)[i].position.y << " " << (*opt_poses)[i].position.z << ")" << std::endl;
			}
		}
	}
	return true;
}

void OffboardControl::check_path(const std::vector<POSE> & poses, const std::shared_ptr<int> wp) {
    
	usleep(0.01e6);

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


	while( valid_path && !_stop_trajectory && *wp < poses.size() && !_wp_traj_completed && !_replan){	 // continue checking while executing

		if(*wp != 0){
			// if(SIMULATION ==1)
			// 	pt_i << _x.pose.position.y, _x.pose.position.x, -_x.pose.position.z; 
			// else
			// 	pt_i << _x.pose.position.x, -_x.pose.position.y, -_x.pose.position.z;

			#ifdef SIMULATION
				pt_i << _position(1), _position(0), -_position(2); 
			#else
				pt_i << _position(0), -_position(1),- _position(2);
			#endif

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
			_wp_traj_completed = (!_trajectory.isReady() && *wp == poses.size()-1);
		}
		
		if( _wp_traj_completed ){

			RCLCPP_WARN(get_logger(), "Checking path ENDED - Trajectory completed");
		}
	}
	if( !valid_path ) {
		RCLCPP_WARN(get_logger(), "New obstacle detected! Replan");
		_replan = true;
		//_stop_trajectory = true;
		
		#ifdef PLAN_FROM_LAST_MEAS
		_stop_sp = _position;
		_stop_att_sp = _attitude;
		_prev_yaw_sp = matrix::Eulerf(_attitude).psi();
		#endif
 
	}
}   

void OffboardControl::compute_time_and_heading(const matrix::Vector3f & sp, float & yaw_d, float & yaw_time, float & duration){

	if(START_FROM_LAST_MEAS){
		_prev_sp = _position;
		_prev_att_sp = _attitude;
		_prev_yaw_sp = matrix::Eulerf(_attitude).psi();
	}

	yaw_d = atan2(sp(1)-_prev_sp(1),sp(0)-_prev_sp(0)); 
	yaw_d = std::isnan(yaw_d) ? _prev_yaw_sp : yaw_d;
	yaw_time = 1.0 + std::abs(_prev_yaw_sp - yaw_d)/_max_yaw_rate;
	duration = 0.1 + std::sqrt(pow(sp(0) - _prev_sp(0),2)+pow(sp(1) - _prev_sp(1),2)+pow(sp(2) - _prev_sp(2),2))/_max_velocity;

}

void OffboardControl::stop_traj(){

	// Clear trajectory

	_trajectory.clear_waypoints();

	_xd.twist.linear.x = 0.0;
	_xd.twist.linear.y = 0.0;
	_xd.twist.linear.z = 0.0;

	_xdd.accel.linear.x = 0.0;
	_xdd.accel.linear.y = 0.0;
	_xdd.accel.linear.z = 0.0;

	_trajectory._last_x = _x;
	_trajectory._last_xd = _xd;
	_trajectory._last_xdd = _xdd;

	#ifdef PLAN_FROM_LAST_MEAS
	_x.pose.position.x = _stop_sp(0);
	_x.pose.position.y = _stop_sp(1);
	_x.pose.position.z = _stop_sp(2);

	_prev_sp = _stop_sp;
	_prev_att_sp =_stop_att_sp;
	_prev_yaw_sp = matrix::Eulerf(_prev_att_sp).psi();

	#else
	// Init for

	_prev_sp(0) = _x.pose.position.x;
	_prev_sp(1) = _x.pose.position.y;
	_prev_sp(2) = _x.pose.position.z;
	_prev_att_sp(0) = _x.pose.orientation.w;
	_prev_att_sp(1) = _x.pose.orientation.x;
	_prev_att_sp(2) = _x.pose.orientation.y;
	_prev_att_sp(3) = _x.pose.orientation.z;
	_prev_yaw_sp = matrix::Eulerf(_prev_att_sp).psi();

	#endif
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

