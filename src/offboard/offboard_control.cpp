#include "offboard_control.hpp"
#include <random>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#define START_FROM_LAST_MEAS 0
#define SIMULATION 1


OffboardControl::OffboardControl() : rclcpp::Node("offboard_control"), _state(STOPPED) {

	this->declare_parameter("offboard_control_mode_topic", "fmu/in/offboard_control_mode");
	_offboard_control_mode_topic = this->get_parameter("offboard_control_mode_topic").as_string();
	RCLCPP_INFO(get_logger(), "offboard_control_mode_topic: %s", _offboard_control_mode_topic.c_str());

	this->declare_parameter("vehicle_command_topic", "fmu/in/vehicle_command");
	_vehicle_command_topic = this->get_parameter("vehicle_command_topic").as_string();
	RCLCPP_INFO(get_logger(), "vehicle_command_topic: %s", _vehicle_command_topic.c_str());

	this->declare_parameter("trajectory_setpoint_topic", "/px4/trajectory_setpoint_enu");
	_trajectory_setpoint_topic = this->get_parameter("trajectory_setpoint_topic").as_string();
	RCLCPP_INFO(get_logger(), "trajectory_setpoint_topic: %s", _trajectory_setpoint_topic.c_str());

	this->declare_parameter("odom_topic", "/px4/odometry/out");
	_odom_topic = this->get_parameter("odom_topic").as_string();
	RCLCPP_INFO(get_logger(), "odom_topic: %s", _odom_topic.c_str());

	this->declare_parameter("octomap_topic", "/octomap_binary");
	_octomap_topic = this->get_parameter("octomap_topic").as_string();
	RCLCPP_INFO(get_logger(), "octomap_topic: %s", _octomap_topic.c_str());

	this->declare_parameter("move_cmd_topic", "move_cmd");
	_move_cmd_topic = this->get_parameter("move_cmd_topic").as_string();
	RCLCPP_INFO(get_logger(), "move_cmd_topic: %s", _move_cmd_topic.c_str());

	this->declare_parameter("path_topic", "rrt/path");
	_path_topic = this->get_parameter("path_topic").as_string();
	RCLCPP_INFO(get_logger(), "path_topic: %s", _path_topic.c_str());

	this->declare_parameter("check_path_topic", "/leo/drone/check_path");
	_check_path_topic = this->get_parameter("check_path_topic").as_string();
	RCLCPP_INFO(get_logger(), "check_path_topic: %s", _check_path_topic.c_str());

	this->declare_parameter("plan_status_topic", "/leo/drone/plan_status");
	_plan_status_topic = this->get_parameter("plan_status_topic").as_string();
	RCLCPP_INFO(get_logger(), "plan_status_topic: %s", _plan_status_topic.c_str());


	this->declare_parameter("tf_buffer_timeout", 0.5);
	_tf_buffer_timeout = this->get_parameter("tf_buffer_timeout").as_double();
	RCLCPP_INFO(get_logger(), "tf_buffer_timeout: %.2f", _tf_buffer_timeout);

	this->declare_parameter("parent_transform", "map");
	_parent_transf = this->get_parameter("parent_transform").as_string();
	RCLCPP_INFO(get_logger(), "parent_transform: %s", _parent_transf.c_str());

	this->declare_parameter("child_transform", "odom");
	_child_transf = this->get_parameter("child_transform").as_string();
	RCLCPP_INFO(get_logger(), "child_transform: %s", _child_transf.c_str());

	this->declare_parameter("check_frame_id", "odom");
	_check_frame_id = this->get_parameter("check_frame_id").as_string();
	RCLCPP_INFO(get_logger(), "check_frame_id: %s", _check_frame_id.c_str());

	_offboard_control_mode_publisher =
		this->create_publisher<OffboardControlMode>(_offboard_control_mode_topic, 10);

	_vehicle_command_publisher =
		this->create_publisher<VehicleCommand>(_vehicle_command_topic, 10);
	
	_trajectory_setpoint_publisher =
		this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(_trajectory_setpoint_topic, 10);
	
	/*Visualization*/
	_path_publisher = this->create_publisher<nav_msgs::msg::Path>(_path_topic, 1);
	_check_path_pub = this->create_publisher<visualization_msgs::msg::Marker>(_check_path_topic, 1);
	_plan_state_publisher = this->create_publisher<std_msgs::msg::String>(_plan_status_topic, 1);

	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(_odom_topic, qos,
				[this](const nav_msgs::msg::Odometry::UniquePtr msg) {

			_attitude = matrix::Quaternionf(
				msg->pose.pose.orientation.w,
				msg->pose.pose.orientation.x,
				msg->pose.pose.orientation.y,
				msg->pose.pose.orientation.z
			);

			_position = matrix::Vector3f(
				msg->pose.pose.position.x,
				msg->pose.pose.position.y,
				msg->pose.pose.position.z
			);

			if (std::isnan(_position(0)) || std::isnan(_position(1)) || std::isnan(_position(2))) {
				RCLCPP_WARN(
					this->get_logger(), 
					"INVALID POSITION: %.5f, %.5f, %.5f",
					_position(0), _position(1), _position(2)
				);
				return;  
			}

			if (!_first_odom) {

				_x.pose.position.x = _position(0);
				_x.pose.position.y = _position(1);
				_x.pose.position.z = _position(2);
				_x.pose.orientation = msg->pose.pose.orientation;  

				_prev_sp = _position;
				_prev_att_sp = _attitude;
				_prev_yaw_sp = matrix::Eulerf(_attitude).psi();

				RCLCPP_DEBUG(
					this->get_logger(),
					"Initialized odometry - Yaw: %.5f",
					_prev_yaw_sp
				);

				_trajectory._last_x.pose = msg->pose.pose;
			}

			_first_odom = true;
		}
	);

	_land_detect_sub = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
		"/fmu/out/vehicle_land_detected", qos,
		[&](const px4_msgs::msg::VehicleLandDetected::SharedPtr land_msg) {
			if ((land_msg->ground_contact || land_msg->maybe_landed || land_msg->landed) && _last_cmd == "land") {
				_is_flying = false;
				this->disarm();
				this->flight_termination(1);
			} else {
				_is_flying = true;
			}
		});

	// Joy subscriber for teleop
	_joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
		"/joy", qos, std::bind(&OffboardControl::joy_callback, this, std::placeholders::_1));

	_x = {};
	_xd = {};
	_xdd = {};
	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	_octo_sub = this->create_subscription<octomap_msgs::msg::Octomap>(_octomap_topic, qos, std::bind(&OffboardControl::octomap_callback, this, std::placeholders::_1));
	_map_set = false;	

	_offboard_setpoint_counter = 0;
	
	rmw_qos_profile_t cmd_qos_profile = rmw_qos_profile_sensor_data;
	cmd_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
	cmd_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE; 
	cmd_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;

	auto cmd_qos = rclcpp::QoS(rclcpp::QoSInitialization(cmd_qos_profile.history, 1), cmd_qos_profile);

	_cmd_sub = this->create_subscription<trajectory_planner::msg::MoveCmd>(_move_cmd_topic, cmd_qos, std::bind(&OffboardControl::move_command_callback, this, std::placeholders::_1));

	_timer = this->create_wall_timer(_timer_period, std::bind(&OffboardControl::offboard_callback, this));
	_status_timer = this->create_wall_timer(_status_timer_period, std::bind(&OffboardControl::status_update, this));

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

	this->declare_parameter("use_key_input", 1.0);
	_use_key_input = this->get_parameter("use_key_input").as_double(); // as_bool not working
	RCLCPP_INFO(get_logger(), "use_key_input: %f", _use_key_input);	

	this->declare_parameter("do_transform", 1.0);
	_do_transform = this->get_parameter("do_transform").as_double(); // as_bool not working
	RCLCPP_INFO(get_logger(), "do_transform: %f", _do_transform);

	// Teleop parameters
	this->declare_parameter("teleop_max_vel", 1.0);
	_teleop_max_vel = this->get_parameter("teleop_max_vel").as_double();
	RCLCPP_INFO(get_logger(), "teleop_max_vel: %f", _teleop_max_vel);

	this->declare_parameter("teleop_max_yaw_rate", 1.0);
	_teleop_max_yaw_rate = this->get_parameter("teleop_max_yaw_rate").as_double();
	RCLCPP_INFO(get_logger(), "teleop_max_yaw_rate: %f", _teleop_max_yaw_rate);

    _pp = new PATH_PLANNER();
    _pp->init( _xbounds, _ybounds, _zbounds);
    _pp->set_robot_geometry(_robot_radius);
	_replan_cnt = 0;

	if (_use_key_input > 0.0f) {
		RCLCPP_INFO(this->get_logger(), "Key input mode active");
		boost::thread key_input_t(&OffboardControl::key_input, this);
	} else {
		RCLCPP_INFO(this->get_logger(), "Move command mode active");
		boost::thread move_cmd_t(&OffboardControl::move_cmd, this);
	}
	// Start the TF lookup thread
	boost::thread tf_lookup_t(&OffboardControl::tf_lookup_loop, this);
    
	// #ifdef SIMULATION
	// 	_T_enu_to_ned.setZero();
	// 	_T_enu_to_ned(0,1) =  1.0;
	// 	_T_enu_to_ned(1,0) =  1.0;
	// 	_T_enu_to_ned(2,2) = -1.0;
	// #else
	// 	_T_enu_to_ned.setZero();
	// 	_T_enu_to_ned(0,0) = 1.0;
	// 	_T_enu_to_ned(1,1) = -1.0;
	// 	_T_enu_to_ned(2,2) = -1.0;
	// #endif
}

void OffboardControl::tf_lookup_loop() {
	RCLCPP_INFO(this->get_logger(), "TF lookup thread started");

	rclcpp::Rate rate(100);
	while (rclcpp::ok()) {
		try {
			rclcpp::Time now = this->get_clock()->now();
			rclcpp::Duration timeout = rclcpp::Duration::from_seconds(_tf_buffer_timeout);  // timeout for waiting
			_tf_map_to_odom = tf_buffer_->lookupTransform(_child_transf, _parent_transf, now, timeout);
			_tf_odom_to_map = tf_buffer_->lookupTransform(_parent_transf, _child_transf, now, timeout);
			// _tf_map_to_odom = tf_buffer_->lookupTransform(_child_transf, _parent_transf, tf2::TimePointZero);
			// _tf_odom_to_map = tf_buffer_->lookupTransform(_parent_transf, _child_transf, tf2::TimePointZero);
		} catch (const tf2::TransformException &ex) {
			RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
		}
		rate.sleep();
	}
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

	if (_teleop_active) {
		// In teleop mode: update _x, _xd, _xdd from teleop variables
		// Position
		_x.pose.position.x = _teleop_position(0);
		_x.pose.position.y = _teleop_position(1);
		_x.pose.position.z = _teleop_position(2);
		
		// Orientation from yaw
		matrix::Quaternionf q_target(matrix::Eulerf(0, 0, _teleop_yaw));
		_x.pose.orientation.w = q_target(0);
		_x.pose.orientation.x = q_target(1);
		_x.pose.orientation.y = q_target(2);
		_x.pose.orientation.z = q_target(3);
		
		// Velocity
		_xd.twist.linear.x = _teleop_velocity(0);
		_xd.twist.linear.y = _teleop_velocity(1);
		_xd.twist.linear.z = _teleop_velocity(2);
		_xd.twist.angular.x = 0.0;
		_xd.twist.angular.y = 0.0;
		_xd.twist.angular.z = _teleop_yawspeed;
		
		// Acceleration (zero per ora)
		_xdd.accel.linear.x = 0.0;
		_xdd.accel.linear.y = 0.0;
		_xdd.accel.linear.z = 0.0;
		_xdd.accel.angular.x = 0.0;
		_xdd.accel.angular.y = 0.0;
		_xdd.accel.angular.z = 0.0;
		
		// Salva la posizione teleop corrente per continuità - ma solo se teleop è stabile
		// Questo evita conflitti durante la transizione
		_prev_sp = _teleop_position;
		_prev_yaw_sp = _teleop_yaw;
		_prev_att_sp = q_target;
	} else {
		// Normal trajectory mode: get values from trajectory planner
		if (_trajectory.isReady()) {
			// Se c'è una traiettoria attiva, usala
			_trajectory.getNext(_x,_xd,_xdd);
			
			// Aggiorna _prev_sp con la posizione corrente della traiettoria
			// Questo ci assicura che quando la traiettoria finisce, _prev_sp sia aggiornato
			_prev_sp(0) = _x.pose.position.x;
			_prev_sp(1) = _x.pose.position.y;
			_prev_sp(2) = _x.pose.position.z;
			_prev_att_sp(0) = _x.pose.orientation.w;
			_prev_att_sp(1) = _x.pose.orientation.x;
			_prev_att_sp(2) = _x.pose.orientation.y;
			_prev_att_sp(3) = _x.pose.orientation.z;
			_prev_yaw_sp = matrix::Eulerf(_prev_att_sp).psi();
		} else {
			// Se non c'è traiettoria attiva, mantieni la posizione corrente (_prev_sp)
			// Questo evita di pubblicare vecchi setpoint durante l'attesa dell'input utente
			_x.pose.position.x = _prev_sp(0);
			_x.pose.position.y = _prev_sp(1);
			_x.pose.position.z = _prev_sp(2);
			_x.pose.orientation.w = _prev_att_sp(0);
			_x.pose.orientation.x = _prev_att_sp(1);
			_x.pose.orientation.y = _prev_att_sp(2);
			_x.pose.orientation.z = _prev_att_sp(3);
			
			// Velocità e accelerazione zero quando non c'è traiettoria attiva
			_xd.twist.linear.x = 0.0;
			_xd.twist.linear.y = 0.0;
			_xd.twist.linear.z = 0.0;
			_xd.twist.angular.x = 0.0;
			_xd.twist.angular.y = 0.0;
			_xd.twist.angular.z = 0.0;
			
			_xdd.accel.linear.x = 0.0;
			_xdd.accel.linear.y = 0.0;
			_xdd.accel.linear.z = 0.0;
			_xdd.accel.angular.x = 0.0;
			_xdd.accel.angular.y = 0.0;
			_xdd.accel.angular.z = 0.0;
		}
	}

	// Always publish trajectory setpoint (unified for both modes)
	publish_trajectory_setpoint();

	// stop the counter after reaching 11
	if (_offboard_setpoint_counter < 25) {
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
	RCLCPP_INFO(get_logger(), "New command: %d ", _new_command);
}

void OffboardControl::move_cmd(){
	
	std::string cmd;
	matrix::Vector3f sp, sp_odom;
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
			cmd = _cmd;           // received from the last callback
			current_sp = _cmd_sp; // received from the last callback
			sp = current_sp;
			_replan_cnt = 0;
			_last_cmd = cmd;

			// Disattiva teleop se era attivo e viene inserito un nuovo comando
			if (_teleop_active && cmd != "teleop") {
				_teleop_active = false;
				RCLCPP_INFO(this->get_logger(), "Exiting teleop mode due to new command from manager: %s", cmd.c_str());
				
				// Aspetta che il thread teleop finisca completamente per evitare conflitti
				usleep(0.1e6); // 100ms per permettere al thread teleop di uscire
				
				// Forza l'aggiornamento di _prev_sp con l'ultima posizione teleop
				_prev_sp = _teleop_position;
				_prev_yaw_sp = _teleop_yaw;
				_prev_att_sp = matrix::Quaternionf(matrix::Eulerf(0, 0, _teleop_yaw));
				
				// Sincronizza anche _x con la posizione finale del teleop per evitare salti
				_x.pose.position.x = _teleop_position(0);
				_x.pose.position.y = _teleop_position(1);
				_x.pose.position.z = _teleop_position(2);
				_x.pose.orientation.w = _prev_att_sp(0);
				_x.pose.orientation.x = _prev_att_sp(1);
				_x.pose.orientation.y = _prev_att_sp(2);
				_x.pose.orientation.z = _prev_att_sp(3);
			}

			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

			if(cmd == "go") {
				RCLCPP_INFO(this->get_logger(),"GO command received: [%.2f, %.2f, %.2f]", sp(0), sp(1), sp(2));
				
				compute_time_and_heading(sp, yaw_d, yaw_time, duration);

				_replan = false;
				_stop_trajectory = false; 
				start_traj(_prev_sp, yaw_d, yaw_time);
				start_traj(sp, yaw_d, duration);
			}
			else if(cmd =="nav"){			
				RCLCPP_WARN(this->get_logger(), "enter while nav");

				do{	
					RCLCPP_INFO(this->get_logger(), "In cnd nav");
					_replan = true;
					_wp_traj_completed = false;
			
					wp[0] = current_sp(0);
					wp[1] = current_sp(1);
					wp[2] = current_sp(2);

					auto opt_poses = std::make_shared<std::vector<POSE>>();
					plan_has_result = plan(wp,opt_poses);

					_stop_trajectory = false;

					if(plan_has_result){
						RCLCPP_INFO(this->get_logger(), "Path found with %zu waypoints", opt_poses->size());
						_replan = false;

						int wp_index = 1;

						auto id_wp_ptr = std::shared_ptr<int>(new int(0));
						
						std::vector<POSE> poses_to_check = *opt_poses;

						boost::thread check_path_t( &OffboardControl::check_path, this, poses_to_check, id_wp_ptr); 

						while(_wp_traj_completed == false && !_stop_trajectory ) {	
							RCLCPP_INFO(this->get_logger(), "in while, _wp_traj_completed=%d",_wp_traj_completed); //CI VA
							
							if(!_replan && wp_index<int(opt_poses->size())){
							    RCLCPP_INFO(this->get_logger(), "after if"); // VI VA
								sp(0) = (*opt_poses)[wp_index].position.x;
								sp(1) = (*opt_poses)[wp_index].position.y;
								sp(2) = (*opt_poses)[wp_index].position.z;    
								
								if(_do_transform){
									geometry_msgs::msg::PointStamped point_in, point_out;
									point_in.header.stamp = this->get_clock()->now();
									point_in.header.frame_id = _parent_transf; 
									point_in.point.x = sp(0);
									point_in.point.y = sp(1);
									point_in.point.z = sp(2);
									try {
										tf2::doTransform(point_in, point_out, _tf_map_to_odom);
										sp_odom(0) = point_out.point.x;
										sp_odom(1) = point_out.point.y;
										sp_odom(2) = point_out.point.z;
									} catch (const tf2::TransformException &ex) {
										RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
									}
								}
								
								
								compute_time_and_heading(sp_odom, yaw_d, yaw_time, duration);
								
								start_traj(_prev_sp, yaw_d, yaw_time);  //Blocking
								start_traj(sp_odom, yaw_d, duration);	
								
								// Aggiorna _prev_sp dopo aver completato il segmento
								_prev_sp = sp_odom;
								_prev_att_sp = matrix::Quaternionf(matrix::Eulerf(0, 0, yaw_d));
								_prev_yaw_sp = yaw_d;
								
								*id_wp_ptr = wp_index;
								wp_index++;

							}
							if(_replan || _stop_trajectory){
								stop_traj();
								_replan_cnt++;
								break;
							}
						}
						RCLCPP_WARN(this->get_logger(), "fuori while nav (traj pub)");
					}else{
						RCLCPP_WARN(this->get_logger(), "No valid path found for navigation command");
					}
					std::cout<<"Replan: "<<_replan_cnt<<std::endl;

				}while(_wp_traj_completed == false && _replan_cnt < _max_replan_iterations && !_stop_trajectory);

			}
			else if(cmd == "takeoff") {
				
				RCLCPP_INFO(this->get_logger(),"TAKEOFF command received: %f", sp(2));

				sp(0) = _position(0);
				sp(1) = _position(1);
				sp(2) = current_sp(2);
			
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
				sp(2) = -0.5; 
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

			}else if(cmd == "teleop") {
				if (!_joy_available) {
					RCLCPP_ERROR(this->get_logger(), "TELEOP command rejected - Joy node not available!");
					RCLCPP_ERROR(this->get_logger(), "Please make sure joy_node is running and publishing to /joy topic");
					continue;
				}
				
				RCLCPP_INFO(this->get_logger(),"TELEOP command received from manager - entering teleop mode");
				
				// Lancia teleop in un thread separato per non bloccare move_cmd
				boost::thread teleop_thread(&OffboardControl::teleop_mode, this);
			}
			
		}
		usleep(0.1e6);
	}
}

void OffboardControl::key_input() {
	bool exit = false;
	bool plan_has_result = true;
	std::string cmd;
	matrix::Vector3f sp, sp_odom;
	float duration;
	float yaw_time;
	float  yaw_d;

	while(rclcpp::ok() && !_first_odom) {
		usleep(0.1e6);
	}

	while(!exit && rclcpp::ok()) {
		std::cout << "Enter command [arm | takeoff | go | nav | land | term | stop | teleop]: \n"; 
		std::cin >> cmd;

		// Disattiva teleop se era attivo e viene inserito un nuovo comando
		if (_teleop_active && cmd != "teleop") {
			_teleop_active = false;
			RCLCPP_INFO(this->get_logger(), "Exiting teleop mode due to new command: %s", cmd.c_str());
			
			// Aspetta che il thread teleop finisca completamente per evitare conflitti
			usleep(0.1e6); // 100ms per permettere al thread teleop di uscire
			
			// Forza l'aggiornamento di _prev_sp con l'ultima posizione teleop
			_prev_sp = _teleop_position;
			_prev_yaw_sp = _teleop_yaw;
			_prev_att_sp = matrix::Quaternionf(matrix::Eulerf(0, 0, _teleop_yaw));
			
			// Sincronizza anche _x con la posizione finale del teleop per evitare salti
			_x.pose.position.x = _teleop_position(0);
			_x.pose.position.y = _teleop_position(1);
			_x.pose.position.z = _teleop_position(2);
			_x.pose.orientation.w = _prev_att_sp(0);
			_x.pose.orientation.x = _prev_att_sp(1);
			_x.pose.orientation.y = _prev_att_sp(2);
			_x.pose.orientation.z = _prev_att_sp(3);
		}

		_last_cmd = cmd;
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		if(cmd == "go") {
			
			std::cout << "Enter X coordinate (ENU frame): "; 
			std::cin >> sp(0);
			std::cout << "Enter Y coordinate (ENU frame): "; 
			std::cin >> sp(1);
			std::cout << "Enter Z coordinate (ENU frame): "; 
			std::cin >> sp(2);
			
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
			_stop_trajectory = false;

			auto opt_poses = std::make_shared<std::vector<POSE>>();
			plan_has_result = plan(wp,opt_poses);

			_stop_trajectory = false;

			if(plan_has_result){

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
						
						if(_do_transform){
							// Transform point from map frame to odom frame
							geometry_msgs::msg::PointStamped point_in, point_out;
							point_in.header.stamp = this->get_clock()->now();
							point_in.header.frame_id = _parent_transf;
							point_in.point.x = sp(0);
							point_in.point.y = sp(1);
							point_in.point.z = sp(2);
							try {
								tf2::doTransform(point_in, point_out, _tf_map_to_odom);
								sp_odom(0) = point_out.point.x;
								sp_odom(1) = point_out.point.y;
								sp_odom(2) = point_out.point.z;
							} catch (const tf2::TransformException &ex) {
								RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
							}
						}
						compute_time_and_heading(sp_odom, yaw_d, yaw_time, duration);
						
						start_traj(_prev_sp, yaw_d, yaw_time);  //Blocking
						start_traj(sp_odom, yaw_d, duration);	
						
						// Update _prev_sp for next segment
						_prev_sp = sp_odom;
						_prev_att_sp = matrix::Quaternionf(matrix::Eulerf(0, 0, yaw_d));
						_prev_yaw_sp = yaw_d;
						
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
		else if(cmd == "takeoff") { // rotate takeoff land and go from map to odom

			sp = _position;
			std::cout << "Enter takeoff altitude (ENU frame): "; 
			std::cin >> sp(2);
			
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
			sp(2) = -0.5; 
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

		}else if(cmd == "teleop") {
			if (!_joy_available) {
				RCLCPP_ERROR(this->get_logger(), "TELEOP command rejected - Joy node not available!");
				RCLCPP_ERROR(this->get_logger(), "Please make sure joy_node is running and publishing to /joy topic");
				continue;
			}
			
			RCLCPP_INFO(this->get_logger(),"TELEOP command received - entering teleop mode");
			std::cout << "Entering teleop mode. Use joystick to control the drone.\n";
			std::cout << "To exit teleop mode, enter any other command (arm, takeoff, go, nav, land, term, stop).\n";
			
			// Lancia teleop in un thread separato per non bloccare key_input
			boost::thread teleop_thread(&OffboardControl::teleop_mode, this);
		
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

// void OffboardControl::publish_offboard_control_mode() {
// 	OffboardControlMode msg{};
// 	rclcpp::Time now = this->get_clock()->now();

// 	msg.timestamp = now.nanoseconds() / 1000.0;

// 	msg.position = true;
// 	msg.velocity = true;
// 	msg.acceleration = true;
// 	msg.attitude = true;
// 	msg.body_rate = false;

// 	_offboard_control_mode_publisher->publish(msg);
// }

void OffboardControl::publish_trajectory_setpoint() {

	rclcpp::Time now = this->get_clock()->now();

	geometry_msgs::msg::Transform transform;
	transform.translation.x = _x.pose.position.x;
	transform.translation.y = _x.pose.position.y;
	transform.translation.z = _x.pose.position.z;
	transform.rotation.x = _x.pose.orientation.x;
	transform.rotation.y = _x.pose.orientation.y;
	transform.rotation.z = _x.pose.orientation.z;
	transform.rotation.w = _x.pose.orientation.w;

	geometry_msgs::msg::Twist velocity;
	velocity.linear.x = _xd.twist.linear.x;
	velocity.linear.y = _xd.twist.linear.y;
	velocity.linear.z = _xd.twist.linear.z;
	velocity.angular.x = 0.0;
	velocity.angular.y = 0.0;
	velocity.angular.z = 0.0;

	geometry_msgs::msg::Twist acceleration;
	acceleration.linear.x = _xdd.accel.linear.x;
	acceleration.linear.y = _xdd.accel.linear.y;
	acceleration.linear.z = _xdd.accel.linear.z;
	acceleration.angular.x = 0.0;
	acceleration.angular.y = 0.0;
	acceleration.angular.z = 0.0;

	if (isnanf(transform.translation.x) || isnanf(transform.translation.y) || isnanf(transform.translation.z))
		RCLCPP_INFO(this->get_logger(),"NAN in trajectory setpoint");

	trajectory_msgs::msg::MultiDOFJointTrajectoryPoint traj_pt;
	traj_pt.transforms.push_back(transform);
	traj_pt.velocities.push_back(velocity);
	traj_pt.accelerations.push_back(acceleration);
	traj_pt.time_from_start = rclcpp::Duration::from_seconds(0.0);

	_trajectory_setpoint_publisher->publish(traj_pt);
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

// void OffboardControl::start_wp_traj(std::shared_ptr<std::vector<POSE>> opt_poses, CARTESIAN_PLANNER & trajectory) {

// 	matrix::Vector3f sp;
// 	matrix::Vector3f prev_sp;
// 	matrix::Quaternionf prev_att_sp;
// 	matrix::Quaternionf att;
// 	float prev_yaw_sp;
// 	double duration;
// 	double yaw_time;
// 	float yaw_d;

// 	std::vector<geometry_msgs::msg::PoseStamped> poses;
// 	std::vector<double> times;
// 	geometry_msgs::msg::PoseStamped p;
// 	double t = 0.0f;

// 	prev_sp = _prev_sp;
// 	prev_att_sp = _prev_att_sp;
// 	prev_yaw_sp = matrix::Eulerf(prev_att_sp).psi();

// 	p.pose.position.x = prev_sp(0);
// 	p.pose.position.y = prev_sp(1);
// 	p.pose.position.z = prev_sp(2); 

// 	p.pose.orientation.w = prev_att_sp(0);
// 	p.pose.orientation.x = prev_att_sp(1);
// 	p.pose.orientation.y = prev_att_sp(2);
// 	p.pose.orientation.z = prev_att_sp(3);

// 	poses.push_back(p);
// 	times.push_back(t);

// 	for(int i = 1; i<int(opt_poses->size()); i++) {
// 		sp(0) = (*opt_poses)[i].position.x;
// 		sp(1) = (*opt_poses)[i].position.y;
// 		sp(2) = (*opt_poses)[i].position.z; 
				
// 		if(_do_transform){
// 			geometry_msgs::msg::PointStamped point_in, point_out;
// 			point_in.header.stamp = this->get_clock()->now();
// 			point_in.header.frame_id = _parent_transf;
// 			point_in.point.x = sp(0);
// 			point_in.point.y = sp(1);
// 			point_in.point.z = sp(2);
// 			try {
// 				tf2::doTransform(point_in, point_out, _tf_map_to_odom);
// 				sp(0) = point_out.point.x;
// 				sp(1) = point_out.point.y;
// 				sp(2) = point_out.point.z;
// 			} catch (const tf2::TransformException &ex) {
// 				RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
// 			}
// 		}

// 		yaw_d = atan2(sp(1)-prev_sp(1),sp(0)-prev_sp(0)); 
// 		yaw_d = std::isnan(yaw_d) ? prev_yaw_sp : yaw_d;
// 		att = matrix::Eulerf(0, 0, yaw_d);

// 		yaw_time = 1.0 + std::abs(prev_yaw_sp - yaw_d)/_max_yaw_rate;
// 		duration = 1.0 + std::sqrt(pow(sp(0) - prev_sp(0),2)+pow(sp(1) - prev_sp(1),2)+pow(sp(2) - prev_sp(2),2))/_max_velocity;


// 		p.pose.position.x = prev_sp(0);
// 		p.pose.position.y = prev_sp(1);
// 		p.pose.position.z = prev_sp(2); 

// 		p.pose.orientation.w = att(0);
// 		p.pose.orientation.x = att(1);
// 		p.pose.orientation.y = att(2);
// 		p.pose.orientation.z = att(3);
		
// 		t = t + yaw_time;
// 		poses.push_back(p);
// 		times.push_back(t);
		
// 		/* */
// 		p.pose.position.x = sp(0);
// 		p.pose.position.y = sp(1);
// 		p.pose.position.z = sp(2); 

// 		p.pose.orientation.w = att(0);
// 		p.pose.orientation.x = att(1);
// 		p.pose.orientation.y = att(2);
// 		p.pose.orientation.z = att(3);

// 		t = t + yaw_time + duration;
// 		poses.push_back(p);
// 		times.push_back(t);

// 		prev_sp = sp;
// 		prev_att_sp = att;

// 	}
// 	trajectory.set_waypoints(poses, times);

// 	trajectory.compute();

// 	_prev_sp = sp;
// 	_prev_att_sp = prev_att_sp;
// 	_prev_yaw_sp = matrix::Eulerf(prev_att_sp).psi();
// }

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

	// Se START_FROM_LAST_MEAS è attivo, usa sempre la posizione corrente
	if(START_FROM_LAST_MEAS){
		_prev_sp = _position;
		_prev_att_sp = _attitude;
		_prev_yaw_sp = matrix::Eulerf(_attitude).psi();
	}
	// Se _prev_sp non è valido, usa la posizione corrente
	else if(std::isnan(_prev_sp(0)) || std::isnan(_prev_sp(1)) || std::isnan(_prev_sp(2))) {
		RCLCPP_WARN(this->get_logger(), "Previous setpoint invalid, using current position");
		_prev_sp = _position;
		_prev_att_sp = _attitude;
		_prev_yaw_sp = matrix::Eulerf(_attitude).psi();
	}
	// Altrimenti usa _prev_sp esistente (già aggiornato dal teleop)

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

	// NON aggiorniamo _prev_sp qui - lo faremo quando la traiettoria sarà completata
	// _prev_sp = pos;
	// _prev_att_sp = att;
	// _prev_yaw_sp = matrix::Eulerf(att).psi();

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

	matrix::Vector3f prev_sp_map;
	matrix::Vector3f position;

	if(_do_transform){
		geometry_msgs::msg::PointStamped point_in, point_out;
		point_in.header.stamp = this->get_clock()->now();
		point_in.header.frame_id = _child_transf;
		point_in.point.x = _prev_sp(0);
		point_in.point.y = _prev_sp(1);
		point_in.point.z = _prev_sp(2);
		try {
			tf2::doTransform(point_in, point_out, _tf_odom_to_map);
			prev_sp_map(0) = point_out.point.x;
			prev_sp_map(1) = point_out.point.y;
			prev_sp_map(2) = point_out.point.z;
		} catch (const tf2::TransformException &ex) {
			RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
			return false; // If transform fails, we cannot proceed
		} 
		s.position.x = prev_sp_map(0); 
		s.position.y = prev_sp_map(1);
		s.position.z = prev_sp_map(2);

		point_in.point.x = _position(0);
		point_in.point.y = _position(1);
		point_in.point.z = _position(2); 

		try {
			tf2::doTransform(point_in, point_out, _tf_odom_to_map);
			position(0) = point_out.point.x;
			position(1) = point_out.point.y;
			position(2) = point_out.point.z;

		} catch (const tf2::TransformException &ex) {
			RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
			return false; // If transform fails, we cannot proceed
		} 
		
	}else {
		s.position.x = _prev_sp(0); 
		s.position.y = _prev_sp(1);
		s.position.z = _prev_sp(2);
		position = _position;
	}

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
	generated_path.header.frame_id = _parent_transf;

    _pp->set_start_state(s);
    _pp->set_goal_state(g);


    std::vector<POSE> nav_poses;
    //std::vector<POSE> opt_poses;


    double xbounds[2];
    double ybounds[2];
    double zbounds[2];
    double bz_min = 0.0; //( _w_p[2] < wp[2] ) ?   
    double bz_max = 0.0; 

	// Create bounds on the basis of actual position
	Eigen::Vector3f _w_p;
	_w_p << position(0), position(1), position(2);

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
    RCLCPP_INFO(get_logger(), "start check");
	usleep(0.01e6);

    visualization_msgs::msg::Marker check_m;
	// Set the frame, timestamp, and namespace
	check_m.header.frame_id = _check_frame_id;
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
	double step = 0.05;
	bool segment_checked = false;
	bool trajetory_is_completed = false;


	while( valid_path && !_stop_trajectory && *wp < poses.size() && !_wp_traj_completed && !_replan){	 // continue checking while executing
		RCLCPP_INFO(get_logger(), "nel while grosso");

		if(*wp != 0){
			RCLCPP_INFO(get_logger(), "nell'if *wp!=0");

			if(_do_transform){
				matrix::Vector3f position;
				geometry_msgs::msg::PointStamped point_in, point_out;
				point_in.header.stamp = this->get_clock()->now();
				point_in.header.frame_id = _child_transf;
				point_in.point.x = _position(0);
				point_in.point.y = _position(1);
				point_in.point.z = _position(2); 

				try {
					tf2::doTransform(point_in, point_out, _tf_odom_to_map);
					position(0) = point_out.point.x;
					position(1) = point_out.point.y;
					position(2) = point_out.point.z;
					
				} catch (const tf2::TransformException &ex) {
					RCLCPP_WARN(get_logger(), "Transform failed: %s", ex.what());
					break; // If transform fails, we cannot proceed
				} 
				pt_i << position(0), position(1), position(2);  // use position in map frame
				
			}else {
				
				pt_i << _position(0), _position(1), _position(2); 
			}
			
			RCLCPP_INFO(get_logger(), "START COLL CECK pose size= %d, wp= %d",poses.size(), *wp);

			for(int i=*wp ; i<poses.size(); i++ ) { // dovrebbe essere <= perchè se rrt trova un solo punto non fa coll check. con 2 segmenti lo ha fatto.(goal molto vicino)
				RCLCPP_INFO(get_logger(), "nel for wp"); // NON CI VA
				pt_f << poses[i].position.x, poses[i].position.y, poses[i].position.z;
				
				dir = (pt_f - pt_i);
				if (dir.norm() > 0.0) dir /= dir.norm();

				pt_check = pt_i;
				segment_checked = false;

    	        while( !segment_checked && valid_path) {
					// RCLCPP_INFO(get_logger(), "while !seg_check"); // NON CI VA se 1 pt
					pt_check += dir*step;
					check_m.pose.position.x = pt_check[0];
					check_m.pose.position.y = pt_check[1];
					check_m.pose.position.z = pt_check[2];
					s[0] = pt_check[0];
					s[1] = pt_check[1];
					s[2] = pt_check[2];

					valid_path = _pp->check_state(s);

					segment_checked = ((pt_check-pt_f).norm() < 2*step);
					if( _rviz_output > 0.0 ) {
						_check_path_pub->publish( check_m );
						// RCLCPP_INFO(get_logger(), "pulish mark"); // NON CI VA se 1 pt
					}
				}
				
				pt_i = pt_f;

				if(!valid_path) break;
				RCLCPP_INFO(get_logger(), "pose size= %d, wp= %d",poses.size(), *wp);
			}
			_wp_traj_completed = (!_trajectory.isReady() && *wp == poses.size()-1); 
			RCLCPP_INFO(get_logger(), "trajectory ended: %d, pose size= %d, wp= %d", _trajectory.isReady(), poses.size(), *wp);
		}
		
		if(_wp_traj_completed ){

			RCLCPP_INFO(get_logger(), "Checking path ENDED - Trajectory completed");
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

void OffboardControl::teleop_mode() {
	_teleop_active = true;
	
	// Initialize teleop position from current position
	_teleop_position = _position;
	_teleop_yaw = matrix::Eulerf(_attitude).psi();
	_teleop_velocity = matrix::Vector3f(0.0f, 0.0f, 0.0f);
	_teleop_yawspeed = 0.0f;
	
	RCLCPP_INFO(this->get_logger(), "Teleop mode active - use joystick to control the drone");
		
	// Integration loop for teleop velocities - solo integrazione, pubblicazione gestita da offboard_callback
	rclcpp::Rate rate(20); // 20 Hz per integrazione velocità
	
	while(_teleop_active && rclcpp::ok()) {
		// Integra velocità per ottenere posizione target
		float dt = 0.05f; // 20Hz
		
		_teleop_position += _teleop_velocity * dt;
		_teleop_yaw += _teleop_yawspeed * dt;
		
		// Normalizza yaw
		while (_teleop_yaw > M_PI) _teleop_yaw -= 2.0f * M_PI;
		while (_teleop_yaw < -M_PI) _teleop_yaw += 2.0f * M_PI;
		
		// Aggiorna continuamente il setpoint precedente con la posizione teleop corrente
		// Questo assicura che quando usciamo dal teleop, le nuove traiettorie partano dalla posizione corretta
		_prev_sp = _teleop_position;
		_prev_yaw_sp = _teleop_yaw;
		
		// Aggiorna anche l'attitude precedente basata sul nuovo yaw
		matrix::Quaternionf teleop_att(matrix::Eulerf(0, 0, _teleop_yaw));
		_prev_att_sp = teleop_att;
		
		rate.sleep();
		
		// Check for exit condition - quando arriva un nuovo comando o il nodo si ferma
		if (!rclcpp::ok()) {
			_teleop_active = false;
		}
	}
	
	// Quando usciamo dal teleop, assicuriamoci che i setpoint precedenti siano aggiornati
	// con l'ultima posizione raggiunta durante il teleop
	_prev_sp = _teleop_position;
	_prev_yaw_sp = _teleop_yaw;
	_prev_att_sp = matrix::Quaternionf(matrix::Eulerf(0, 0, _teleop_yaw));
	
	// Aggiorna anche le variabili del trajectory system per continuità
	_x.pose.position.x = _teleop_position(0);
	_x.pose.position.y = _teleop_position(1);
	_x.pose.position.z = _teleop_position(2);
	_x.pose.orientation.w = _prev_att_sp(0);
	_x.pose.orientation.x = _prev_att_sp(1);
	_x.pose.orientation.y = _prev_att_sp(2);
	_x.pose.orientation.z = _prev_att_sp(3);
	
	RCLCPP_INFO(this->get_logger(), "Exiting teleop mode");
}

void OffboardControl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
	// Attiva il flag joy_available al primo messaggio ricevuto
	if (!_joy_available) {
		_joy_available = true;
		RCLCPP_INFO(this->get_logger(), "Joy node detected - teleop functionality enabled");
	}
	
	if (!_teleop_active || !_first_odom) return;
	
	// Verifica che il joystick abbia abbastanza assi (almeno 4: 0,1,2,3)
	if (msg->axes.size() < 4) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
			"Joystick doesn't have enough axes (need at least 4, got %zu)", msg->axes.size());
		return;
	}
	
	// Calcolo velocità target dai comandi joystick - mapping corretto per il tuo joystick
	float vx = msg->axes[3] * _teleop_max_vel * 0.5f;     // axis 3 -> X velocity (ridotta)
	float vy = msg->axes[2] * _teleop_max_vel * 0.5f;     // axis 2 -> Y velocity (ridotta)  
	float vz = msg->axes[1] * _teleop_max_vel * 0.5f;     // axis 1 -> Z velocity (normale, ridotta)
	
	_teleop_velocity(0) = vx;
	_teleop_velocity(1) = vy;
	_teleop_velocity(2) = vz;
	
	// Yaw rate - axis 0 (velocità normale)
	_teleop_yawspeed = msg->axes[0] * _teleop_max_yaw_rate;
	
	// Debug info (throttled to avoid spam)
	RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
		"Joy axes: [0]=%.2f, [1]=%.2f, [2]=%.2f, [3]=%.2f -> vx=%.2f, vy=%.2f, vz=%.2f, yaw_rate=%.2f",
		msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3],
		vx, vy, vz, _teleop_yawspeed);
	
	// Deadband per evitare drift
	float deadband = 0.05f;
	if (std::abs(_teleop_velocity(0)) < deadband) _teleop_velocity(0) = 0.0f;
	if (std::abs(_teleop_velocity(1)) < deadband) _teleop_velocity(1) = 0.0f;
	if (std::abs(_teleop_velocity(2)) < deadband) _teleop_velocity(2) = 0.0f;
	if (std::abs(_teleop_yawspeed) < deadband) _teleop_yawspeed = 0.0f;
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

