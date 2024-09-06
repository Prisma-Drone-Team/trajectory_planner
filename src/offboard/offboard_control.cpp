#include "offboard_control.hpp"

#include <cmath>

// TODO -----------------------------------------------------------------------------------------
/*
	1. checker thread ( IDEA: add  _plan_is_valid flag to evluate)
	2. manager node (publish a string with "nav" command "land" etc. )
	3. add a way to control also yaw (no fixed heading)
	5. octomap integration (subscription etc.)

*/
// END - TODO -----------------------------------------------------------------------------------

OffboardControl::OffboardControl() : rclcpp::Node("offboard_control"), _state(STOPPED) {

	_offboard_control_mode_publisher =
		this->create_publisher<OffboardControlMode>("fmu/in/offboard_control_mode", 10);
	_trajectory_setpoint_publisher =
		this->create_publisher<TrajectorySetpoint>("fmu/in/trajectory_setpoint", 10);
	_vehicle_command_publisher =
		this->create_publisher<VehicleCommand>("fmu/in/vehicle_command", 10);


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
					_prev_yaw_sp = matrix::Eulerf(_attitude).psi();
				}
			});

	_offboard_setpoint_counter = 0;


	_timer = this->create_wall_timer(_timer_period, std::bind(&OffboardControl::timer_callback, this));

	boost::thread key_input_t( &OffboardControl::key_input, this );

	//---Init planner
	this->declare_parameter("max_yaw_rate", .5);
	_max_yaw_rate = this->get_parameter("max_yaw_rate").as_double();
	RCLCPP_INFO(get_logger(), "max_yaw_rate: %f", _max_yaw_rate);
	this->declare_parameter("max_velocity", .5);
	_max_velocity = this->get_parameter("max_velocity").as_double();
	RCLCPP_INFO(get_logger(), "max_velocity: %f", _max_velocity);

	
	this->declare_parameter("robot_radius",0.1);
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

    _pp = new PATH_PLANNER();
    _pp->init( _xbounds, _ybounds, _zbounds);
    _pp->set_robot_geometry(_robot_radius);
    //---
}

void OffboardControl::timer_callback() {
	if(!_first_odom)
		return;

	if(!_first_traj){
		firstTraj();
		_first_traj = true;
	}

	if (_offboard_setpoint_counter == 10) {
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
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

void OffboardControl::key_input() {
	bool exit = false;
	std::string cmd;
	matrix::Vector3f sp;
	double duration;
	float  yaw_d;
	while(!exit && rclcpp::ok()) {
		std::cout << "Enter command [arm | go | takeoff | land | stop | nav | term]: \n"; 
		std::cin >> cmd;
		if(cmd == "go") {
			std::cout << "Enter X coordinate: "; 
			std::cin >> sp(0);
			std::cout << "Enter Y coordinate: "; 
			std::cin >> sp(1);
			std::cout << "Enter Z coordinate: "; 
			std::cin >> sp(2);
			// std::cout << "Enter yaw: "; 
			// std::cin >> yaw;
			yaw_d = atan2(sp(1)-_prev_sp(1),sp(0)-_prev_sp(0)); 
			// std::cout << "Enter duration: "; 
			// std::cin >> duration;

			double yaw_time = std::abs(_prev_yaw_sp - yaw_d)/_max_yaw_rate;
			double duration = std::sqrt(pow(sp(0) - _prev_sp(0),2)+pow(sp(1) - _prev_sp(1),2)+pow(sp(2) - _prev_sp(2),2))/_max_velocity;

			startTraj(_prev_sp, yaw_d, yaw_time); 
			startTraj(sp, yaw_d, duration);

			_prev_sp = sp;
			_prev_yaw_sp = yaw_d;

		}
		else if(cmd=="nav"){
			Eigen::Vector3d wp;
			std::cout << "Enter X coordinate: "; 
			std::cin >> wp[0];
			std::cout << "Enter Y coordinate: "; 
			std::cin >> wp[1];
			std::cout << "Enter Z coordinate: "; 
			std::cin >> wp[2];

			//std::vector<POSE>* opt_poses;
			auto opt_poses = std::make_shared<std::vector<POSE>>();
			plan(wp,opt_poses);

			for(int i = 0; i<int(opt_poses->size()); i++) {
				matrix::Vector3f sp((*opt_poses)[i].position.x, (*opt_poses)[i].position.y, -(*opt_poses)[i].position.z);    // TODO proper conversion
				yaw_d = atan2(sp(1)-_prev_sp(1),sp(0)-_prev_sp(0)); 
				double yaw_time = std::abs(_prev_yaw_sp - yaw_d)/_max_yaw_rate;
				double duration = std::sqrt(pow(sp(0) - _prev_sp(0),2)+pow(sp(1) - _prev_sp(1),2)+pow(sp(2) - _prev_sp(2),2))/_max_velocity;

				startTraj(_prev_sp, yaw_d, yaw_time); 
				startTraj(sp, yaw_d, duration);

				_prev_sp = sp;
				_prev_yaw_sp = yaw_d;
			}

		}
		// else if(cmd == "traj") {
		// 	if(!_traj_present) {
		// 		std::cout << "Trajectory not loaded correctly!\n";
		// 		continue;
		// 	}
		// 	for(int i = 0; i<int(_traj_points.size()); i+=7) {
		// 		matrix::Vector3f sp(_traj_points[i], _traj_points[i+1], _traj_points[i+2]);
		// 		float yaw   = _traj_points[i+3];
		// 		double duration = _traj_points[i+4];
		// 		startTraj(sp, yaw, duration);
		// 	}
		// }
		else if(cmd == "takeoff") {
			float alt;
			std::cout << "Enter altitude: "; 
			std::cin >> alt;
			this->arm();
			alt = alt > 0 ? -alt : alt;
			takeoffTraj(alt);                   // TODO check time logic 
			_prev_sp(2) = alt;                    
		}
		else if(cmd == "land") {
			std::cout << "Landing procedure triggered... \nRemember to kill disarm manually after landed.\n";
			_prev_sp(2) = 0.5; 
			startTraj(_prev_sp, yaw_d, 15);	 // TODO tune time
		}
		else if(cmd == "stop") {
			exit = true;
			rclcpp::shutdown();
		}
		else if(cmd == "arm") {
			_first_traj = false;
			this->flight_termination(0);
			this->arm();
		}
		else if(cmd == "term") {
			this->flight_termination(1);		 // TODO unire term a land (check odometry feedback)
		}
		else {
			std::cout << "Unknown command;\n";
		}

	}

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
	msg.attitude = false;
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
	msg.yawspeed = 0.0f;

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
	
	std::cout << "Sending command\n";

	_vehicle_command_publisher->publish(msg);
}

void OffboardControl::firstTraj() {
	std::vector<geometry_msgs::msg::PoseStamped> poses;
	std::vector<double> times;
	geometry_msgs::msg::PoseStamped p;
	double t;

	_last_pos_sp = _position;
	_last_att_sp = _attitude;

	p.pose.position.x = _last_pos_sp(0);
	p.pose.position.y = _last_pos_sp(1);
	p.pose.position.z = _last_pos_sp(2); 

	p.pose.orientation.w = _last_att_sp(0);
	p.pose.orientation.x = _last_att_sp(1);
	p.pose.orientation.y = _last_att_sp(2);
	p.pose.orientation.z = _last_att_sp(3);
	t = 0.0f;

	poses.push_back(p);
	times.push_back(t);
	
	poses.push_back(p);
	times.push_back(0.1);

	_trajectory.set_waypoints(poses, times);
	_trajectory.compute();

}

void OffboardControl::takeoffTraj(float alt) {

	while(_trajectory.isReady()) {
		usleep(0.1e6);
	}

	std::vector<geometry_msgs::msg::PoseStamped> poses;
	std::vector<double> times;
	geometry_msgs::msg::PoseStamped p;
	double t;

	p.pose.position.x = _last_pos_sp(0);
	p.pose.position.y = _last_pos_sp(1);
	p.pose.position.z = _last_pos_sp(2); 

	p.pose.orientation.w = _last_att_sp(0);
	p.pose.orientation.x = _last_att_sp(1);
	p.pose.orientation.y = _last_att_sp(2);
	p.pose.orientation.z = _last_att_sp(3);
	t = 0.0f;

	poses.push_back(p);
	times.push_back(t);
	
	
	p.pose.position.z = alt; 
	poses.push_back(p);
	times.push_back(3.0f * abs(alt));

	_last_pos_sp(2) = alt;

	_trajectory.set_waypoints(poses, times);
	_trajectory.compute();
	std::cout << "Takeoff trajectory set\n";
}

void OffboardControl::startTraj(matrix::Vector3f pos, float yaw, double d) {

	while(_trajectory.isReady()) {
		usleep(0.1e6);
	}
	
	std::vector<geometry_msgs::msg::PoseStamped> poses;
	std::vector<double> times;
	geometry_msgs::msg::PoseStamped p;
	double t;

	// if(pos(2) > 0.0f)
	//     pos(2) *= -1;

	matrix::Quaternionf att(matrix::Eulerf(0, 0, yaw));

	/* */
	p.pose.position.x = _last_pos_sp(0);
	p.pose.position.y = _last_pos_sp(1);
	p.pose.position.z = _last_pos_sp(2); 

	p.pose.orientation.w = _last_att_sp(0);
	p.pose.orientation.x = _last_att_sp(1);
	p.pose.orientation.y = _last_att_sp(2);
	p.pose.orientation.z = _last_att_sp(3);

	t = 0.0;
	
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

	t = d;

	_last_pos_sp = pos;
	_last_att_sp = att;

	poses.push_back(p);
	times.push_back(t);

	// auto start = steady_clock::now();

	_trajectory.set_waypoints(poses, times);

	_trajectory.compute();

	// auto end = steady_clock::now();
}


void OffboardControl::plan(Eigen::Vector3d wp, std::shared_ptr<std::vector<POSE>> opt_poses) {

    // if( _use_octomap ) {
    //     while(!_map_set) usleep(0.1*1e6);
    // }

    POSE s;
    POSE g;
    
    s.position.x = _last_pos_sp(0); 
    s.position.y = _last_pos_sp(1);
    s.position.z = -_last_pos_sp(2);
    s.orientation.w = 1.0;
    s.orientation.x = 0.0;
    s.orientation.y = 0.0;
    s.orientation.z = 0.0;

    g.position.x = wp[0];
    g.position.y = wp[1];
    g.position.z = wp[2];
    g.orientation.w = 1.0;
    g.orientation.x = 0.0;
    g.orientation.y = 0.0;
    g.orientation.z = 0.0;


    // geometry_msgs::PoseStamped p;
    // p.header.frame_id = "drone/map";

    // Eigen::Vector3d lp;
    // Eigen::Vector3d gp;

    // nav_msgs::Path generated_path_opt;
    // generated_path_opt.header.frame_id = "drone/map";


    // nav_msgs::Path generated_path;
    // generated_path.header.frame_id = "drone/map";

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
    zbounds[0] = bz_min - _z_motion_threshold; //param?

    xbounds[1] = _x_valid_max;
    ybounds[1] = _y_valid_max;
    zbounds[1] = bz_max + _z_motion_threshold; 

    int ret = _pp->plan(2, xbounds, ybounds, zbounds, nav_poses, *opt_poses);

	if( ret < 0 || nav_poses.size() < 2 ) 
            std::cout << "Planner not correctly initialized" << std::endl;
	else {

		std::cout << "Solution: " << std::endl;
		
		for(int i=0; i<nav_poses.size(); i++ ) {
			std::cout << "Pose: [" << i << "]: " << "(" << nav_poses[i].position.x << " " << nav_poses[i].position.y << " " << nav_poses[i].position.z << ")" << std::endl;
		}
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
