#pragma once

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
// #include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <sensor_msgs/msg/joy.hpp>

// #include <Eigen/Matrix>
// #include <Eigen/Geometry>
#include <matrix/math.hpp>
#include "matrix/Matrix.hpp"
#include "matrix/Quaternion.hpp"
#include "matrix/Euler.hpp"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <string>
#include <vector>

#include <chrono>
#include <iostream>

#include "Trajectory.hpp"
#include "planner_spline.h"

#include "planner.h"

// ROS2 Lybraries
#include <octomap/octomap.h>
#include <nav_msgs/msg/path.hpp>
#include <octomap/AbstractOcTree.h>
#include "octomap_msgs/conversions.h"
#include <octomap_msgs/msg/octomap.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

// Custom messages
#include "trajectory_planner/msg/move_cmd.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

enum NodeState {
	IDLE = 0,
	STOPPED = 1,
	REPLAN = 2,
	SP_ENDED = 3,
	WP_ENDED = 4,
};


class OffboardControl : public rclcpp::Node {
public:
	OffboardControl();

	/**
	 * @brief Send a command to Arm the vehicle
	 */
	void arm();
	/**
	 * @brief Send a command to Disarm the vehicle
	 */
	void disarm();

	void setState(NodeState state) {_state = state;}

	void key_input();

	void move_cmd();

	void flight_termination(float val);

	// Teleop methods
	void teleop_mode();
	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);




private:

	void offboard_callback();
	void status_update();
	// void status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg); 
	void move_command_callback(const trajectory_planner::msg::MoveCmd::SharedPtr msg);
	void octomap_callback( const octomap_msgs::msg::Octomap::SharedPtr octo_msg );
	void check_path(const std::vector<POSE> & poses, const std::shared_ptr<int> wp );
	void tf_lookup_loop();
	bool _first_odom{false};

	rclcpp::TimerBase::SharedPtr _timer;
	rclcpp::TimerBase::SharedPtr _check_timer;
	rclcpp::TimerBase::SharedPtr _status_timer;
	milliseconds _timer_period{20ms};
	milliseconds _status_timer_period{200ms};
	
	float _timer_freq{100.0f};

	CARTESIAN_PLANNER _trajectory{_timer_freq};
	void stop_traj();
	void compute_time_and_heading(const matrix::Vector3f & sp, float & yaw_d, float & yaw_time, float & duration);
	void start_traj(matrix::Vector3f pos, float yaw, double d);
	// void start_wp_traj(std::shared_ptr<std::vector<POSE>> opt_poses, CARTESIAN_PLANNER & trajectory);
	bool plan(Eigen::Vector3d wp, std::shared_ptr<std::vector<POSE>> opt_poses);

	rclcpp::Publisher<OffboardControlMode>::SharedPtr _offboard_control_mode_publisher;
	rclcpp::Publisher<VehicleCommand>::SharedPtr _vehicle_command_publisher;
	rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr _trajectory_setpoint_publisher;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_publisher;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _check_path_pub;
	
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _plan_state_publisher;

	// rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _px4_vehicle_status_sub;
	rclcpp::Subscription<trajectory_planner::msg::MoveCmd>::SharedPtr _cmd_sub;
	rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr _octo_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _land_detect_sub;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_sub;


	std::atomic<uint64_t> _timestamp;   //!< common synced timestamped

	matrix::Quaternionf _attitude{};
	matrix::Vector3f _position{};
	matrix::Vector3f _last_pos_sp{};
	matrix::Quaternionf _last_att_sp{};

	std::vector<double> _traj_points;
	bool _traj_present = false;

	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	geometry_msgs::msg::TransformStamped _tf_map_to_odom, _tf_odom_to_map;

	uint64_t _offboard_setpoint_counter;   //!< counter for the number of setpoints sent

	// /**
	//  * @brief Publish the offboard control mode.
	//  *        For this example, only position and altitude controls are active.
	//  */
	// void publish_offboard_control_mode();
	// /**
	//  * @brief Publish a trajectory setpoint
	//  *        For this example, it sends a trajectory setpoint to make the
	//  *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
	//  */
	// void publish_trajectory_setpoint();
	/**
	 * @brief Publish vehicle commands
	 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
	 * @param param1    Command parameter 1
	 * @param param2    Command parameter 2
	 */
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

	void publish_trajectory_setpoint();
	

	NodeState _state;
	matrix::Vector3f _goal{};
	matrix::Vector3f _starting_point{};
	float _starting_yaw{};
	matrix::Vector3f _prev_sp{};
	matrix::Vector3f _stop_sp{};
	matrix::Quaternionf _stop_att_sp{};
	matrix::Quaternionf _prev_att_sp{};
	float _prev_yaw_sp;

	geometry_msgs::msg::PoseStamped _x;
	geometry_msgs::msg::TwistStamped _xd;
	geometry_msgs::msg::AccelStamped _xdd;

	PATH_PLANNER *_pp;
	double _xbounds[2];
	double _ybounds[2];
	double _zbounds[2];
	double _robot_radius, _x_valid_min, _x_valid_max, _y_valid_min, _y_valid_max, _z_motion_threshold;

	double _max_yaw_rate, _max_velocity;

	matrix::Matrix3f _T_enu_to_ned;
	bool _map_set;
	double _use_octomap, _rviz_output, _dist_from_th_error;
	int _replan_cnt;

	bool _stop_trajectory{false}, _plan_is_valid{true}, _wp_traj_completed{false};
	double _do_transform;
	double _use_key_input;
	bool _is_flying{false};

	std::string _cmd="";
	std::string _last_cmd="";
	matrix::Vector3f _cmd_sp;
	bool _new_cmd{false};
	bool _replan{false};
	bool _plan_has_result{true};
	bool _armed{false};
	bool _new_command{false};
	int _max_replan_iterations{5};
	std::string _status="IDLE";

	std::string _offboard_control_mode_topic;
	std::string _vehicle_command_topic;
	std::string _trajectory_setpoint_topic;
	std::string _odom_topic;
	std::string _octomap_topic;
	std::string _move_cmd_topic;
	std::string _path_topic;
	std::string _check_path_topic;
	std::string _plan_status_topic;

	std::string _parent_transf;
	std::string _child_transf;
	std::string _check_frame_id;

	// Teleop variables
	bool _teleop_active{false};
	bool _joy_available{false};  // Flag di sicurezza - true solo se joy_node è disponibile
	matrix::Vector3f _teleop_velocity{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _teleop_position{0.0f, 0.0f, 0.0f};
	float _teleop_yaw{0.0f};
	float _teleop_yawspeed{0.0f};
	float _teleop_max_vel{1.0f};
	float _teleop_max_yaw_rate{1.0f};
	
	// Joy axis mapping (default Xbox controller)
	int _axis_linear_x{1};   // Left stick vertical
	int _axis_linear_y{0};   // Left stick horizontal  
	int _axis_linear_z{4};   // Right stick vertical
	int _axis_angular_z{3};  // Right stick horizontal
	int _button_enable{4};   // LB button
	int _button_arm{0};      // A button
	int _button_takeoff{3};  // Y button
	int _button_land{1};     // B button
};
