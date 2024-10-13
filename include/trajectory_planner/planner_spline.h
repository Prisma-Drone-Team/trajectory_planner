#include "rclcpp/rclcpp.hpp"
#include "boost/thread.hpp"
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>
// #include <tf/tf.h>
// #include <tf_conversions/tf_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2/convert.h>
// #include <tf2_eigen/tf2_eigen_h>

bool T_RPY(Eigen::Matrix3d &R, double phi, double theta, double psi);
bool Tdot_RPY(Eigen::Matrix3d &R, double phi, double theta, double psi);
void twist2Vector(const geometry_msgs::msg::TwistStamped twist, Eigen::VectorXd &vel);
void accel2Vector(const geometry_msgs::msg::AccelStamped acc, Eigen::VectorXd &a);
void wrench2Vector(const geometry_msgs::msg::WrenchStamped wrench, Eigen::VectorXd &w);
Eigen::Matrix3d Skew(Eigen::Vector3d v);
Eigen::Vector3d Vee(Eigen::Matrix3d S);

class SPLINE_PLANNER
{
public:
	SPLINE_PLANNER(double freq);
	void compute_traj();
	void set_waypoints(std::vector<double> points, std::vector<double> times, double xdi = 0, double xdf = 0, double xddi = 0, double xddf = 0);
	bool isReady() { return _ready; };
	bool getNext(double &x, double &xd, double &xdd);
	
	
	std::vector<double> _x;
	std::vector<double> _xd;
	std::vector<double> _xdd;
	std::vector<double> _t;

private:
	std::vector<double> _points;
	std::vector<double> _times;
	double _freq;
	int _N;
	bool _ready;
	int _counter;
	double _xdi, _xdf, _xddi, _xddf;
};

class CARTESIAN_PLANNER
{

public:
	CARTESIAN_PLANNER(double freq) : xplanner(freq), yplanner(freq), zplanner(freq), aplanner(freq)
	{
		_ready = false;
		_counter = 0;
		_xdi.resize(6);
		_xdf.resize(6);
		_xddi.resize(6);
		_xddf.resize(6);
		_last_x.pose.position.x = _last_x.pose.position.y = _last_x.pose.position.z = 0.0f;
		_last_x.pose.orientation.w = _last_x.pose.orientation.x = _last_x.pose.orientation.y = _last_x.pose.orientation.z = 0.0f;
		_last_xd.twist.linear.x = _last_xd.twist.linear.y = _last_xd.twist.linear.z = 0.0f;
		_last_xd.twist.angular.x = _last_xd.twist.angular.y = _last_xd.twist.angular.z = 0.0f;
		_last_xdd.accel.linear.x = _last_xdd.accel.linear.y = _last_xdd.accel.linear.z = 0.0f;
		_last_xdd.accel.angular.x = _last_xdd.accel.angular.y = _last_xdd.accel.angular.z = 0.0f;
	};
	void compute();
	void clear_waypoints();
	void set_waypoints(std::vector<geometry_msgs::msg::PoseStamped> poses, std::vector<double> times);
	void set_waypoints(std::vector<geometry_msgs::msg::PoseStamped> poses, std::vector<double> times, Eigen::VectorXd xdi, Eigen::VectorXd xdf, Eigen::VectorXd xddi, Eigen::VectorXd xddf);
	bool isReady() { return _ready; };
	int getCounter(){return _counter;};
	bool getNext(geometry_msgs::msg::PoseStamped &x, geometry_msgs::msg::TwistStamped &xd, geometry_msgs::msg::AccelStamped &xdd);
	std::vector<geometry_msgs::msg::PoseStamped> _x;
	std::vector<geometry_msgs::msg::TwistStamped> _xd;
	std::vector<geometry_msgs::msg::AccelStamped> _xdd;

	geometry_msgs::msg::PoseStamped _last_x;
	geometry_msgs::msg::TwistStamped _last_xd;
	geometry_msgs::msg::AccelStamped _last_xdd;

	std::vector<double> _t;

private:
	void R_axisAngle(double th, Eigen::Vector3d r, Eigen::Matrix3d &R);
	std::vector<geometry_msgs::msg::PoseStamped> _poses;
	std::vector<double> _times;
	SPLINE_PLANNER xplanner, yplanner, zplanner, aplanner;
	int _N;
	bool _ready;
	int _counter;
	Eigen::VectorXd _xdi, _xdf, _xddi, _xddf;
};
