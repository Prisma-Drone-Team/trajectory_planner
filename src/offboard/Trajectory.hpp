#pragma once

#include "matrix/Matrix.hpp"
#include "matrix/Quaternion.hpp"
#include "matrix/Euler.hpp"

class Trajectory {

public:
	Trajectory() = default;

	void setStartingPoint( const matrix::Vector3f &p, const float yaw );
	void setGoal( const matrix::Vector3f &p );
	void setDuration( const float d ) {_duration = d;};
	void setMaxAcc( const float a ) {_maxAcc = a;};
	void setMaxVel( const float v ) {_maxVel = v;};
	void setYawVel( const float v ) {_yawVel = v;};
	void start();
	void getNextPoint(const float dt, matrix::Vector3f &p, matrix::Vector3f &v, matrix::Vector3f &a, float &yaw);

private:
	matrix::Vector3f _startingPoint{};
	float _startingYaw{};
	matrix::Vector3f _goal{};
	float _goalYaw{};
	float _yawDiff{};
	float _duration{10.0f};
	float _arcLength{0.0f};
	float _maxVel{5.0f};
	float _maxAcc{2.0f};
	float _maxJerk{0.5f};
	float _yawVel{0.5f};
	float _trajVel;
	float _trajAcc;
	float _trajJerk;
	float _s1, _s2, _s3;
	matrix::Vector3f _currentPos{};
	matrix::Vector3f _currentVel{};
	matrix::Vector3f _currentAcc{};
	float _currentYaw{};
	float _currentTime{0.0f};

	bool _arrived{false};

	float _a_t;
	float _s{0.0f};
	float _s_dot{0.0f};
	float _s_ddot{0.0f};

};