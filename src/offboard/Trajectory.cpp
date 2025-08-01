#include "Trajectory.hpp"

#include <iostream>

#define _USE_MATH_DEFINES
 
#include <cmath>

using namespace matrix;

void Trajectory::start() {
	_currentTime = 0.0f;
	_currentPos = _startingPoint;
	_currentYaw = _startingYaw;
	_currentVel = Vector3f(0, 0, 0);
	_currentAcc = Vector3f(0, 0, 0);
	_arcLength = (_goal-_startingPoint).norm();
	_trajVel = _maxVel;
	_trajAcc = _maxAcc;
	_trajJerk = _maxJerk;

	_yawDiff = _goalYaw - _startingYaw;
	if( _yawDiff > M_PI )
		_yawDiff -= 2.0f * M_PI;
	else if( _yawDiff < -M_PI )
		_yawDiff += 2.0f * M_PI;
	_goalYaw = _startingYaw + _yawDiff;

	if(_arcLength < 0.1f) {
		_arrived = true;
		return;
	}

	_s = 0.0f;
	_s_dot = 0.0f;
	_s_ddot = 0.0f;

	float t1 = _maxAcc / _maxJerk;
	float t_am = _maxVel / _maxAcc - t1;

	float v1 = 0.5f * _maxJerk * powf(t1, 2);
	float v2 = v1 + _maxAcc * t_am;

	_s1 = _maxJerk * powf(t1, 3) / 6.0f;
	_s2 = _s1 + v1 * t_am + 0.5f * _maxAcc * powf(t_am, 2);
	_s3 = _s2 + v2 * t1 + 0.5f * _maxAcc * powf(t1, 2) - _maxJerk * powf(t1, 3) / 6.0f;

	float c1 = 0.5f * _arcLength - _s1;
	float c2 = c1 - 0.5f * _maxAcc * powf(t1,2) + _maxJerk*powf(t1,3)/6.0f;
	float c3 = v1*t1 - c2;
	
	if(_arcLength < 2.0f * _s3) { // The trapezoids intersect
		float s_min = _s1 + v1 * t1 + 0.5f * _maxAcc * powf(t1, 2) - _maxJerk * powf(t1, 3) / 6.0f;
		if(_arcLength / 2.0f < s_min) {
			// I cannot reach _maxAcc
			// std::cout << "I cannot reach maxAcc\n";
			t1 = cbrt(0.5f * _arcLength / _maxJerk);
			t_am = 0.0f;
			_s1 = _maxJerk * powf(t1, 3) / 6.0f;
			_s2 = _s1;
			_s3 = _arcLength / 2.0f;
		}
		else {
			// I cannot reach _maxVel
			// std::cout << "I cannot reach maxVel\n";
			float delta = powf(v1+_maxAcc*t1, 2) - 2.0f * _maxAcc * c3;
			t_am = (-(v1+_maxAcc*t1) + sqrt(delta)) / _maxAcc;
			if(t_am < 0.0f) {
				std::cout << "Error: t_am < 0\n";
				t_am = 0.0f;
			}
			v1 = 0.5f * _maxJerk * powf(t1, 2);
			v2 = v1 + _maxAcc * t_am;
			_s1 = _maxJerk * powf(t1, 3) / 6.0f;
			_s2 = _s1 + v1 * t_am + 0.5f * _maxAcc * powf(t_am, 2);
			_s3 = _s2 + v2 * t1 + 0.5f * _maxAcc * powf(t1, 2) - _maxJerk * powf(t1, 3) / 6.0f;
		}
	}	

	// std::cout << "t1: " << t1 << std::endl;
	// std::cout << "t_am: " << t_am << std::endl;

	// std::cout << "S1: " << _s1 << std::endl;
	// std::cout << "S2: " << _s2 << std::endl;
	// std::cout << "S3: " << _s3 << std::endl;
	// std::cout << "arcLength: " << _arcLength << std::endl;
	// std::cout << "Duration: " << _duration << std::endl;
	// std::cout << "Maximum Acceleration: " << _maxAcc << std::endl;
	// std::cout << "Maximum Velocity: " << _maxVel << std::endl;

	_arrived = false;
}

void Trajectory::getNextPoint(const float dt, Vector3f &p, Vector3f &v, Vector3f &a, float &yaw) {

	static float sf=0, sdotf=0, sddotf=0;
	static float tf=0;

	float deltaTime = 0;

	if(abs(_currentYaw - _goalYaw) > _yawVel * 0.05f) {
		_currentYaw += _yawDiff > 0.0f ? _yawVel * 0.05f : -_yawVel * 0.05f;
		if( _currentYaw > 2.0f * M_PI )
			_currentYaw -= 2.0f * M_PI;
		else if( _currentYaw < -2.0f * M_PI )
			_currentYaw += 2.0f * M_PI;
		yaw = _currentYaw;
	}
	else{
		yaw = _goalYaw;
		_startingYaw = _goalYaw;
	}
	// std::cout << "Yaw : " << yaw << std::endl;
	Vector3f dir = (_goal-_startingPoint).normalized();

	if(_arrived) {
		p = _goal;
		v = Vector3f(0, 0, 0);
		a = Vector3f(0, 0, 0);
		sf = sdotf = sddotf = 0;
		tf = 0;
		return;
	}

	_currentTime += dt;

	deltaTime = _currentTime - tf;

	if ( _s < _s1 ) { // Constant Jerk positive
		_s_ddot = _trajJerk * _currentTime;
		_s_dot = 0.5f * _trajJerk * powf(_currentTime, 2);
		_s = _trajJerk * powf(_currentTime, 3) / 6.0f;
		if( _s >= _s1 ) {
			// std::cout << "End of constant jerk positive" << std::endl;
			sf = _s;
			sdotf = _s_dot;
			sddotf = _s_ddot;
			tf = _currentTime;
		}
	}
	else if( _s < _s2 ) { // Constant Acceleration positive
		//_s_ddot = sddotf;
		_s_dot = sdotf + sddotf * deltaTime;
		_s = sf + sdotf * deltaTime + 0.5 * sddotf * powf(deltaTime, 2);
		if( _s >= _s2 ) {
			// std::cout << "End of constant acceleration positive" << std::endl;
			sf = _s;
			sdotf = _s_dot;
			sddotf = _s_ddot;
			tf = _currentTime;
		}
	}
	else if( _s < _s3 ) { // Constant Jerk negative
		_s_ddot = sddotf - _trajJerk * deltaTime;
		_s_dot = sdotf + sddotf * deltaTime - 0.5f * _trajJerk * powf(deltaTime, 2);
		_s = sf + sdotf*deltaTime + 0.5f * sddotf * powf(deltaTime, 2) - _trajJerk * powf(deltaTime, 3) / 6.0f;
		if( _s >= _s3 ) {
			// std::cout << "End of constant jerk negative" << std::endl;
			sf = _s;
			sdotf = _s_dot;
			sddotf = _s_ddot;
			tf = _currentTime;
		}
	}
	else if ( _s < _arcLength - _s3 ) { // Constant Velocity
		_s_ddot = 0.0f;
		// _s_dot = 0.0f;
		_s = sf + _s_dot * deltaTime;
		if( _s >= _arcLength - _s3 ) {
			// std::cout << "End of constant velocity" << std::endl;
			sf = _s;
			sdotf = _s_dot;
			sddotf = _s_ddot;
			tf = _currentTime;
		}
	}
	else if( _s < _arcLength - _s2 ) { // Constant Jerk negative
		_s_ddot = - _trajJerk * deltaTime;
		_s_dot = sdotf - 0.5f * _trajJerk * powf(deltaTime, 2);
		_s = sf + sdotf*deltaTime - _trajJerk * powf(deltaTime, 3) / 6.0f;
		if( _s >= _arcLength - _s2 ) {
			// std::cout << "End of constant jerk negative" << std::endl;
			sf = _s;
			sdotf = _s_dot;
			sddotf = _s_ddot;
			tf = _currentTime;
		}
	}
	else if( _s < _arcLength - _s1 ) { // Constant Acceleration negative
		//_s_ddot = sddotf;
		_s_dot = sdotf + sddotf * deltaTime;
		_s = sf + sdotf * deltaTime + 0.5 * sddotf * powf(deltaTime, 2);
		if( _s >= _arcLength - _s1 ) {
			// std::cout << "End of constant acceleration negative" << std::endl;
			sf = _s;
			sdotf = _s_dot;
			sddotf = _s_ddot;
			tf = _currentTime;
		}
	}
	else if( _s < _arcLength ) { // Constant Jerk positive
		_s_ddot = sddotf + _trajJerk * deltaTime;
		_s_dot = sdotf + sddotf * deltaTime + 0.5f * _trajJerk * powf(deltaTime, 2);
		_s = sf + sdotf*deltaTime + 0.5f * sddotf * powf(deltaTime, 2) + _trajJerk * powf(deltaTime, 3) / 6.0f;
		if( _s >= _arcLength ) {
			std::cout << "End of Trajectory" << std::endl;
			sf = _s;
			sdotf = _s_dot;
			sddotf = _s_ddot;
			tf = _currentTime;
		}
	}
	else {
		_s_ddot = 0.0f;
		_s_dot = 0.0f;
		// _s = _arcLength;
		_goal = _startingPoint + dir * _s;
		_arrived = true;
	}

	p = _startingPoint + dir * _s;
	v = dir * _s_dot;
	a = dir * _s_ddot;

}

void Trajectory::setStartingPoint( const Vector3f &p, const float yaw ) {
	_startingPoint = p;
	_startingYaw = yaw;
	if(_startingYaw < 0.0f)
			_startingYaw += 2.0f*M_PI;
	std::cout << "Starting point:\n" << 
		_startingPoint(0) << " " << _startingPoint(1) << " " << _startingPoint(2) << std::endl;
	std::cout << "Starting yaw: " << _startingYaw << std::endl;
}

void Trajectory::setGoal( const Vector3f &p ) {
	_goal = p;
	Vector2f s(_goal(0) - _startingPoint(0), _goal(1) - _startingPoint(1));
	if(s.norm() > 1.0f) {
		_goalYaw = atan2(s(1), s(0));
		if(_goalYaw < 0.0f)
			_goalYaw += 2.0f*M_PI;
	}
	else
		_goalYaw = _startingYaw;
	std::cout << "Goal point:" <<
		 _goal(0) << " " << _goal(1) << " " << _goal(2) << std::endl;
	std::cout << "Goal yaw: " << _goalYaw << std::endl;
}