#include "Trajectory.hpp"

#include <iostream>

using namespace Eigen;

void Trajectory::start() {
	_currentTime = 0.0f;
	_currentPos = _startingPoint;
	_currentVel = Vector3f::Zero();
	_currentAcc = Vector3f::Zero();
	_arcLength = (_goal-_startingPoint).norm();
	_trajAcc = _maxAcc;
	_trajVel = _maxVel;

	if(_arcLength < 0.1f) {
		_arrived = true;
		return;
	}

	_s = 0.0f;
	_s_dot = 0.0f;
	_s_ddot = 0.0f;

	float a1 = pow(_maxAcc*_duration, 2); //25
	float a2 = 4*_maxAcc*_arcLength; //20

	if( a1 < a2 ) {
		std::cout << "a1 < a2" << std::endl;
		_duration = 2 * sqrt(_arcLength / _maxAcc) * 1.1f;
	}

	_a_t = (_maxAcc * _duration - sqrt(pow(_maxAcc*_duration, 2)-4*_maxAcc*_arcLength)) \
					/ (2*_maxAcc); //1.38

	if( _maxAcc * _a_t > _maxVel) {
		std::cout << "vel > maxVel" << std::endl;
		_a_t = _maxVel / _maxAcc;
		float diff = _arcLength - _maxAcc * pow(_a_t, 2);
		_duration = 2 * _a_t + diff / _maxVel;
	}

	std::cout << "Acceleration time: " << _a_t << std::endl;
	std::cout << "Duration: " << _duration << std::endl;
	std::cout << "Maximum Acceleration: " << _maxAcc << std::endl;
	std::cout << "Maximum Velocity: " << _maxVel << std::endl;

	_arrived = false;
}

void Trajectory::getNextPoint(const float dt, Vector3f &p, Vector3f &v, Vector3f &a) {

	static float s1=0, s2=0, sdot1=0;
	static float t1=0, t2=0;

	if(_arrived) {
		p = _goal;
		v = Vector3f::Zero();
		a = Vector3f::Zero();
		return;
	}

	_currentTime += dt;

	if ( _currentTime < _a_t ) {
		_s_ddot = _trajAcc;
		_s_dot = _trajAcc * _currentTime;
		_s = 0.5f * _trajAcc * pow(_currentTime, 2);
		s1 = _s;
		sdot1 = _s_dot;
		t1 = _currentTime;
	}
	else if( _currentTime < _duration - _a_t ) {
		_s_ddot = 0.0f;
		//_s_dot = _trajAcc * _a_t;
		_s = s1 + _s_dot * (_currentTime - t1);
		s2 = _s;
		t2 = _currentTime;
	}
	else if(_currentTime < _duration) {
		_s_ddot = -_trajAcc;
		_s_dot = sdot1 - _trajAcc * (_currentTime - t2);
		_s = s2 + sdot1*(_currentTime - t2) - 0.5f * _trajAcc * pow(_currentTime - t2, 2);
	}
	else {
		_s_ddot = 0.0f;
		_s_dot = 0.0f;
		_startingPoint = _goal;
		_arrived = true;
	}

	Vector3f dir = (_goal-_startingPoint).normalized();

	p = _startingPoint + dir * _s;
	v = dir * _s_dot;
	a = dir * _s_ddot;

}