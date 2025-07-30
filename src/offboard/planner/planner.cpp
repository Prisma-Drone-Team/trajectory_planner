/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2023, Jonathan Cacace <jonathan.cacace@gmail.com>
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "planner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;


bool PATH_PLANNER::check_state( const double * s ) {

  if( _tree_obj ) {
      
        // set the rototranslation of the robot considering the planned state
        fcl::Vec3f translation(s[0], s[1], s[2]);
        fcl::Quaternion3f rotation(1.0, 0.0, 0.0, 0.0);

        // check collisions between the octomap and the robot shape      
        fcl::CollisionObject treeObj((_tree_obj));
        fcl::CollisionObject robotObject(_Robot);
            
        robotObject.setTransform(rotation, translation);
        fcl::CollisionRequest requestType(1,false,1,false);
        fcl::CollisionResult collisionResult;
        fcl::collide(&robotObject, &treeObj, requestType, collisionResult);

        return(!collisionResult.isCollision());
    }
    else {        
        return true;
    } // No map available, return a valid state in anycase
}


/**
 * \brief Check the validity of a sample state.
 *
 * This function takes a sample state to validate if it is free or not
 * It is called directly from the OMPL framework and used an OctoTree along with the FCL libraries to validate the state
 * 
 * \param state: an SE(3) ompl state  
 * \return The state validity (true: free state) or (false: occupied state)
 */
bool PATH_PLANNER::isStateValid(const ob::State *state) {
    

    // cast the abstract state type to the type we expect
    const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    if( pos->values[0] < _x_bounds[0] || pos->values[0] > _x_bounds[1]) {
        //std::cout << "X fault: " << pos->values[0] << " - " << _x_bounds[0] << ", " << _x_bounds[1] << std::endl;
        return false;
    }
    
    if( pos->values[1] < _y_bounds[0] || pos->values[1] > _y_bounds[1]) {

        //std::cout << "Y fault: " << pos->values[1] << " - " << _y_bounds[0] << ", " << _y_bounds[1] << std::endl;
        return false;
    }
    
    if( pos->values[2] < _z_bounds[0] || pos->values[2] > _z_bounds[1]) {

        //std::cout << "Z fault: " << pos->values[2] << " - " << _z_bounds[0] << ", " << _z_bounds[1] << std::endl;
        return false;
    }
    
    if( _tree_obj ) {
        // cast the abstract state type to the type we expect
        const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
            
        // set the rototranslation of the robot considering the planned state
        fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
        fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);

        // check collisions between the octomap and the robot shape      
        fcl::CollisionObject treeObj((_tree_obj));
        fcl::CollisionObject robotObject(_Robot);
            
        robotObject.setTransform(rotation, translation);
        fcl::CollisionRequest requestType(1,false,1,false);
        fcl::CollisionResult collisionResult;
        fcl::collide(&robotObject, &treeObj, requestType, collisionResult);

        return(!collisionResult.isCollision());
    }
    else {        
        return true;
    } // No map available, return a valid state in anycase
}

/** Return an optimization objective which attempts to steer the robot
    away from obstacles. */
ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si) {
    return std::make_shared<ClearanceObjective>(si);
}


ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si) {
    auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    auto clearObj(std::make_shared<ClearanceObjective>(si));

    return 10.0*lengthObj + clearObj;
}

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si) {
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}

ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si) {
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
	return obj;
}

/**
 * \brief PATH_PLANNER constructor.
 *
 * This function instantiate an object of PATH_PLANNER type
 *
 */
PATH_PLANNER::PATH_PLANNER( ) {}

/**
 * \brief Init function.
 *
 * This function initialize all the stuff needed to plan
 * 
 * \param xbounds: workspace bounds toward x axis
 * \param xbounds: workspace bounds toward y axis
 * \param xbounds: workspace bounds toward z axis
 */
void PATH_PLANNER::init( const double * xbounds, const double * ybounds, const double * zbounds ) {

    std::cout << "---Initializing OMPL path planner---" << std::endl;

	_space = ob::StateSpacePtr(new ob::SE3StateSpace());
	ob::RealVectorBounds bounds(3);

	bounds.setLow (0,  xbounds[0]);
	bounds.setHigh(0,  xbounds[1]); //x
	bounds.setLow (1,  ybounds[0]);
	bounds.setHigh(1,  ybounds[1]); //y
	bounds.setLow (2,  zbounds[0]);
	bounds.setHigh(2,  zbounds[1]); //z

	_space->as<ob::SE3StateSpace>()->setBounds(bounds);
	
    // construct an instance of  space information from this state space
	_si = ob::SpaceInformationPtr(new ob::SpaceInformation(_space));
	_si->setStateValidityChecker(std::bind(&PATH_PLANNER::isStateValid, this, std::placeholders::_1 ));
	_pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(_si));    

    //_pdef->setOptimizationObjective( getClearanceObjective(_si) );
    _pdef->setOptimizationObjective( getThresholdPathLengthObj( _si ));

    _planner = ob::PlannerPtr(new og::RRTstar(_si));    
    
	std::cout << "---Planner initialized!---" << std::endl;

}


/**
 * \brief This function optimies the path planned by the plan function. 
 * It checks all the points in sequence that don't contains obstacles in the middle, removing them
 * 
 * \param poses: the full not-optimized path
 * \param delta: the space interval to check two valid points of the path
 * \return int: not used
 */
int PATH_PLANNER::optimize_path(const std::vector<POSE> & poses, const double delta, std::vector<POSE> & opt_poses) {

    opt_poses = poses;
    if( poses.size() == 2 ) {
        std::cout << "[optimization not useful] only 2 points in the path" << std::endl;        
        return 0;
    } //Two point, it is already an optimized path
    
    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
    Eigen::Vector3d pX;
    Eigen::Vector3d s;
  
    fcl::CollisionObject treeObj((_tree_obj));
    fcl::CollisionObject robotObject(_Robot);


    for ( int i=0; i<opt_poses.size()-1; i++ ) {
        for ( int j=i+1;j<opt_poses.size(); j++ ) {
            int i0=i;
            int i1=j;
        
            p0 << opt_poses[i0].position.x, opt_poses[i0].position.y, opt_poses[i0].position.z;
            p1 << opt_poses[i1].position.x, opt_poses[i1].position.y, opt_poses[i1].position.z;    
            pX = p0;

            bool obs = false;

            s =  (p1-p0);
            s /= s.norm();

            while( (pX-p1).norm() > delta && !obs ) {
                pX += s*delta;

                if( _tree_obj ) {
                    fcl::Vec3f translation(pX[0], pX[1], pX[2]);
                    fcl::Quaternion3f rotation(1, 0, 0, 0);                            
                    robotObject.setTransform(rotation, translation);
                    fcl::CollisionRequest requestType(1,false,1,false);
                    fcl::CollisionResult collisionResult;            
                    fcl::collide(&robotObject, &treeObj, requestType, collisionResult);
                 
                    obs = collisionResult.isCollision();
                }                
            }

            if( !obs ) {
                opt_poses.erase( opt_poses.begin()+i0+1, opt_poses.begin()+i1 );               
                //if( i>0) i--;
            }
        }
    }
    return 1;
}


/**
 * \brief This function plans the path
 * 
 * \param max_t: maximum planning time in seconds
 * \return poses: the not-optimized path
 * \return opt_poses: the optimized path 
 * \return int: planned path (1-solved, 0-not solved)
 */
//int PATH_PLANNER::plan(const double & max_t, std::vector<POSE> & poses, std::vector<POSE> & opt_poses) {
int PATH_PLANNER::plan(const double & max_t, const double * xbounds, const double * ybounds, const double * zbounds, std::vector<POSE> & poses, std::vector<POSE> & opt_poses) {
    
    _x_bounds[0] = xbounds[0];
    _x_bounds[1] = xbounds[1];
    _y_bounds[0] = ybounds[0];
    _y_bounds[1] = ybounds[1];
    _z_bounds[0] = zbounds[0];
    _z_bounds[1] = zbounds[1];
    
    //Planner not correctly initialized
    if( !_start_state_set || !_goal_state_set ) {
        std::cout << "Problem not correctly set: start or goal state not defined!" << std::endl;
        return -1;
    }

    if( !_rgeometry_set ) {
        std::cout << "Problem not correctly set: robot geometry not specified!" << std::endl;
        return -1;
    }


    //-3: goal state not valid
    if( _goal.position.x < xbounds[0] || _goal.position.x > xbounds[1]) {
        //std::cout << "X out of bounds: " << _goal.position.x << " (" << xbounds[0] << ", " << xbounds[1] << ")" << std::endl;
        return -3;
    }
    
    if( _goal.position.y < ybounds[0] || _goal.position.y > ybounds[1]) {
        //std::cout << "Y out of bounds: " << _goal.position.y << " (" << ybounds[0] << ", " << ybounds[1] << ")" << std::endl;
        return -3;
    }
    
    if( _goal.position.z < zbounds[0] || _goal.position.z > zbounds[1]) {
        //std::cout << "Z out of bounds: " << _goal.position.z << " (" << zbounds[0] << ", " << zbounds[1] << ")" << std::endl;
        return -3;
    }


    ob::ScopedState<ob::SE3StateSpace> start(_space);		
    ob::ScopedState<ob::SE3StateSpace> goal(_space);
    
    if( _random_start_state )
        start.random();
    else {
	    start->setXYZ(_start.position.x, _start.position.y, _start.position.z);
        start->rotation().w = _start.orientation.w;
        start->rotation().x = _start.orientation.x;
        start->rotation().y = _start.orientation.y;
        start->rotation().z = _start.orientation.z;
    }

    if( _random_goal_state )
        goal.random();
    else {
        goal->setXYZ(_goal.position.x, _goal.position.y, _goal.position.z);
        goal->rotation().w = _goal.orientation.w;
        goal->rotation().x = _goal.orientation.x;
        goal->rotation().y = _goal.orientation.y;
        goal->rotation().z = _goal.orientation.z;
    }  
    
    
    _pdef->clearGoal();
    _pdef->clearStartStates();
    _pdef->clearSolutionPaths();
    
    _pdef->setStartAndGoalStates(start, goal);
    
    _planner->clear();
    
    _planner->setProblemDefinition(_pdef);
    _planner->setup();

    if( _verbose ) _si->printSettings(std::cout);
	if( _verbose ) _pdef->print(std::cout);
      
    ob::PlannerStatus solved = _planner->solve(max_t);

    if( solved ) {
    
        ob::PathPtr path = _pdef->getSolutionPath();
        og::PathGeometric* pth = _pdef->getSolutionPath()->as<og::PathGeometric>();
        
        if(_verbose) pth->printAsMatrix(std::cout);
        
        poses.resize( pth->getStateCount () );

        //for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++) {
        for ( auto path_idx = 0; path_idx < pth->getStateCount (); path_idx++) {
		
        	const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();
			// extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
			// extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);


            poses[path_idx].position.x = pos->values[0];
            poses[path_idx].position.y = pos->values[1];
            poses[path_idx].position.z = pos->values[2];

            poses[path_idx].orientation.x = rot->x;
            poses[path_idx].orientation.y = rot->y;
            poses[path_idx].orientation.z = rot->z;
            poses[path_idx].orientation.w = rot->w;		
		}

        optimize_path(poses, _delta, opt_poses);
    }

    reset_planner();

    return (solved) ? 1 : 0;    
}