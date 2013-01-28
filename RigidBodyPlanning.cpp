

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
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

/* Author: Ioan Sucan */

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/contrib/rrt_star/RRTstar.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "pRRTstar.h"
#include "ompl/geometric/SimpleSetup.h"

#include "ompl/config.h"
#include <iostream>
#include <fstream>
#include <vector>

#define RAD(x)  ((x) * (M_PI / 180.0))
        
#define HEAD_YAW_MIN           RAD(-119.5)
#define HEAD_YAW_MAX           RAD( 119.5)
#define HEAD_PITCH_MIN         RAD( -38.5)
#define HEAD_PITCH_MAX         RAD(  29.5)

#define L_SHOULDER_PITCH_MIN   RAD(-119.5)
#define L_SHOULDER_PITCH_MAX   RAD( 119.5)
#define L_SHOULDER_ROLL_MIN    RAD(   0.5)
#define L_SHOULDER_ROLL_MAX    RAD(  94.5)
#define L_ELBOW_YAW_MIN        RAD(-119.5)
#define L_ELBOW_YAW_MAX        RAD( 119.5)
#define L_ELBOW_ROLL_MIN       RAD( -89.5)
#define L_ELBOW_ROLL_MAX       RAD(  -0.5)
#define L_WRIST_YAW_MIN        RAD(-104.5)
#define L_WRIST_YAW_MAX        RAD( 104.5)

#define R_SHOULDER_PITCH_MIN   RAD(-119.5)
#define R_SHOULDER_PITCH_MAX   RAD( 119.5)
#define R_SHOULDER_ROLL_MIN    RAD( -94.5)
#define R_SHOULDER_ROLL_MAX    RAD(  -0.5)
#define R_ELBOW_YAW_MIN        RAD(-119.5)
#define R_ELBOW_YAW_MAX        RAD( 119.5)
#define R_ELBOW_ROLL_MIN       RAD(   0.5)
#define R_ELBOW_ROLL_MAX       RAD(  89.5)
#define R_WRIST_YAW_MIN        RAD(-104.5)
#define R_WRIST_YAW_MAX        RAD( 104.5)

/* These are from the spec sheets */
#define NECK_OFFSET_Z (126.50 / 1000.0)
#define SHOULDER_OFFSET_Y (98.00 / 1000.0)
#define UPPER_ARM_LENGTH (90.00 / 1000.0)
#define LOWER_ARM_LENGTH (50.55 / 1000.0)
#define SHOULDER_OFFSET_Z (100.00 / 1000.0)
#define HAND_OFFSET_X (58.00 / 1000.0)
#define HIP_OFFSET_Z (85.00 / 1000.0)
#define HIP_OFFSET_Y (50.00 / 1000.0)
#define THIGH_LENGTH (100.00 / 1000.0)
#define TIBIA_LENGTH (102.74 / 1000.0)
#define FOOT_HEIGHT (45.11 / 1000.0)
#define HAND_OFFSET_Z (15.90 / 1000.0)

/* These are made up and need verification */
#define FOOT_WIDTH (90.0 / 1000.0)
#define FOOT_LENGTH (150.0 / 1000.0)
#define FOOT_OFFSET_X (25.0 / 1000.0)
#define FOOT_OFFSET_Y (10.0 / 1000.0)
#define FOOT_OFFSET_Z (FOOT_HEIGHT / 2.0)

#define HEAD_RADIUS (115.0/2.0 / 1000.0)
#define EAR_RADIUS (90.0/2.0 / 1000.0)
#define HEAD_WIDTH (133.0 / 1000.0)
#define TORSO_RADIUS (75.0 / 1000.0)
#define ARM_RADIUS (66.7/2.0 / 1000.0)
#define LEG_RADIUS (88.9/2.0 / 1000.0)
#define HAND_RADIUS (20.0 / 1000.0)
#define HAND_WIDTH (50.0 / 1000.0)


#define check_angle(joint, angle) assert((joint##_MIN) <= (angle) && (angle) \
                                                           <= (joint##_MAX));
 
namespace ob = ompl::base;
namespace og = ompl::geometric;                                                          

static const double nao_min_config[] = {
        0,0
};

static const double nao_max_config[] = {
        2,4
};
/*
static const double nao_init_config[] = {
       1,.1
};

static const double nao_target_config[] = {
        1,3.9
}; */ 

static const double nao_init_config[] = {
       0,0
};

static const double nao_target_config[] = {
        2,1.55
};                                                          
                                                           
struct Circle{
	double center[2];
	double radius;
};
struct Rect{
	double min[2];
	double max[2];
};
bool collide(double *state, struct Rect rect){
	for(int i = 0; i < 2; i++){
		
		if((state[i] < rect.min[i])||(state[i] > rect.max[i])){
			return false;
		}
	}
	return true;
}


double distanceTo(double *state, struct Circle circle){
	//std::cout << "dist" << std::endl;
	double squaredDist = 0;

	for(int i = 0; i < 2 ; i++){
		squaredDist+=((state[i]-circle.center[i])*
			      (state[i]-circle.center[i]));
	}
	return sqrt(squaredDist);
}


bool collide(double *state, struct Circle circle){
	if(distanceTo(state,circle) > circle.radius){
		return false;
	}
	return true;
}

bool isStateValid(const ob::State *state)
{
	double *config = (double*)malloc(2*sizeof(double));
	for (int i = 0; i < 2 ; i++){
  		config[i] = state->as<ob::RealVectorStateSpace::StateType>()
  ->operator[](i);
	}

	/*struct Circle a;
	a.center[0] = 1;
	a.center[1] = 2;
	a.radius = .99999;
	*/
	struct Rect rect1;
	rect1.min[0] = 0;
	rect1.max[0] = 1.9;
	rect1.min[1] = .5;
	rect1.max[1] = .75;
	struct Rect rect2;
	rect2.min[0] = .1;
	rect2.max[0] = 2;
	rect2.min[1] = 1;
	rect2.max[1] = 1.5;
	struct Circle circ;
	circ.center[0] = 1.25;
	circ.center[1] = 1.75;
	circ.radius = .2;
	
	bool inCollision = false;
	inCollision = 
		collide(config,rect1) ||
		collide(config,rect2) ||
		collide(config,circ);
	
    return (   !inCollision   );
}

void planWithPRRTS(ob::SpaceInformationPtr si, ob::ProblemDefinitionPtr pdef)
{
    // create a planner for the defined space
    ob::PlannerPtr planner(new og::pRRTstar(si));
    
    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
    
    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // perform setup steps for the planner
    planner->setup();    
    
    // solve with the planner
    std::cout<< "Call solve now ......."<< std::endl;
    planner->solve(5.0);
    
    ob::PlannerData data(si);
    planner->getPlannerData(data);
    
    //std::ofstream dotFile;
    //dotFile.open ("path.dot");
    //data.printGraphML (std::cout);
    //dotFile.close();
  
}

void planWithRRTS(ob::SpaceInformationPtr si, ob::ProblemDefinitionPtr pdef)
{
    // create a planner for the defined space
    ob::PlannerPtr planner(new og::RRTstar(si));
    
    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
    
    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // perform setup steps for the planner
    planner->setup();    
    
    // solve with the planner
    std::cout<< "Call solve now ......."<< std::endl;
    planner->solve(5.0);
    
    ob::PlannerData data(si);
    planner->getPlannerData(data);
    
    //std::ofstream dotFile;
    //dotFile.open ("path.dot");
    data.printGraphviz (std::cout);
    //dotFile.close();
  
}

int main(int, char **)
{

    int dimensions_  = 2;
    
    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(dimensions_));

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(dimensions_);
    for (int i = 0; i < dimensions_;i++){
        bounds.low[i] = nao_min_config[i];
        bounds.high[i] = nao_max_config[i];    
    }
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
 
    // construct an instance of  space information from this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // set state validity checking for this space
    si->setStateValidityChecker(boost::bind(&isStateValid, _1));
  
    // create a start state    
    ob::ScopedState<> start(space);
    for (int i = 0; i < dimensions_; i++){
        start[i] = nao_init_config[i];
    }
    
    // create a goal state
    ob::ScopedState<> goal(space);
    for (int i = 0; i < dimensions_; i++){
        goal[i] = nao_target_config[i];
    }
        
    // create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);    
    
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    
    std::cout << "Testing with pRRT* : " <<  std::endl;
	
	planWithpRRTS(si,pdef); 
	
    std::cout << std::endl << std::endl;
    std::cout << "Testing with pRRT* completed: " <<  std::endl;  
 }


