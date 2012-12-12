/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Authors: Alejandro Perez, Sertac Karaman, Ioan Sucan */

#include "PRRTstar.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/base/StateSpaceTypes.h"
#include "ompl/base/GoalTypes.h"
#include "ompl/tools/config/SelfConfig.h"

extern "C"{
#include "utils/alloc.h"
#include "prrts.h"
}

#include <algorithm>
#include <limits>
#include <map>
#include <assert.h>


struct ompl::geometric::PRRTstar_wrapper {
    ompl::geometric::PRRTstar * prrtstar_obj;
         
};

ompl::geometric::PRRTstar::PRRTstar(const base::SpaceInformationPtr &si) 
    : base::Planner(si, "PRRTstar")
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    
    /* Get the StateSpacePtr from the SpaceInformationPtr. This is needed
     * to check the space type and also convert the double [] used to 
     * represent a state in the current prrts implementation and convert it 
     * to a ompl::base::State
     */
    
    stateSpace_            = si_->getStateSpace();
    dimensions_            = si_->getStateDimension();
    ballRadiusConst_       = 5.0;
    regionalSampling_      = false;
    samplesPerStep_        = 1;
    Planner::declareParam<double>("ball_radius_constant", this
                                 , &PRRTstar::setBallRadiusConstant
                                 , &PRRTstar::getBallRadiusConstant);  

    Planner::declareParam<bool>("regional_sampling", this
                                 , &PRRTstar::setRegionalSampling
                                 , &PRRTstar::getRegionalSampling);  
                                 
    Planner::declareParam<int>("samples_per_step", this
                                 , &PRRTstar::setSamplesPerStep
                                 , &PRRTstar::getSamplesPerStep);                                   
}

ompl::geometric::PRRTstar::~PRRTstar(void)
{
    freeMemory();
}

void ompl::geometric::PRRTstar::setup(void)
{   
    /*
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    */
    
    Planner::setup();
    if (!setupPrrtsSystem())
        setup_ = false;
    setupPrrtsOptions();

}

void ompl::geometric::PRRTstar::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    /*
    if (nn_)
        nn_->clear();
    */
}

ompl::base::PlannerStatus ompl::geometric::PRRTstar::solve(
                                 const base::PlannerTerminationCondition &ptc)
{
    std::cout << "Executing solve...." << std::endl;
    checkValidity();
    assert(setup_ == true);
    assert(prrtsSystem_ != NULL);
    prrts_run_for_samples(prrtsSystem_, &options_, 2, 1000);
    return (ompl::base::PlannerStatus::StatusType)NULL;
}


void ompl::geometric::PRRTstar::freeMemory(void)
{
    free(prrtsSystem_);
    delete[] init_config;
    delete[] target_config;
    delete[] max_config;
    delete[] min_config;
}

void ompl::geometric::PRRTstar::getPlannerData(base::PlannerData &data) const
{
    /*
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
        data.addEdge (base::PlannerDataVertex (motions[i]->parent ? motions[i]->parent->state : NULL),
                      base::PlannerDataVertex (motions[i]->state));
     */
}

/* Private helper functions */

bool ompl::geometric::PRRTstar::setupPrrtsSystem()
{
    /* Allocate memory for the prrts_system_t */
    if ((prrtsSystem_ = struct_alloc(prrts_system_t)) == NULL)
        return false;
        
    using namespace ompl::base;
    /**TODO - Adding a check to ensure the supplied state is a RealVectorSpace
     *       for the present the underlying datastructure kdtree used by the
     *       prrts implementation cannot handle unbounded revolute joins.
     *
     *       Syed Hassan added a patch to fix that. Needs to be integreted.
     */
    int spaceType  = stateSpace_->getType();
    if (spaceType != STATE_SPACE_REAL_VECTOR)
        return false;
        
    init_config   = new double[dimensions_];
    target_config = new double[dimensions_];
    max_config    = new double[dimensions_];
    min_config    = new double[dimensions_];
    
    //LOG : print dimensions_
    std::cout<< "Number of dimensions_ : "<< dimensions_ << std::endl;
    //LOG : end

    /* get start states */
    State * st_state = pdef_->getStartState(0);
    for (int i = 0; i < dimensions_ ; i++)
        init_config[i] = st_state->as<RealVectorStateSpace::StateType>()
                                                               ->operator[](i);
    //LOG : print init_config values
    std::cout<< "Init Config : "<< std::endl;    
    for (int i = 0; i< dimensions_; i++)
        std::cout <<  init_config[i] << " ";
    std::cout << std::endl;
    //LOG : end
    
    /* get the target states */
    int goalType = pdef_->getGoal()->getType();	
    
    /**TODO - Adding a check to ensure that the goal has only a single goal
     *       state. Not sure if prrts can handle goal with multiple states.
     *       Or a goal samplable region. type GOAL_STATE means that the goal
     *       can be cast to a single GoalState object.
     *
     *       Discuss with Dr. Alterovitz.
     */
    if (goalType != GOAL_STATE )
        return false;
    
    State * end_state = pdef_->getGoal()->as<GoalState>()->getState();
    for (int i = 0; i < dimensions_ ; i++)
        target_config[i] = end_state->as<RealVectorStateSpace::StateType>()
                                                              ->operator[](i);
                                                               
    //LOG : print target_config values
    std::cout<< "Target Config : "<< std::endl;    
    for (int i = 0; i< dimensions_; i++)
        std::cout <<  target_config[i] << " ";
    std::cout << std::endl;       
    //LOG : end                                                  
    
    /* get the max and min bounds */

    RealVectorBounds rb = stateSpace_->as<RealVectorStateSpace>()->getBounds();
    std::vector<double> lowerBound = rb.low;
    std::vector<double> upperBound = rb.high;
    
    for (unsigned int i=0; i<lowerBound.size(); i++){
        min_config[i] = lowerBound.at(i);
    }
    
    for (unsigned int i=0; i<upperBound.size(); i++){
        max_config[i] = upperBound.at(i);

    }
    
    //LOG : print min and max bounds
    std::cout<< "Min Config : "<< std::endl;    
    for (int i = 0; i< dimensions_; i++)
        std::cout <<  min_config[i] << " ";
    std::cout << std::endl;       
    std::cout<< "Max Config : "<< std::endl;    
    for (int i = 0; i< dimensions_; i++)
        std::cout <<  max_config[i] << " ";
    std::cout << std::endl;   
    //LOG : end     
    ompl::geometric::PRRTstar_wrapper * w 
                                       = new ompl::geometric::PRRTstar_wrapper;
    w->prrtstar_obj = this;
    prrtsSystem_->dimensions   = dimensions_;
    prrtsSystem_->init         = init_config;
    prrtsSystem_->min          = min_config;
    prrtsSystem_->max          = max_config;
    prrtsSystem_->target       = target_config;
    prrtsSystem_->space_config = w;
    
    /* system_data_alloc_func is not being used in our implementation
     * instead have introduces a new void* data member in the prrts_system_t
     * to directly pass the PRRTstar object.
     */
    prrtsSystem_->system_data_alloc_func = NULL ;
    prrtsSystem_->system_data_free_func  = NULL;
     
    prrtsSystem_->dist_func    = PRRTstar::prrts_dist_func;
    prrtsSystem_->in_goal_func = PRRTstar::prrts_in_goal;
    prrtsSystem_->clear_func   = PRRTstar::prrts_clear_func;
    prrtsSystem_->link_func    = PRRTstar::prrts_link_func;
    
    return true;

}

void ompl::geometric::PRRTstar::setupPrrtsOptions ()
{
    memset(&options_, 0, sizeof(options_));
    options_.gamma             = ballRadiusConst_;
    options_.regional_sampling = regionalSampling_;
    options_.samples_per_step  = samplesPerStep_;

}

bool ompl::geometric::PRRTstar::isValid(const double *config)
{
    ompl::base::RealVectorStateSpace state;
    std::vector<double> real;    
    for (int i = 0; i < this->dimensions_; i++){
        real.push_back(config[i]);
    }
    this->stateSpace_->copyFromReals((ompl::base::State*)(&state), real);
    bool valid =  si_->isValid((ompl::base::State*)&state);
    return valid;
}

bool ompl::geometric::PRRTstar::checkMotion (const double *config1
                                           , const double *config2)
{
    ompl::base::RealVectorStateSpace state1, state2;
    std::vector<double> real1, real2;    
    for (int i = 0; i < this->dimensions_; i++){
        real1.push_back(config1[i]);
        real2.push_back(config2[i]);
    }
    
    this->stateSpace_->copyFromReals((ompl::base::State*)(&state1), real1);
    this->stateSpace_->copyFromReals((ompl::base::State*)(&state2), real2);
    
    bool valid =  si_->checkMotion((ompl::base::State*)&state1
                                 , (ompl::base::State*)&state2);
    return valid;

}

bool ompl::geometric::PRRTstar::isSatisfied (const double *config)
{
    ompl::base::RealVectorStateSpace state;
    std::vector<double> real;    
    for (int i = 0; i < this->dimensions_; i++){
        real.push_back(config[i]);
    }
    this->stateSpace_->copyFromReals((ompl::base::State*)(&state), real);
    
    bool valid = pdef_->getGoal()->isSatisfied((ompl::base::State*)&state);
    return valid;
}

double ompl::geometric::PRRTstar::distanceFunction(const double *config1
                                                 , const double *config2) 
{
    std::cout << "Log2" << std::endl;
    ompl::base::State *state1 = stateSpace_->allocState();
    ompl::base::State *state2 = stateSpace_->allocState();
    
    std::vector<double>  real1, real2;    
    for (int i = 0; i < this->dimensions_; i++){
        real1.push_back(config1[i]);
        real2.push_back(config2[i]);
    }
    std::cout << "Log3" << std::endl;
    for( std::vector<double>::const_iterator i = real1.begin(); i != real1.end(); ++i)
        std::cout << *i << ' ';
    //this->stateSpace_->copyFromReals((ompl::base::State*)(&state1), real1);
    //this->stateSpace_->copyFromReals((ompl::base::State*)(&state2), real2);
    //si_->printState(state1);
    //si_->printState((ompl::base::State*)state2);
    //exit(1);
    //return  si_->distance((ompl::base::State*)&state1
    //                             , (ompl::base::State*)&state2);
    //si_->freeState(state1);
    //si_->freeState(state2);
    return 0;
}     

/* end of private helper functions */

/* private static functions. These are used to set the prrts_system_t
   environment */

bool ompl::geometric::PRRTstar::prrts_clear_func(void * usrPtr
                                               , const double *config)
{
    //return static_cast<PRRTstar*>(usrPtr)->isValid(config);
    return true;
}

bool ompl::geometric::PRRTstar::prrts_link_func (void * usrPtr
                                               , const double *config1
                                               , const double *config2)
{
    //return static_cast<PRRTstar*>(usrPtr)->checkMotion(config1, config2);
    return true;

}    

bool ompl::geometric::PRRTstar::prrts_in_goal (void * usrPtr
                                             , const double *config)
{
    //return static_cast<PRRTstar*>(usrPtr)->isSatisfied(config);
    return true;
}

double ompl::geometric::PRRTstar::prrts_dist_func(void * usrPtr
                                                , const double *config1
                                                , const double *config2) 
{
    std::cout << "Log1" << std::endl;
    return static_cast<PRRTstar*>(usrPtr)->distanceFunction(config1,config2);


}     

ompl::geometric::PRRTstar_wrapper * ompl::geometric::PRRTstar::wrapObjToStruct()
{
    ompl::geometric::PRRTstar_wrapper * w = new ompl::geometric::PRRTstar_wrapper;
    w->prrtstar_obj = this;
    return w;

}                                       

                                                                                                                        



