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

/* Authors: Diptorup Deb */

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
#include <vector>

ompl::geometric::PRRTstar::PRRTstar(const base::SpaceInformationPtr &si) 
    : base::Planner(si, "PRRTstar")
{
    /*TODO - Does prrts have notion of approximate solutions
           - What are the Goal Types prrts can handle (for that matter rrts)
     */
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = true;
    specs_.directed = true;
    specs_.multithreaded = true;
    
    /* Get the StateSpacePtr from the SpaceInformationPtr. This is needed
     * to check the space type and also convert the double[], used to 
     * represent a state in the current prrts implementation 
     * to an ompl::base::State
     */
    
    stateSpace_            = si_->getStateSpace();
    dimensions_            = si_->getStateDimension();
    ballRadiusConst_       = 5.0;
    regionalSampling_      = false;
    samplesPerStep_        = 1;
    numOfThreads_          = get_num_procs();
    Planner::declareParam<double>("ball_radius_constant", this
                                 , &PRRTstar::setBallRadiusConstant
                                 , &PRRTstar::getBallRadiusConstant);  

    Planner::declareParam<bool>("regional_sampling", this
                                 , &PRRTstar::setRegionalSampling
                                 , &PRRTstar::getRegionalSampling);  
                                 
    Planner::declareParam<int>("samples_per_step", this
                                 , &PRRTstar::setSamplesPerStep
                                 , &PRRTstar::getSamplesPerStep);       
    
    Planner::declareParam<int>("num_of_threads", this
                                 , &PRRTstar::setNumOfThreads
                                 , &PRRTstar::getNumOfThreads);       
                                                             
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
}

ompl::base::PlannerStatus ompl::geometric::PRRTstar::solveForSamples(
                                                            size_t sampleCount)
{
    checkValidity();
    assert(setup_ == true);
    assert(prrtsSystem_ != NULL);
    
    if (target_config_ ==NULL){
        //logError("Goal undefined");
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (init_config_ ==NULL){
        //logError("There are no valid initial states!");
        return base::PlannerStatus::INVALID_START;
    }

    solution_ = prrts_run_for_samples(prrtsSystem_, &options_, numOfThreads_
                                    , sampleCount);
    
    if (addPathToSolution()){
        return base::PlannerStatus::EXACT_SOLUTION;
    }
    else{
        return base::PlannerStatus::TIMEOUT;
    }
}


ompl::base::PlannerStatus ompl::geometric::PRRTstar::solve(
                                 const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    assert(setup_ == true);
    assert(prrtsSystem_ != NULL);
    
    if (target_config_ ==NULL){
        //logError("Goal undefined");
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (init_config_ ==NULL){
        //logError("There are no valid initial states!");
        return base::PlannerStatus::INVALID_START;
    }
    ptc_ = ptc;
    prrtsSystem_->term_cond = PRRTstar::ompl_planner_term_cond;
    
    solution_ = prrts_run_indefinitely(prrtsSystem_, &options_, numOfThreads_);
    
    if (addPathToSolution()){
        return base::PlannerStatus::EXACT_SOLUTION;
    }
    else{
        return base::PlannerStatus::TIMEOUT;
    }
}


void ompl::geometric::PRRTstar::freeMemory(void)
{
    free(prrtsSystem_);
    delete[] init_config_;
    delete[] target_config_;
    delete[] max_config_;
    delete[] min_config_;
}

void ompl::geometric::PRRTstar::getPlannerData(base::PlannerData &data) const
{
    /*init_config
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
     * for the present the underlying datastructure kdtree used by the
     * prrts implementation cannot handle unbounded revolute joins.
     *
     * Syed Hassan added a patch to fix that. Needs to be integreted.
     */
    int spaceType  = stateSpace_->getType();
    if (spaceType != STATE_SPACE_REAL_VECTOR)
        return false;
        
    init_config_   = new double[dimensions_];
    target_config_ = new double[dimensions_];
    max_config_    = new double[dimensions_];
    min_config_    = new double[dimensions_];
    
    /* get start states */
    State * st_state = pdef_->getStartState(0);
    for (int i = 0; i < dimensions_ ; i++)
        init_config_[i] = st_state->as<RealVectorStateSpace::StateType>()
                                                               ->operator[](i);
    /*
    //LOG : print init_config values
    std::cout<< "Init Config : "<< std::endl;    
    for (int i = 0; i< dimensions_; i++)
        std::cout <<  init_config_[i] << " ";
    std::cout << std::endl;
    //LOG : end
    */
    
    /* get the target states */
    int goalType = pdef_->getGoal()->getType();	
    
    /**TODO - Adding a check to ensure that the goal has only a single goal
     * state. Not sure if prrts can handle goal with multiple states. Or a 
     * goal samplable region. type GOAL_STATE means that the goal can be 
     * cast to a single GoalState object.
     *
     * Discuss with Dr. Alterovitz.
     */
    if (goalType != GOAL_STATE )
        return false;
    
    State * end_state = pdef_->getGoal()->as<GoalState>()->getState();
    for (int i = 0; i < dimensions_ ; i++)
        target_config_[i] = end_state->as<RealVectorStateSpace::StateType>()
                                                              ->operator[](i);
                                                               
    /*
    //LOG : print target_config values
    std::cout<< "Target Config : "<< std::endl;    
    for (int i = 0; i< dimensions_; i++)
        std::cout <<  target_config_[i] << " ";
    std::cout << std::endl;       
    //LOG : end
    */                                                  
    
    /* get the max and min bounds */

    RealVectorBounds rb = stateSpace_->as<RealVectorStateSpace>()->getBounds();
    std::vector<double> lowerBound = rb.low;
    std::vector<double> upperBound = rb.high;
    
    for (unsigned int i=0; i<lowerBound.size(); i++){
        min_config_[i] = lowerBound.at(i);
    }
    
    for (unsigned int i=0; i<upperBound.size(); i++){
        max_config_[i] = upperBound.at(i);

    }
    
    /*
    //LOG : print min and max bounds
    std::cout<< "Min Config : "<< std::endl;    
    for (int i = 0; i< dimensions_; i++)
        std::cout <<  min_config_[i] << " ";
    std::cout << std::endl;       
    std::cout<< "Max Config : "<< std::endl;    
    for (int i = 0; i< dimensions_; i++)
        std::cout <<  max_config_[i] << " ";
    std::cout << std::endl;   
    //LOG : end
    */     

    prrtsSystem_->dimensions   = dimensions_;
    prrtsSystem_->init         = init_config_;
    prrtsSystem_->min          = min_config_;
    prrtsSystem_->max          = max_config_;
    prrtsSystem_->target       = target_config_;
    prrtsSystem_->space_config = this;
    
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
    std::vector<double> real;    
    for (int i = 0; i < dimensions_; i++){
        real.push_back(config[i]);
    }
    ompl::base::State *state = stateSpace_->allocState();
    stateSpace_->copyFromReals(state, real);
    
    stateSpace_->copyFromReals(state, real);
    
    bool clear =  si_->isValid(state);
    
    stateSpace_->freeState(state);    
    
    return clear;
}

bool ompl::geometric::PRRTstar::checkMotion (const double *config1
                                           , const double *config2)
{
    std::vector<double> real1, real2;    
    for (int i = 0; i < dimensions_; i++){
        real1.push_back(config1[i]);
        real2.push_back(config2[i]);
    }
    ompl::base::State *state1 = stateSpace_->allocState();
    ompl::base::State *state2 = stateSpace_->allocState();
    
    stateSpace_->copyFromReals(state1, real1);
    stateSpace_->copyFromReals(state2, real2);
    
    bool validMotion =  si_->checkMotion(state1, state2);
    
    stateSpace_->freeState(state1);
    stateSpace_->freeState(state2);
    
    return validMotion;
}

bool ompl::geometric::PRRTstar::isSatisfied (const double *config)
{
    std::vector<double> real;    
    for (int i = 0; i < dimensions_; i++){
        real.push_back(config[i]);
    }
    ompl::base::State *state = stateSpace_->allocState();
    stateSpace_->copyFromReals(state, real);
        
    bool inGoal = pdef_->getGoal()->isSatisfied(state); 
       
    stateSpace_->freeState(state);
    
    return inGoal;
}

double ompl::geometric::PRRTstar::distanceFunction(const double *config1
                                                 , const double *config2) 
{
    std::vector<double>  real1, real2; 
    for (int i = 0; i < dimensions_; i++){
        real1.push_back(config1[i]);
        real2.push_back(config2[i]);
    }

    ompl::base::State *state1 = stateSpace_->allocState();
    ompl::base::State *state2 = stateSpace_->allocState();
    
    stateSpace_->copyFromReals(state1, real1);
    stateSpace_->copyFromReals(state2, real2);
  
    //si_->printState(state1);
    //si_->printState(state2);
    
    double distance =   si_->distance(state1, state2);
    //std::cout<<"Distance : " << distance <<std::endl;
    stateSpace_->freeState(state1);
    stateSpace_->freeState(state2);
    
    return distance;
}     

int ompl::geometric::PRRTstar::get_num_procs()
{
#ifdef __linux__
    return sysconf(_SC_NPROCESSORS_ONLN);
#elif defined(BSD) || defined(__APPLE__)

    int procs;
    int mib[4];
    size_t len = sizeof(procs);

    mib[0] = CTL_HW;
    mib[1] = HW_AVAILCPU;

    sysctl(mib, 2, &procs, &len, NULL, 0);

    if (procs < 1) {
        mib[1] = HW_NCPU;
        sysctl(mib, 2, &procs, &len, NULL, 0);

        if (procs < 1)
            procs = 1;
    }

    return procs;

#else
    return -1;
#endif
}  

bool ompl::geometric::PRRTstar::addPathToSolution()
{
    if (solution_ != NULL){
        /* construct the solution path */
        
        ompl::geometric::PathGeometric *path 
                              = new ompl::geometric::PathGeometric(si_);
        path->append(pdef_->getStartState(0));
        for (unsigned int j=1 ; j < solution_->path_length ; ++j){
            std::vector<double> real;    
            for (int i = 0; i < dimensions_; i++){
                real.push_back(solution_->configs[j][i]);
            }
            ompl::base::State *state = stateSpace_->allocState();
            stateSpace_->copyFromReals(state, real);
            path->append(state);
        }
    /*TODO - Does prrts have the notion of approximate solutions ?
     * Does/can the planner return multiple solutions? How can they be 
     * stored?
     */
        pdef_->addSolutionPath(base::PathPtr(path));
        return true;
    }
    else{
        return false;
    }
}

/** \brief Check if the PlannerTerminationCondition.operator()
 *   is true.
 */
bool ompl::geometric::PRRTstar::checkPlannerTermCond()
{
    return ptc_();
}
    
/* end of private helper functions */

/* private static functions. These are used to set the prrts_system_t
   environment */

bool ompl::geometric::PRRTstar::prrts_clear_func(void * usrPtr
                                               , const double *config)
{
    return static_cast<PRRTstar*>(usrPtr)->isValid(config);
}

bool ompl::geometric::PRRTstar::prrts_link_func (void * usrPtr
                                               , const double *config1
                                               , const double *config2)
{
    return static_cast<PRRTstar*>(usrPtr)->checkMotion(config1, config2);
}    

bool ompl::geometric::PRRTstar::prrts_in_goal (void * usrPtr
                                             , const double *config)
{
    return static_cast<PRRTstar*>(usrPtr)->isSatisfied(config);
}

double ompl::geometric::PRRTstar::prrts_dist_func(void * usrPtr
                                                , const double *config1
                                                , const double *config2) 
{
    return static_cast<PRRTstar*>(usrPtr)->distanceFunction(config1,config2);
}   


/** \brief Check if the PlannerTerminationCondition is satisfied  
 */
 
bool ompl::geometric::PRRTstar::ompl_planner_term_cond(void * usrPtr)
{   
    return static_cast<PRRTstar*>(usrPtr)->checkPlannerTermCond();
}
                                                                                                                    



