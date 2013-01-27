/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012,2013 UNC Chapel Hill
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

#include "pRRTstar.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/StateSpaceTypes.h"
#include "ompl/base/GoalTypes.h"
#include "ompl/tools/config/SelfConfig.h"

#include "utils/atomic.h"

#include <algorithm>
#include <limits>
#include <map>
#include <assert.h>
#include <vector>

/************************************************************************
************************************************************************/

void ompl::geometric::pRRTstar::printConfig(double *config) 
{
    std::cout<< "State Configuration : " ;
    for (int i = 0 ; i < 2; ++i ) {
       std::cout << config[i] <<", ";
    }
    std::cout<< std::endl;
}


/************************************************************************
************************************************************************/

ompl::geometric::pRRTstar::pRRTstar(const base::SpaceInformationPtr &si) 
    : base::Planner(si, "pRRTstar")
{
    /**\todo - Does prrts have notion of approximate solutions
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
                                 , &pRRTstar::setBallRadiusConstant
                                 , &pRRTstar::getBallRadiusConstant);  

    Planner::declareParam<bool>("regional_sampling", this
                                 , &pRRTstar::setRegionalSampling
                                 , &pRRTstar::getRegionalSampling);  
                                 
    Planner::declareParam<int>("samples_per_step", this
                                 , &pRRTstar::setSamplesPerStep
                                 , &pRRTstar::getSamplesPerStep);       
    
    Planner::declareParam<int>("num_of_threads", this
                                 , &pRRTstar::setNumOfThreads
                                 , &pRRTstar::getNumOfThreads);       
                                                             
}

ompl::geometric::pRRTstar::~pRRTstar(void)
{
    freeMemory();
    //std::cout << "Destroying pRRTstar object" << cout::endl;
}

void ompl::geometric::pRRTstar::setup(void)
{   
    /*
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    */
    
    Planner::setup();
    
    /*if (!setup_temp())
        setup_ = false;
     */
        
    //setupKdTree();    
    //setupThreadedSystem();

}

void ompl::geometric::pRRTstar::clear(void)
{
    Planner::clear();
    //freeMemory();
}

ompl::base::PlannerStatus ompl::geometric::pRRTstar::solve(
                                 const base::PlannerTerminationCondition &ptc)
{   
    std::cout<< "############# Executing Solve now ###############"<<std::endl;
    checkValidity();
    ptc_ = ptc;
    
    std::cout << "Setting up threaded system now ........" << std::endl;
    if(!setupThreadedSystem()){
        return base::PlannerStatus::UNKNOWN;
    }   
    
    std::cout << "Threaded system setup ........." << std::endl;
    
    if (targetConfig_ ==NULL){
        //OMPL_ERROR("Goal undefined");
        return base::PlannerStatus::INVALID_GOAL;
    }
    if (initConfig_ ==NULL){
        //logError("There are no valid initial states!");
        return base::PlannerStatus::INVALID_START;
    }

    /* Start separate threads for each Worker object to execute */
    std::cout << "Start workers now ........." << std::endl;
    startWorkers(workers_, numOfThreads_);
    
    
    
    //solution_ = prrts_run_indefinitely(prrtsSystem_, &options_, numOfThreads_);
    
    if (true){
        return base::PlannerStatus::EXACT_SOLUTION;
    }
    else{
        return base::PlannerStatus::TIMEOUT;
    }
}

void ompl::geometric::pRRTstar::freeMemory(void)
{
    delete[] initConfig_;
    delete[] targetConfig_;
    delete[] maxConfig_;
    delete[] minConfig_;
    delete[] workers_;    
}

void ompl::geometric::pRRTstar::getPlannerData(base::PlannerData &data) const
{
    /*initConfig_
    Planner::getPlannerData(data); */
    
    /**\todo Expose the kd_tree_data structure used by the prrts planner.
     * so that it can be used to populate the Graph in the PlannerData.
     */
}

/* Private helper functions */

bool ompl::geometric::pRRTstar::isValid(const double *config)
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

bool ompl::geometric::pRRTstar::checkMotion (const double *config1
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

bool ompl::geometric::pRRTstar::isSatisfied (const double *config)
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

double ompl::geometric::pRRTstar::distanceFunction(const double *config1
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
  
    double distance =   si_->distance(state1, state2);
    std::cout<<"Distance : " << distance <<std::endl;
    stateSpace_->freeState(state1);
    stateSpace_->freeState(state2);
    
    return distance;
}     

int ompl::geometric::pRRTstar::get_num_procs()
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

/** \brief Check if the PlannerTerminationCondition.operator()
 *   is true.
 */
bool ompl::geometric::pRRTstar::checkPlannerTermCond()
{
    return ptc_();
}
    
/* end of private helper functions */

/* private static functions. These are used to set the prrts_system_t
   environment */

double ompl::geometric::pRRTstar::prrts_dist_func(void * usrPtr
                                                , const double *config1
                                                , const double *config2) 
{
    return static_cast<pRRTstar*>(usrPtr)->distanceFunction(config1,config2);
}   

/*****************************************************************************
functions realted to the setting up the multi threaded system.
*****************************************************************************/

bool ompl::geometric::pRRTstar::setupPrimitives()
{
     using namespace ompl::base;
    /**\todo - Adding a check to ensure the supplied state is a RealVectorSpace
     * for the present the underlying datastructure kdtree used by the
     * prrts implementation cannot handle unbounded revolute joins.
     *
     * Syed Hassan added a patch to fix that. Needs to be integreted.
     */
    int spaceType  = stateSpace_->getType();
    if (spaceType != STATE_SPACE_REAL_VECTOR)
        return false;
        
    initConfig_   = new double[dimensions_];
    targetConfig_ = new double[dimensions_];
    maxConfig_    = new double[dimensions_];
    minConfig_    = new double[dimensions_];
    
    /* get start states */
    State * st_state = pdef_->getStartState(0);
    for (int i = 0; i < dimensions_ ; i++)
        initConfig_[i] = st_state->as<RealVectorStateSpace::StateType>()
                                                               ->operator[](i);
    
    /* get the target states */
    int goalType = pdef_->getGoal()->getType();	
    
    /**\todo - Adding a check to ensure that the goal has only a single goal
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
        targetConfig_[i] = end_state->as<RealVectorStateSpace::StateType>()
                                                              ->operator[](i);
                                                               
    /* get the max and min bounds */

    RealVectorBounds rb = stateSpace_->as<RealVectorStateSpace>()->getBounds();
    std::vector<double> lowerBound = rb.low;
    std::vector<double> upperBound = rb.high;
    
    for (unsigned int i=0; i<lowerBound.size(); i++){
        minConfig_[i] = lowerBound.at(i);
    }
    
    for (unsigned int i=0; i<upperBound.size(); i++){
        maxConfig_[i] = upperBound.at(i);

    }
    
    return true;
}

/**
 *
 *
 */
bool ompl::geometric::pRRTstar::setupThreadedSystem()
{
    std::cout<<"Start setting up threaded system ....." << std::endl;
    Motion *rootMotion  = new Motion();    
    Node *rootNode      = new Node(stateSpace_, pdef_->getStartState(0));
    
    rootMotion->node_   = rootNode;
    rootNode->motion_   = rootMotion; 
    rootNode->inGoal_   = false;
    
    /* Create the Kd_Tree  */
    kd_tree_t *kdTree;
    if(setupPrimitives()) {
    
        kdTree = kd_create_tree(
                             dimensions_
                           , minConfig_
                           , maxConfig_
                           , pRRTstar::prrts_dist_func
                           , rootNode->getConfig()
                           , rootNode
                           , this); 
    }
    else {                           
        return false; 
    }                     
                     
    /** \brief An instance of a Runtime object is shared by each Worker 
     *  thread. It contains the shared configuration and communication 
     *  fields for the workers.
     */
    Runtime *runtime = new Runtime(kdTree, rootNode, numOfThreads_
                                  , samplesPerStep_);
    
    /* Create the Worker objects */
    workers_ = new Worker[numOfThreads_];
    
    for (int i = 0; i< numOfThreads_; i++){
        workers_[i].runtime_  = runtime;
        workers_[i].system_   = this;
        workers_[i].workerID_ = i;
        workers_[i].rng_      = new RNG();
        
        std::cout<< "Worker - "<< workers_[i].workerID_<< " created"
                                                       << std::endl;
       
    }

    return true;
}

/**
 *
 *
 */
void ompl::geometric::pRRTstar::startWorkers(Worker *workers
                                           , int threadCount)
{
    int i;
    pthread_attr_t pthread_attr;
    int error;

    pthread_attr_init(&pthread_attr);

    /*
     * a write barrier here is probably not needed since
     * pthread_create likely takes care of it.  But this is done
     * once, so the overhead does not matter.
     */
    smp_write_barrier();

    /*
     * Start the threads, worker[0] is run on the calling thread,
     * the rest get their own new threads.
     */
     
    for (i=1 ; i<threadCount ; ++i) {

        if ((error = pthread_create(&workers[i].threadID_
                                  , &pthread_attr
                                  , workerMain
                                  , &workers[i])) != 0){
                                 
          //  OMPL_WARN("thread create failed on worker %u", i);
        }

    }
    
    pthread_attr_destroy(&pthread_attr);

    /* run worker 0 on the calling thread */
    workerMain(&workers[0]);
    
    /*
     * After worker[0] completes, join the rest of the workers
     */
    for (i=1 ; i<threadCount ; ++i) {
        error = pthread_join(workers[i].threadID_, NULL);
        if (error != 0){
            //OMPL_WARN("join failed on worker %u", i);
        }
    }
    
    /*
     * this is probably not necessary since pthread_join likely
     * performs the memory synchronization, but again, done
     * once.
     */
    smp_read_barrier_depends();    

}

void * ompl::geometric::pRRTstar::workerMain(void *arg)
{
    Worker *worker              = (Worker*)arg;
    bool is_main_thread         = (worker->workerID_ == 0);
    Runtime *runtime            = worker->runtime_;
    pRRTstar *system            = worker->system_;
    int dimensions              = system->dimensions_;
    

    worker->nearMotion_         = (near_motion_t*)malloc(sizeof(near_motion_t) 
                                                * INITIAL_NEAR_LIST_CAPACITY);
                                                
    //Equivalent of the create_worker_system function
    
    if (!system->regionalSampling_) {
        worker->sampleMin_      = system->minConfig_;
        worker->sampleMax_      = system->maxConfig_;
    } 
    else {
        double *sampleMin       = (double*)array_copy(system->minConfig_
                                                    , system->dimensions_);
        double *sampleMax       = (double*)array_copy(system->maxConfig_
                                                    , system->dimensions_);

        double min = system->minConfig_[REGION_SPLIT_AXIS];
        double t = (system->maxConfig_ [REGION_SPLIT_AXIS] - min) 
                                             / runtime->threadCount_;
        sampleMin[REGION_SPLIT_AXIS] = min + worker->workerID_ * t;
        
        if (worker->workerID_+1 < runtime->threadCount_) {
            sampleMax[REGION_SPLIT_AXIS] = min + (worker->workerID_+1) * t;
        }

        worker->sampleMin_      = sampleMin;
        worker->sampleMax_      = sampleMax;
    }
                                                    
    worker->nearMotionSize_     = 0;
    worker->nearMotionCapacity_ = INITIAL_NEAR_LIST_CAPACITY;                                               
    worker->sampleCount_        = 0;
    worker->clearCount_         = 0;
    worker->motionDistSum_      = 0.0;    
    worker->newConfig_          = new double[dimensions];
    
    int stepNo                  = runtime->stepNo_;
    
    while (!runtime->done_) {
    
        worker->sampleCount_++;
        
        if (workerStep(worker, stepNo)) {
            worker->clearCount_++;
            /*
             * Successfullly added a configuration, update
             * the shared total step count
             */
            stepNo = __sync_fetch_and_add(&runtime->stepNo_, 1);
        }

        if (is_main_thread && system->checkPlannerTermCond()) {
            /* __sync_set(&runtime->done, true); */
            __sync_bool_compare_and_swap(&runtime->done_, false, true);
        }
    }
    
    /* issue a write barrier mostly for the thread-local stats */
    smp_write_barrier();

    return NULL;
}

bool ompl::geometric::pRRTstar::workerStep(Worker *worker, int stepNo)
{
    return false;
}

/*****************************************************************************
 * Defining the functions in the Node class. 
 *****************************************************************************/



void ompl::geometric::pRRTstar::Node::setConfig(const double *config)
{
    std::vector<double> real ;
    for (unsigned int i = 0; i < stateSpace_->getDimension(); ++i) {
         real.push_back(config[i]);
    }
    
    stateSpace_->copyFromReals(state_, real);
}                                    

inline
double * ompl::geometric::pRRTstar::Node::getConfig() const
{
    double * config = new double[stateSpace_->getDimension()];
    std::vector<double> real;  
    
    stateSpace_->copyToReals(real,state_);
      
    for (unsigned int i = 0; i < real.size(); i++){
        config[i] = real[i];
    }
    
    return config;
}

void ompl::geometric::pRRTstar::Node::setMotion(Motion *motion)
{
    this->motion_ = motion;

}
                
