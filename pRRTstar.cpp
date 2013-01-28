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
}

void ompl::geometric::pRRTstar::clear(void)
{
    Planner::clear();
}

ompl::base::PlannerStatus ompl::geometric::pRRTstar::solve(
                                 const base::PlannerTerminationCondition &ptc)
{   
    checkValidity();
    ptc_ = ptc;
    
    printLog("Setting up the multi-threaded system.");
    if(!setupThreadedSystem()){
        return base::PlannerStatus::UNKNOWN;
    }   
    
    printLog("Multi-threaded system setup.");
    
    if (targetConfig_ ==NULL){
        //OMPL_ERROR("Goal undefined");
        return base::PlannerStatus::INVALID_GOAL;
    }
    if (initConfig_ ==NULL){
        //OMPL_ERROR("There are no valid initial states!");
        return base::PlannerStatus::INVALID_START;
    }

    /* Start separate threads for each Worker object to execute */
    printLog("Starting the workers threads now.");
    
    startWorkers(workers_, numOfThreads_);
    
    if(runtime_->bestPath) {
        getWorkerData();
        return base::PlannerStatus::EXACT_SOLUTION;
    }
    else {
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
    Planner::getPlannerData(data);    
    
    Motion *ptr;
    std::vector<Motion*> motions;
    
    if (runtime_->bestPath_) {
        for(ptr = runtime_->bestPath_; ptr != NULL; ptr = ptr->parent_) {
            motions.push_back(ptr);
        }
    }
        
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
        data.addEdge (base::PlannerDataVertex (motions[i]->parent_ ? 
                                    motions[i]->parent_->node_->state_ : NULL),
                      base::PlannerDataVertex (motions[i]->node_->state_));
                      
                         
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
    
    if(!validMotion)
        printLog("Motion not valid");
    else
        printLog("Motion is valid");
        
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

/******************************************************************************
 * Functions to setup the threaded system and the other                       *
 * primitives needed by the pRRTstar implementation.                          *
 *****************************************************************************/

/**
 *
 *
 */
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
                     

    runtime_ = new Runtime(kdTree, rootNode, numOfThreads_
                                  , samplesPerStep_);
    
    /* Create the Worker objects */
    workers_ = new Worker[numOfThreads_];
    
    for (int i = 0; i< numOfThreads_; i++){
        workers_[i].runtime_  = runtime_;
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


/**
 *
 *
 */
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


/**
 *
 *
 */
bool ompl::geometric::pRRTstar::workerStep(Worker *worker, int stepNo)
{
    pRRTstar *system = worker->system_;
    double radius;
    double nearestDist, dist;
    unsigned nearMotionSize;
    Node *nearest;
    Node *newNode;
    Motion *motion;
    unsigned i, j;
    double *newConfig = worker->newConfig_;

    system->randomSample(worker, newConfig);
    
    printConfig(newConfig, worker->system_->dimensions_);
        
    if (!system->isValid(newConfig)) {
        printLog("Sampled state is not valid (in collision maybe)");
        return false;
    }    
    
    radius = worker->system_->ballRadiusConst_ *
                pow(log((double)stepNo + 1.0) / (double)(stepNo + 1.0),
                         1.0 / (double)system->dimensions_);

    assert(radius > 0.0);
    
    worker->nearMotionSize_ = 0;
    
    nearMotionSize = kd_near(worker->runtime_->kdTree_, newConfig, radius
                           , workerNearCallback, worker);

    assert(nearMotionSize == worker->nearMotionSize_);
    
    if (nearMotionSize == 0) {
    
        printLog("Nothing in vicinity, steering towards nearest");
        /*
         * nothing found in the vicinity, try steering towards
         * the nearest
         */

        nearest = (Node*)kd_nearest(worker->runtime_->kdTree_, newConfig
                                                             , &nearestDist);
        assert(radius <= nearestDist);

        system->steer(system->dimensions_, newConfig, nearest->getConfig()
                                                    , radius / nearestDist);
        /**\todo - Why is this check needed again? redundant ? */    
        if (!system->isValid(newConfig)) {
            printLog("Sampled state is not valid (in collision maybe)");
            return false;
        }    

        dist = system->distanceFunction(newConfig, nearest->getConfig());
        std::cout << "Nearest distance - " << dist << std::endl;       
        assert(dist <= nearestDist);

        if (!system->canLink(worker, newConfig, nearest->getConfig(), dist)) {
            printLog("Could not link to nearest");
            return false;
        }    
        printLog("Link to the nearest node and add a node to the kd_tree");
        /*
         * The new configuration has a link.  Create a new
         * node and insert it into the kd-tree.  There are no
         * near nodes to rewire, no children node to update,
         * so we can return immediately
         */

        newNode = system->createNode(
                       newConfig
                     , system->targetConfig_ == NULL 
                                            && system->isSatisfied(newConfig)
                     , dist
                     , nearest->motion_);

        system->updateBestPath(worker, newNode->motion_, radius);

        kd_insert(worker->runtime_->kdTree_, newNode->getConfig(), newNode);
      
        return true;
    }
    
    /*
     * At this point the near_list array is populated, and the
     * next step is to sort by their path distances in ascending
     * order, to reduce the number of calls to link that follow.
     */

    qsort(worker->nearMotion_, nearMotionSize, sizeof(near_motion_t)
                                                           , nearListCompare);

    for (i=0 ; i<nearMotionSize ; ++i) {
        motion = worker->nearMotion_[i].motion_;

        if (system->canLink(worker, motion->node_->getConfig(), newConfig
                          , worker->nearMotion_[i].motionCost_)) {
            /*
             * We've found a near node that the new
             * configuration can link to.  We're
             * guaranteed at this point that we're linking
             * to the shortest reachable path because of
             * the previous sort.  (Caveat that another
             * what another thread might do to the nodes
             * after this one)
             */
            newNode = system->createNode(
                                newConfig
                              , system->targetConfig_ == NULL 
                                            && system->isSatisfied(newConfig)
                              , worker->nearMotion_[i].motionCost_
                              , motion);

            system->updateBestPath(worker, newNode->motion_, radius);

            /*
             * insert into the kd-tree.  After the
             * kd-insert, other threads might modify the
             * new_node's path.
             */
            kd_insert(worker->runtime_->kdTree_, newNode->getConfig()
                                                                 , newNode);

            /**\todo: reference release on near_list[i]->link */
            
            /*
             * Now we rewire the remaining nodes in the
             * near list through this the new_node if the
             * resulting path would be shorter.
             *
             * Iteration is done in reverse order (from
             * most distant to closest), to reduce the
             * likelihood of a node getting updated twice
             * (or more) during the rewiring.  E.g. if
             * node A is rewired through new_node, and
             * node B goes through A, it will be
             * recursively rewired.  If B also appears in
             * the near list, then B would be rewired
             * again.
             */
            
            for (j=nearMotionSize ; --j > i ; ) {
                system->rewire(worker,
                       worker->nearMotion_[j].motion_,
                       worker->nearMotion_[j].motionCost_,
                       newNode,
                       radius);
            
                /**\todo: reference release on near_list[j].link */
            }
            
            return true;
        }

        /**\todo: reference release on near_list[j].link */
    }

    /*
     * At this point, we've iterated through every near node and
     * found nothing that links.  We return false indicating no
     * new samples were added.
     */
    return false;
}

/******************************************************************************
 * Functions needed by the pRRT* execution                                    *
 *****************************************************************************/
            
/** \brief Callback function which is passed to the kd_tree to 
 *  build the near-list (Worker.nearMotion_) data structure 
 *  during a call to kd_near.
 * 
 */
void ompl::geometric::pRRTstar::workerNearCallback(void *workerArg
                                                 , int no
                                                 , void *nearNode
                                                 , double dist)
{

    Worker *worker = (Worker*)workerArg;
    Motion *nearMotion;

    assert(no == (int)worker->nearMotionSize_);
    assert(no <= (int)worker->nearMotionCapacity_);

    /* check if we need to grow the array */
    if (no == (int)worker->nearMotionCapacity_) {
        if ((worker->nearMotion_ 
               = (near_motion_t*)realloc(
                                     worker->nearMotion_
                                   , sizeof(near_motion_t) * 2 * no)) == NULL
                                        ) {
            /*OMPL_ERROR(1, "failed to reallocate cost nodes 
                            (requested %lu bytes)"
                 , (unsigned long)(sizeof(near_list_t) * 2 * no));
             */
                 
         }
        
		worker->nearMotionCapacity_ = 2 * no;
    }

    nearMotion = ((Node*)nearNode)->motion_;

    /**\todo: reference acquire on near_link */

    worker->nearMotion_[no].motion_     = nearMotion;
    worker->nearMotion_[no].motionCost_ = dist;
    worker->nearMotion_[no].pathCost_   = nearMotion->pathCost_ + dist;

    worker->nearMotionSize_++;
} 

/**
 *
 * \todo: steer makes some assumptions, it's implementation should be
 * left to the system 
 */
void ompl::geometric::pRRTstar::steer(size_t dimensions, double *newConfig
                     , const double *targetConfig, double scale)
{                    
    unsigned i;

    for (i=0 ; i<dimensions ; ++i) {
        newConfig[i] = targetConfig[i] + (newConfig[i] - targetConfig[i]) 
                                                                    * scale;
    }
}                                                

/**
 *
 *
 */
bool ompl::geometric::pRRTstar::canLink(Worker *worker, const double *a
                                      , const double *b, double motionCost)
{
    worker->motionDistSum_ += motionCost;
    bool r = worker->system_->checkMotion(a, b);

    return r;   
} 

/**
 *
 *
 */
ompl::geometric::pRRTstar::Node * ompl::geometric::pRRTstar::createNode(
                                                       double *config
                                                     , bool inGoal
                                                     , double motionCost
                                                     , Motion *parentMotion)
{
    Node *newNode = new Node(stateSpace_, inGoal);
    newNode->setConfig(config);

    Motion *motion = createMotion(newNode, motionCost, parentMotion);

     /*
      * The new node and link will not be visible to other threads
      * until we call add_child_link.
      */
    newNode->motion_ = motion;

    smp_write_barrier();

    addChildMotion(parentMotion, motion);

    return newNode;
}  

/**
 *
 *
 */
ompl::geometric::pRRTstar::Motion * ompl::geometric::pRRTstar::createMotion(
                             Node *node, double motionCost, Motion *parent)
{
    assert(motionCost > 0);
    Motion *newMotion = new Motion(node, parent, motionCost);   
    newMotion->pathCost_ = motionCost + parent->pathCost_;

    return newMotion;
}                                                                                        

/**
 *
 *
 */ 
void ompl::geometric::pRRTstar::addChildMotion(Motion *parent, Motion *child)
{
    Motion *expected = NULL;
    Motion *nextSibling;

    do {
        nextSibling = parent->firstChild_;

        /*
         * We could use:
         *
         * __sync_set(&child->next_sibling, next_sibling); 
         *
         * but using __sync_bool_compare_and_swap allows us to
         * test our assumption that this is the only thread
         * adding the child.  It also allows us to use a
         * builtin atomic.
         */
        if (!__sync_bool_compare_and_swap(&child->nextSibling_
                                        , expected, nextSibling)) {
            assert(false);
        }
        expected = nextSibling;

    } while (!__sync_bool_compare_and_swap(&parent->firstChild_
                                          , nextSibling, child));
}

/**
 *
 *
 */
void ompl::geometric::pRRTstar::updateBestPath(Worker *worker, Motion *motion
                                                , double radius)
{
    Node *node;
    double motionDistToGoal;
    Motion *bestPath;
    double bestDist;
    double distToTarget;

    node = motion->node_;

    if (node->inGoal_) {
        motionDistToGoal = motion->pathCost_;
    } 
    else if (targetConfig_ != NULL) {
        /* goal-biased search */
        distToTarget = distanceFunction(node->getConfig(), targetConfig_);
        if (distToTarget > radius ||
            !canLink(worker, node->getConfig(), targetConfig_, distToTarget))
            return;
        motionDistToGoal = motion->pathCost_ + distToTarget;
    } 
    else {
        return;
    }

    do {
        bestPath = worker->runtime_->bestPath_;

        if (bestPath != NULL) {
            bestDist = bestPath->pathCost_;

            if (!bestPath->node_->inGoal_) {
                /*
                 * if the current best is not in goal
                 * itself, it must have a link to a
                 * target goal configuration
                 */
                bestDist += distanceFunction(bestPath->node_->getConfig()
                                                             , targetConfig_);

                /**\todo: the dist_func is called for
                 * every update, might be worth
                 * caching */
            }

            if (motionDistToGoal >= bestDist) {
                return;
            }
        }
    } while (!__sync_bool_compare_and_swap(&worker->runtime_->bestPath_
                                                          , bestPath, motion));

#if 0
    printf("New best path found, dist=%f\n", link_dist_to_goal);
#endif

}  

/**
 *
 *
 */
ompl::geometric::pRRTstar::Motion * ompl::geometric::pRRTstar::setNodeMotion(
                                   Node *node, Motion *oldMotion
                                 , double motionCost, Motion *parentMotion)
{
    Motion *newMotion = createMotion(node, motionCost, parentMotion);

    smp_write_barrier();

    if (__sync_bool_compare_and_swap(&node->motion_, oldMotion, newMotion)) {
    
        assert(newMotion->pathCost_ <= oldMotion->pathCost_);
        addChildMotion(parentMotion, newMotion);        
        return newMotion;
    } 
    else {
        return NULL;
    }

}      

/**
 *
 *
 */  
bool ompl::geometric::pRRTstar::isMotionExpired(Motion *motion)
{
    Node *node = motion->node_;
    
    smp_read_barrier_depends();
    
    return node->motion_ != motion;
}

/**
 *
 *
 */  
ompl::geometric::pRRTstar::Motion * ompl::geometric::pRRTstar::removeFirstChild
                                                               (Motion *parent)
{
    Motion *child;
    Motion *sibling;

    assert(isMotionExpired(parent));

    do {
        child = parent->firstChild_;
        if (child == NULL) {
            return NULL;
        }
        sibling = child->nextSibling_;
    } while (!__sync_bool_compare_and_swap(&parent->firstChild_, child
                                         , sibling));

    if (!__sync_bool_compare_and_swap(&child->nextSibling_, sibling, NULL))
        assert(false);

    return child;

}                                                               

/**
 *
 *
 */                                 
void ompl::geometric::pRRTstar::updateChildren(Worker *worker
                                             , Motion *newParent
                                             , Motion *oldParent
                                             , double radius)
{
    Motion *oldChild;
    Motion *newChild;
    Node *node;
    double pathCost;

    assert(newParent->node_ == oldParent->node_);
    assert(isMotionExpired(oldParent));
    assert(newParent->pathCost_ <= oldParent->pathCost_);

    for (;;) {
        oldChild = removeFirstChild(oldParent);
        
        if (oldChild == NULL) {
            if (isMotionExpired(newParent)) {
                oldParent = newParent;
                newParent = oldParent->node_->motion_;
                continue;
            }
            return;
        }

        if (isMotionExpired(oldChild)) {
        
            assert(worker->runtime_->threadCount_ > 1);
            continue;
        }

        pathCost = newParent->pathCost_ + oldChild->motionCost_;
        node = oldChild->node_;

        if (node->motion_->parent_->node_ != oldParent->node_) {
            continue;
        }

        newChild = setNodeMotion(node, oldChild, oldChild->motionCost_
                                               , newParent);
        if (newChild != NULL) {

            updateChildren(worker, newChild, oldChild, radius);
            updateBestPath(worker, newChild, radius);
        } 
        else {

            assert(worker->runtime_->threadCount_ > 1);
            assert(node->motion_ != oldChild);
        }
    }
}

/**
 *
 *
 */
bool ompl::geometric::pRRTstar::removeChild(Motion *parent , Motion *child)
{
    Motion *sibling;
    Motion *n;
    Motion *p;

    assert(isMotionExpired(child));
    assert(child->parent_ == parent);

    for (;;) {
        n = parent->firstChild_;

        if (n == child) {
            /* the child is at the head of the list of children */

            sibling = child->nextSibling_;
            
            if (__sync_bool_compare_and_swap(&parent->firstChild_, child
                                           , sibling)) {
                break;
            } 
            else {
                continue;
            }
        }

        if (n == NULL) {
            return false;
        }

        for (;;) {
            p = n;

            n = n->nextSibling_;

            if (n == NULL) {
                /*
                 * iterated through the entire list of
                 * children and did not find the
                 * child.  possibly another thread has
                 * already removed it.
                 */
                return false;
            }

            if (n == child) {
                /*
                 * found it.  try to remove the child
                 * by linking the previous node's
                 * sibling to the nodes next sibling.
                 */
                sibling = child->nextSibling_;
                if (__sync_bool_compare_and_swap(&p->nextSibling_, child
                                               , sibling)) {
                    goto done;
                } 
                else {
                    break;
                }
            }
        }
    }

done:
    __sync_bool_compare_and_swap(&child->nextSibling_, sibling, NULL);

    return true;
}
             
/**
 *
 *
 */
void ompl::geometric::pRRTstar::rewire(Worker *worker, Motion *oldMotion
                                      , double motionCost, Node *newParent
                                      , double radius)
{
    Node *node;
    Motion *parentMotion;
    Motion *newMotion;
    Motion *updatedOldMotion;
    double pathCost;

    assert(oldMotion->parent_ != NULL);
    assert(oldMotion->parent_->node_ != newParent);

    node = oldMotion->node_;
    parentMotion = newParent->motion_;
    pathCost = parentMotion->pathCost_ + motionCost;

    /* check if rewiring would create a shorter path */
    if (pathCost >= oldMotion->pathCost_)
        return;
    /* make sure rewiring is possible */
    if (!canLink(worker, oldMotion->node_->getConfig()
                       , newParent->getConfig(), motionCost))
        return;

    /*
     * Rewire the node.  This loop continues to attempt atomic
     * updates until either the update succeeds or the path_cost
     * of the old_link is found to be better than what we are
     * trying to put in.
     */
    do {
        newMotion = setNodeMotion(node, oldMotion, motionCost, parentMotion);

        if (newMotion != NULL) {
            
            updateChildren(worker, newMotion, oldMotion, radius);
            updateBestPath(worker, newMotion, radius);
            
            if (isMotionExpired(parentMotion)) {
                updateChildren(worker, parentMotion->node_->motion_
                             , parentMotion, radius);
            }
            /*
             * Setting the new_link expires the old_link
             * but does not remove it from the parent.
             * Here we just do a little clean up to remove
             * the old_link.  This is placed after the
             * parent expiration check because an expired
             * parent will already take care of this
             * issue, and the call below will be O(1)
             * instead of O(n).
             */
            
            if (!removeChild(oldMotion->parent_, oldMotion)) {
                assert(worker->runtime_->threadCount_ > 1);
            }
            

            return;
        }

        /*
         * We shouldn't see a concurrent update on 1 thread,
         * unless there is something wrong with the order in
         * which rewire (see note in calling method)
         */
        assert(worker->runtime_->threadCount_ > 1);

        updatedOldMotion = node->motion_;
        assert(oldMotion != updatedOldMotion);

        oldMotion = updatedOldMotion;

        /* try again, no need to check/recalculated path cost
         * or link */
    } while (pathCost < oldMotion->pathCost_);
   
}                                                                                          
                                             
/******************************************************************************
 * Utility Functions                                                          *
 *****************************************************************************/

/**
 *
 *
 */
void ompl::geometric::pRRTstar::printConfig(const double *config, int size) 
{

    std::cout<< "State Configuration : " ;
    for (int i = 0 ; i < size; ++i ) {
       std::cout << config[i] ;
       if (i < size- 1)
          std::cout<<", ";
    }
    std::cout<< std::endl;
}

/**
 *
 *
 */
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


/**
 *
 *
 */
void ompl::geometric::pRRTstar::randomSample(Worker *worker, double *config)
{
    int dimensions = worker->system_->dimensions_;
    for (int i=dimensions ; --i>=0 ; ) {
        config[i] = worker->rng_->uniformReal(worker->system_->minConfig_[i]
                                            , worker->system_->maxConfig_[i]);

    }
}


/**
 *
 *
 */
void ompl::geometric::pRRTstar::printLog( std::string logString)
{
    std::cout << logString << std::endl;
} 

/**
 *
 *
 */
int ompl::geometric::pRRTstar::nearListCompare(const void *a, const void *b)
{
    double a_cost = ((near_motion_t*)a)->pathCost_;
    double b_cost = ((near_motion_t*)b)->pathCost_;

    return (a_cost < b_cost ? -1 : (a_cost > b_cost ? 1 : 0));
}  

/**
 *
 *
 */
void ompl::geometric::pRRTstar::getWorkerData()
{
    long sampleCount;
    for(int i = 0; i < numOfThreads_;++i) {
        sampleCount += workers_[i].sampleCount_;
    }
    printf("\nNumber of threads used: %d\n", numOfThreads_);
    printf("Number of states sampled: %ld\n",sampleCount);
    printLog("\nRoot Node :");
    printConfig(initConfig_, dimensions_);
    printLog("Best Path :");
    
    Motion *path = runtime_->bestPath_;
    int n = 0;
    Motion *ptr;

    if (path == NULL) {
        printLog("No Path found ??");
        return;
    }
    
    /*
     * if there is a target configuration, we include it in the
     * solution
     */
    if (targetConfig_ != NULL) {
        printConfig(targetConfig_,dimensions_);
        n++;
    }   
     
    for ( ptr = path ; ptr != NULL ; ptr = ptr->parent_ ) {
        printConfig(ptr->node_->getConfig(),dimensions_);
        n++;
    }
       
    printf("Best Path Length: %d\n",n);
    printf("Best Path Cost: %f\n",path->pathCost_);    

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
                
