/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, 2013 UNC Chapel Hill
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

/* Authors:  Diptorup Deb*/

#ifndef OMPL_CONTRIB_RRT_STAR_PRRTSTAR_
#define OMPL_CONTRIB_RRT_STAR_PRRTSTAR_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

#include <limits>
#include <vector>

#include "datastructures/kdtree.h"

#define REGION_SPLIT_AXIS 0
#define INITIAL_NEAR_LIST_CAPACITY 1024

namespace ompl
{
    namespace geometric
    {
    /**
      @anchor gpRRTstar
      @par Short description
      \ref gpRRTstar "PRRT*" (parallel RRT*) is a parallel implementation 
      Of the asymptotically-optimal incremental sampling-based RRT*
      algorithm. 
      \ref gpRRTstar "PRRT*" algorithm is able to achieve super-liner
      speed ups on multi-core architectures, using three key features:
      <ol>
      <li> lock-free parallelism using atomic operations to
           eliminate slowdowns caused by lock overhead and contention,</li>
      <li> partition-based sampling to reduce the size of each processor
           core’s working data set to improve cache efficiency, and ,</li>
      <li> parallel backtracking to reduce the number of rewiring steps
           performed in PRRT*. </li>
      <ol>
      <p>
      @par External documentation
      Jeffrey Ichnowski and Ron Alterovitz
      Parallel Sampling-Based Motion Planning with Superlinear Speedup
      (IROS), 2012.
      <a href="http://robotics.cs.unc.edu/publications/Ichnowski2012_IROS.pdf">
          http://robotics.cs.unc.edu/publications/Ichnowski2012_IROS.pdf
      </a>
     */
        
        /** \brief Parallel and Optimal Rapidly-exploring Random Trees */
        class pRRTstar : public base::Planner
        {
        public:

            pRRTstar(const base::SpaceInformationPtr &si);

            virtual ~pRRTstar(void);

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(
                                 const base::PlannerTerminationCondition &ptc);
                                 
            virtual void clear(void);

            /** \brief When the planner attempts to rewire the tree,git shor
             *  it does so by looking at some of the neighbors within
             *  a computed radius. The computation of that radius
             *  depends on the multiplicative factor set here.
             *  Set this parameter should be set at least to the side
             *  length of the (bounded) state space. E.g., if the state
             *  space is a box with side length L, then this parameter
             *  should be set to at least L for rapid and efficient
             *   convergence in trajectory space. 
             */
            void setBallRadiusConstant(double ballRadiusConstant)
            {
                ballRadiusConst_ = ballRadiusConstant;
            }

            /** \brief Get the multiplicative factor used in the
             *  computation of the radius whithin which tree rewiring
             *  is done. 
             */
            double getBallRadiusConstant(void) const
            {
                return ballRadiusConst_;
            }
            
            /** \brief 
             */
            void setRegionalSampling(bool regionalSampling)
            {
                regionalSampling_ = regionalSampling;
            }

            /** \brief 
             */
            bool getRegionalSampling(void) const
            {
                return regionalSampling_;
            }
            
            /** \brief 
             */
            void setSamplesPerStep(int samplesPerStep)
            {
                samplesPerStep_ = samplesPerStep;
            }

            /** \brief 
             */
            int getSamplesPerStep(void) const
            {
                return samplesPerStep_;
            }
            
            /** \brief 
             */
            void setNumOfThreads(int numOfThreads)
            {
                numOfThreads_ = numOfThreads;
            }

            /** \brief 
             */
            int getNumOfThreads(void) const
            {
                return numOfThreads_;
            }
            
            /** \brief 
             */
            virtual void setup(void);
            
            friend class Worker;
            
        protected:

            /** \brief Free the memory allocated by this planner */
            void freeMemory(void);
            
            /** \brief Shrink rate of radius the planner uses to find near 
             *   neighbors and rewire 
             */
            double                                         ballRadiusConst_;
            
            /**
             *
             */
            bool                                           regionalSampling_;
            
            /**
             *
             */
            int                                            samplesPerStep_;
            
            class Motion;
            
             /*
             * A single node in the RRT* tree.  The in_goal and config fields 
             * are constant.  The link field changes as the node is rewired to 
             * better paths.
             */
            class Node
            {
            public:
                Node(ompl::base::StateSpacePtr stateSpace, base::State *state)
                                        : stateSpace_(stateSpace),state_(state)
                {
                }
                
                Node(ompl::base::StateSpacePtr stateSpace, bool inGoal)
                                    : stateSpace_ (stateSpace), inGoal_(inGoal)
                {
                    state_ = stateSpace_->allocState();
                }                                    
                      
                ~Node()
                {
                    stateSpace_->freeState(state_);  
                }
                
                double * getConfig() const;
                void setConfig(const double *config);
                void setMotion(Motion *motion);
                
                friend class ompl::geometric::pRRTstar;
                
            private:
            
                Motion * volatile                          motion_;                
                ompl::base::StateSpacePtr                  stateSpace_;
                base::State                               *state_;
                bool                                       inGoal_;
           
            };
            
            /** \brief Representation of a motion between two nodes in the 
             *  RRT* */
            class Motion
            {
            public:

                Motion(void) : node_(NULL), parent_(NULL), motionCost_(0.0)
                             , pathCost_(0.0), firstChild_(NULL)
                             , nextSibling_(NULL)
                             
                {
                }
                
                Motion( Node *node , Motion *parent, double motionCost)
                            : node_(node), parent_(parent)
                            , motionCost_(motionCost), firstChild_(NULL)
                            , nextSibling_(NULL)
                {
                }                            

                ~Motion(void)
                {
                }

                /** \brief The state contained by the motion */
                Node                                      *node_;

                /** \brief The parent motion in the exploration tree */
                Motion                                    *parent_;

                /** \brief The cost of this motion */
                double                                     motionCost_;
                                
                double                                     pathCost_;

                /** \brief The set of motions descending from the current
                 * motion 
                 */
                /* linked list of children */
                Motion * volatile                          firstChild_; 
                /* next element in the list */
                Motion * volatile                          nextSibling_; 
            };
            
           
            
        private:
        
            /*
             * The near-list data structure built up during a call to kd_near
             */
            typedef struct near_motion {
                /* motion returned by kd_near */
                Motion *motion_; 
                /* distance from the  motion's node to near node */
                double motionCost_; 
                /* path distance through motion to near node */
                double pathCost_; 
            } near_motion_t;

        
            /** \brief The Runtime class represents the shared configuration
             * and communication parameters which are shared by the Wroker 
             * threads.
             */
            class Runtime
            {
                public:
                    Runtime( kd_tree_t * kdTree, Node *root
                           , int threadCount, int samplesPerStep ) 
                                              : kdTree_(kdTree) 
                                              , root_(root)
                                              , threadCount_(threadCount)
                                              , samplePerStep_(samplesPerStep)
                                              , done_(false)
                                              , stepNo_ (1)
                                              , bestPath_(NULL)
                    {
                    }
                    
                    ~Runtime (void)
                    {
                    }
                friend class ompl::geometric::pRRTstar; 
                   
                private:
                    /*
                     * unchanging fields are first and padded to be in a 
                     * separate cache-line than the changing fields
                     */

                    kd_tree_t *kdTree_;
                    ompl::geometric::pRRTstar::Node *root_;
                    size_t threadCount_;
                    size_t samplePerStep_;
                    
                    /*
                     * Runtime status fields, these are constantly modified and
                     * checked as more samples are added to the graph.
                     */
                    volatile bool done_;
                    volatile size_t stepNo_;  
                    
                    /*
                     * The best path field, also modified often, but in a 
                     * separate process.  It gets its own cache line too.
                     */
                                      
                    ompl::geometric::pRRTstar::Motion * volatile bestPath_;
            };
            
           
            /** \brief Representation of the Worker Threads.
             *
             * A worker struct is a per-worker-thread allocated structure.  
             * It is essentially a thread-local storage structure that contains
             * everything a worker-thread needs to do its share of the
             * computation.
             */
            class Worker
            {
                public:
                    Worker()
                    {
                    }
   
                    ~Worker(void)
                    {
                        free(nearMotion_);
                    }
                    
                    friend class ompl::geometric::pRRTstar;
                    
                private:
                
                    #ifdef _OPENMP
                        int                            threadID_;
                    #else
                        pthread_t                      threadID_;
                    #endif
                    
                    ompl::geometric::pRRTstar::Runtime *runtime_;
                    ompl::geometric::pRRTstar         *system_ ;     
                                   
                    int                                workerid_;
                    RNG                               *rng_;
                    
                    /** \todo change the newConfig_ to be of base::State type 
                    */
                    double                            *newConfig_;
                    
                    near_motion_t                     *nearMotion_;
                    size_t                             nearMotionSize_;
                    size_t                             nearMotionCapacity_;

                    const double                      *sampleMin_;
                    const double                      *sampleMax_;

                    long                               clearCount_;
                    long                               sampleCount_;
                    double                             motionDistSum_;
                   
            };
            
            // private members of pRRTstar
            
            ompl::base::StateSpacePtr                  stateSpace_;
            int                                        dimensions_;
            double                                    *initConfig_;
            double                                    *targetConfig_;
            double                                    *maxConfig_;
            double                                    *minConfig_;
            int                                        numOfThreads_;
            ompl::base::PlannerTerminationCondition    ptc_;
            
            /** \brief Worker threads which do localized sampling and identify
              *        best path for the RRT*
              */   
            Worker                                    *workers_;
           
            // private functions
            
            /******************************************************************
             * Functions to setup the threaded system and the other           *
             * primitives                                                     *
             *****************************************************************/
            
            bool setupThreadedSystem();
            
            bool setupPrimitives();
            
            void startWorkers(Worker *workers, int threadCount);
            
            /* These functions are static since they need to be passed to the
             * pthread_create callback.
             */
            static void * worker_main(void *arg);
            
            static bool workerStep(Worker *worker, int stepNo);
            
            /******************************************************************
             * Functions provided by the OMPL problem setup, used to check    *
             * things like if goal located, motion is possible/obstacle free. *
             *****************************************************************/
            
            /** \brief A helper method to wrap the SpaceInformation.isValid 
             *   function
             *
             *  This private helper function is an indirection to the 
             *  SpaceInformation.isValid(State * s) function. This is needed
             *  for the moment, since prrts represents space values as a 
             *  simple array of doubles, which first needs to be converted to
             *  ompl::base::State.
             */
            bool isValid (const double *config);
            
            /** \brief A helper method to wrap the SpaceInformation.checkMotion 
             *   function
             *
             *  This private helper function is an indirection to the 
             *  SpaceInformation.checkMotion(State * s1, State * s2) function. 
             *  This is needed for the moment, since prrts represents space 
             *  values as an array of doubles, which first needs to be 
             *  converted to ompl::base::State.
             */
            bool checkMotion (const double *config1, const double *config2);         
            
            /** \brief A helper method to wrap the Goal.isSatisfied
             *   function
             *
             *  This private helper function is an indirection to the 
             *  Goal.isSatisfied(State * s1) function. 
             *  This is needed for the moment, since prrts represents space 
             *  values as an array of doubles, which first needs to be 
             *  converted to ompl::base::State.
             */
            bool isSatisfied (const double *config1);      
            
            /** \brief Compute distance between states. We need this static 
              * function to callback from the kd_tree.c. 
             */
 
            static double prrts_dist_func(void * usrPtr, const double *config1
                                        , const double * config2);

            /** \brief Compute distance between states  
             */
            double distanceFunction(const double *config1
                                  , const double *config2);   
            
            /** \brief Check if the PlannerTerminationCondition.operator()
             *   is true.
             */
            bool checkPlannerTermCond();   
            
            /******************************************************************
             * Utility Functions                                              *
             *****************************************************************/
            
            #ifdef _OPENMP
            #define get_num_procs() omp_get_max_threads()
            #else   
            int get_num_procs();
            #endif
            
            static void printConfig(double *config);       
            
            /******************************************************************
             * Functions needed by the pRRT* execution                        *
             *****************************************************************/
             
            /** \brief Callback function which is passed to the kd_tree to 
             *  build the near-list (Worker.nearMotion_) data structure 
             *  during a call to kd_near.
             * 
             */
            static void worker_near_callback(void *workerArg, int no
                                           , void *nearNode, double dist);
                                           
            bool can_link(Worker *worker, const double *a
                        , const double *b, double motionCost);

            bool is_link_expired(Motion *motion);
            
            void update_children(Worker *worker, Motion *newParent
                               , Motion *oldParent, double radius);                                      
            

            Motion * remove_first_chld(Motion *parent);
            
            bool remove_child(Motion *parent , Motion *child);
            
            void steer(size_t dimensions, double *newConfig
                     , const double *targetConfig, double scale);
                            
            Node * create_node(pRRTstar *prrts_system, double *config
                             , bool inGoal, double linkCost
                             , Motion *parentMotion);     

            Motion * set_node_link(Worker *worker
                                 , Node *node
                                 , Motion *oldMotion
                                 , double motionCost
                                 , Motion *parentMotion);                             
                             
            Motion * create_link(Node *node
                               , double linkCost
                               , Motion *parent);   
                               
            void add_child_link(Motion *parent, Motion *child);       
            
            void update_best_path(Worker *worker, Motion *motion
                                                , double radius);    

            static int near_list_compare(const void *a, const void *b); 
            
            void rewire(Worker *worker, Motion *oldMotion, double motionCost
                                      , Node *newParent, double radius);

         };
    }
}

#endif
