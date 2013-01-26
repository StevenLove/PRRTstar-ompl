/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Diptorup Deb
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

#ifndef OMPL_CONTRIB_RRT_STAR_PPRRTSTAR_
#define OMPL_CONTRIB_RRT_STAR_PPRRTSTAR_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

#include <limits>
#include <vector>

extern "C"{
#include "prrts.h"
}
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
                                 
            /**\todo - Adding this solve method since the prrts datastructires
             * are not yet integrated with the ompl api, to be accessible to
             * create a PlannerTerminationCondition
             */
            virtual base::PlannerStatus solveForSamples(size_t sampleCount);                                      

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
            
            
            
        protected:


            /** \brief Free the memory allocated by this planner */
            void freeMemory(void);

            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief The random number generator */
            RNG                                            rng_;
            
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

        private:
            prrts_system_t * prrtsSystem_;
            prrts_options_t options_;
            prrts_solution_t * solution_;
            ompl::base::StateSpacePtr stateSpace_;
            int dimensions_;
            double *init_config_;
            double *target_config_;
            double *max_config_;
            double *min_config_;
            int numOfThreads_;
            ompl::base::PlannerTerminationCondition  ptc_;
            
            /** \brief Convinience function to initialize the prrts_system_t 
             *  struct needed by the prrts C implementation.
             *
             */
            bool setupPrrtsSystem ();
            
            /** \brief Setup the prrts_options_t data structure needed by the
             *   prrts C implementation.
             */
            
            void setupPrrtsOptions ();
              
            /** \brief Hack to get around type conversion issue in C callback
             *  the prrts_clear_func encapsulates the isValid() function.
             *
             *  Since a C++ member function can not be simply passes to a
             *  C function pointer, this detour is taken to rather do the 
             *  callback via a static function. The caller then passes the 
             *  'this' object to actually invoke the member function.
             *  So essentially the function calls the object's isValid() 
             *  state validity checker.
             */             
            static bool prrts_clear_func(void * usrPtr, const double *config);
            
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
            
            /** \brief Hack to get around type conversion issue in C callback
             *  the prrts_link_func encapsulates the checkMotion() function.
             *
             *  Since a C++ member function can not be simply passes to a
             *  C function pointer, this detour is taken to rather do the 
             *  callback via a static function. The caller then passes the 
             *  'this' object to actually invoke the member function.
             *  So essentially the function calls the object's checkMotion() 
             *  motion validator, to validate the motion between two states.
             */             
            static bool prrts_link_func (void * usrPtr, const double *config1
                                                      , const double *config2);

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
            
            /** \brief Hack to get around type conversion issue in C callback
             *  the prrts_in_goal_func encapsulates the isSatisfied() function.
             *
             *  Since a C++ member function can not be simply passed to a
             *  C function pointer, this detour is taken to rather do the 
             *  callback via a static function. The caller then passes the 
             *  'this' object to actually invoke the member function.
             *  So essentially the function calls the object's checkMotion() 
             *  motion validator, to validate the motion between two states.
             */            
            static bool prrts_in_goal (void * usrPtr, const double *config);

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
            
            /** \brief Compute distance between states  
             */
 
            static double prrts_dist_func(void * usrPtr, const double *config1
                                        , const double * config2);

            /** \brief Compute distance between states  
             */
            double distanceFunction(const double *config1
                                  , const double *config2);   
            
            /** \brief Check if the PlannerTerminationCondition is satisfied  
             */
 
            static bool ompl_planner_term_cond(void * usrPtr);

            /** \brief Check if the PlannerTerminationCondition.operator()
             *   is true.
             */
            bool checkPlannerTermCond();   
            
            bool addPathToSolution();                                
                                  
            #ifdef _OPENMP
            #define get_num_procs() omp_get_max_threads()
            #else   
            int get_num_procs();
            #endif
         };
         
    }
}

#endif
