UTILS_ROOT            = utils
UTILS_SRC             = $(UTILS_ROOT)/src
KDTREE_ROOT           = datastructures
KDTREE_SRC            = $(KDTREE_ROOT)/src
OMPL_HOME             = /home/diptorupd/Desktop/OMPL
INCLUDE_DIR_OMPL_11.1 = $(OMPL_HOME)/ompl-0.11.1/include
LIB_DIR_OMPL_11.1     = $(OMPL_HOME)/ompl-0.11.1/lib
INCLUDE_DIR_OMPL_12.1 = $(OMPL_HOME)/ompl-0.12.1/include
LIB_DIR_OMPL_12.1     = $(OMPL_HOME)/ompl-0.12.1/lib
OBJS                  = alloc.o kdtree.o mt19937a.o hrtimer.o prrts.o stats.o pRRTstar.o RigidBodyPlanning.o 
LIBS                  = -lpthread -lrt -lompl 
CC                    = g++
CFLAGS                = -c -Wall -Wextra -ansi -pedantic -g 
LDFLAGS               =  

plannerApp: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@ -I$(INCLUDE_DIR):. -L$(LIB_DIR) $(LIBS)
	
checkOMPLversion:
ifeq ($(OMPL_VERSION),'0_12_1')
INCLUDE_DIR = $(INCLUDE_DIR_OMPL_12.1)
LIB_DIR     = $(LIB_DIR_OMPL_12.1)
else
INCLUDE_DIR = $(INCLUDE_DIR_OMPL_11.1)
LIB_DIR     = $(LIB_DIR_OMPL_11.1)
endif		

alloc.o   :
	$(CC) $(CFLAGS) $(UTILS_SRC)/alloc.c -o $@ -I$(UTILS_ROOT)
	
kdtree.o  : alloc.o
	$(CC) $(CFLAGS) $(KDTREE_SRC)/kdtree.c -o $@ -I$(KDTREE_ROOT)

mt19937a.o: 
	$(CC) $(CFLAGS) $(UTILS_SRC)/mt19937a.c -o $@ -I$(UTILS_ROOT)

hrtimer.o:
	$(CC) $(CFLAGS) $(UTILS_SRC)/hrtimer.c -o $@ -I$(UTILS_ROOT)	

# g++ does not compile without the extra flags. throwing error: ‘INT64_MAX’ was not declared in this scope
stats.o: hrtimer.o
	$(CC) $(CFLAGS) $(UTILS_SRC)/stats.c -o $@ -I$(UTILS_ROOT)	-D __STDC_LIMIT_MACROS -D __STDC_FORMAT_MACROS
		
prrts.o  : checkOMPLversion alloc.o kdtree.o mt19937a.o hrtimer.o stats.o
	$(CC) $(CFLAGS) prrts.c -o $@ 
	
pRRTstar.o: kdtree.o prrts.o
	$(CC) $(CFLAGS) pRRTstar.cpp -o $@ -I$(INCLUDE_DIR)
	
RigidBodyPlanning.o: pRRTstar.o
	$(CC) $(CFLAGS) RigidBodyPlanning.cpp -o $@ -I$(INCLUDE_DIR)
	
clean   :
	rm -rf *o plannerApp
	

