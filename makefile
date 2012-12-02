UTILS_ROOT  = utils
UTILS_SRC   = $(UTILS_ROOT)/src
KDTREE_ROOT = datastructures
KDTREE_SRC  = $(KDTREE_ROOT)/src
INCLUDE_DIR = /usr/local/include
LIB_DIR     = /usr/local/lib  
OBJS        = alloc.o kdtree.o mt19937a.o hrtimer.o prrts.o stats.o PRRTstar.o RigidBodyPlanning.o 
LIBS        = -lpthread -lrt -lompl 
CC          = g++
CFLAGS      = -c -Wall -Wextra
LDFLAGS     =  

plannerApp: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@ -I$(INCLUDE_DIR):. -L$(LIB_DIR) $(LIBS)

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
		
prrts.o  : alloc.o kdtree.o mt19937a.o hrtimer.o stats.o
	$(CC) $(CFLAGS) prrts.c -o $@ 
	
PRRTstar.o: kdtree.o prrts.o
	$(CC) $(CFLAGS) PRRTstar.cpp -o $@ 
	
RigidBodyPlanning.o: PRRTstar.o
	$(CC) $(CFLAGS) RigidBodyPlanning.cpp -o $@ 	
	
clean   :
	rm -rf *o plannerApp
	

