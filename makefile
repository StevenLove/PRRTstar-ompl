UTILS_ROOT  = utils
UTILS_SRC   = $(UTILS_ROOT)/src
KDTREE_ROOT = datastructures
KDTREE_SRC  = $(KDTREE_ROOT)/src
INCLUDE_DIR = /usr/local/include
LIB_DIR     = /usr/local/lib  
OBJS        = alloc.o kdtree.o prrts.o PRRTstar.o RigidBodyPlanning.o
LIBS        = -lpthread -lompl 
CC          = g++
CFLAGS      = -c -Wall -Wextra
LDFLAGS     =  


plannerApp: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@ -I$(INCLUDE_DIR):. -L$(LIB_DIR) $(LIBS)

alloc.o   :
	$(CC) $(CFLAGS) $(UTILS_SRC)/alloc.c -o $@ -I$(UTILS_ROOT)
	
kdtree.o  : alloc.o
	$(CC) $(CFLAGS) $(KDTREE_SRC)/kdtree.c -o $@ -I$(KDTREE_ROOT)
	
prrts.o  : alloc.o kdtree.o
	$(CC) $(CFLAGS) prrts.c -o $@ 
	
PRRTstar.o: kdtree.o prrts.o
	$(CC) $(CFLAGS) PRRTstar.cpp -o $@ 
	
RigidBodyPlanning.o: PRRTstar.o
	$(CC) $(CFLAGS) RigidBodyPlanning.cpp -o $@ 	
	
clean   :
	rm -rf *o plannerApp
	

