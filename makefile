KDTREE_ROOT= datastructures
KDTREE_SRC = $(KDTREE_ROOT)/src
OBJS   = alloc.o kdtree.o PRRTstar.o RigidBodyPlanning.o
CC     = g++
CFLAGS = -c -Wall -Wextra
LDFLAGS= -I/usr/local/include:. -L/usr/local/lib  -lpthread -lompl 

plannerApp: $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

alloc.o   :
	$(CC) $(CFLAGS) $(KDTREE_SRC)/alloc.c -o $@ -I$(KDTREE_ROOT)
	
kdtree.o  : alloc.o
	$(CC) $(CFLAGS) $(KDTREE_SRC)/kdtree.c -o $@ -I$(KDTREE_ROOT)
	
PRRTstar.o: kdtree.o
	$(CC) $(CFLAGS) PRRTstar.cpp -o $@ 
	
RigidBodyPlanning.o: PRRTstar.o
	$(CC) $(CFLAGS) RigidBodyPlanning.cpp -o $@ 	
	
clean   :
	rm -rf *o plannerApp
	

