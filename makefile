KDTREE_ROOT= datastructures
KDTREE_SRC = $(KDTREE_ROOT)/src
CC     = g++
CFLAGS = -c -Wall -Wextra
LDFLAGS=
SOURCES=

all     : alloc kdtree PRRTstar

alloc.o :
	$(CC) $(CFLAGS) $(KDTREE_SRC)/alloc.c -o $@ -I$(KDTREE_ROOT)
kdtree.o: alloc.o
	$(CC) $(CFLAGS) $(KDTREE_SRC)/kdtree.c -o $@ -I$(KDTREE_ROOT)
PRRTstar: kdtree 

clean   :
	rm -rf *o plannerApp
    


