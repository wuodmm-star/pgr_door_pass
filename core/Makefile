# Build the C core without ROS:  make && ./test_pgr_door
CC ?= cc
CFLAGS ?= -O2 -std=c99 -Wall -Wextra -Iinclude
all: libpgr_door.a test_pgr_door
libpgr_door.a: src/pgr_door.c include/pgr_door.h
	$(CC) $(CFLAGS) -c src/pgr_door.c -o pgr_door.o && ar rcs $@ pgr_door.o
test_pgr_door: test/test_core.c libpgr_door.a
	$(CC) $(CFLAGS) test/test_core.c libpgr_door.a -lm -o $@
clean:
	rm -f pgr_door.o libpgr_door.a test_pgr_door
