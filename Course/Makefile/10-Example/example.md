# Makefile - Example

This is an example of the Makefile for compiling the hello program. This program consists of three files main.cpp, factorial.cpp and hello.cpp.

# Define required macros here
SHELL = /bin/sh

OBJS =  main.o factorial.o hello.o
CFLAG = -Wall -g
CC = gcc
INCLUDE =
LIBS = -lm

hello:${OBJ}
   ${CC} ${CFLAGS} ${INCLUDES} -o $@ ${OBJS} ${LIBS}

clean:
   -rm -f *.o core *.core

.cpp.o:
   ${CC} ${CFLAGS} ${INCLUDES} -c $<
Now you can build your program hello using the make. If you will issue a command make clean then it removes all the object files and core files available in the current directory.