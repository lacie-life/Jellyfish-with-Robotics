# Defining Dependencies in Makefile

It is very common that a final binary will be dependent on various source code and source header files. Dependencies are important because they let the make Known about the source for any target. Consider the following example −

hello: main.o factorial.o hello.o
   $(CC) main.o factorial.o hello.o -o hello

Here, we tell the make that hello is dependent on main.o, factorial.o, and hello.o files. Hence, whenever there is a change in any of these object files, make will take action.

At the same time, we need to tell the make how to prepare .o files. Hence we need to define those dependencies also as follows −

main.o: main.cpp functions.h
   $(CC) -c main.cpp

factorial.o: factorial.cpp functions.h
   $(CC) -c factorial.cpp

hello.o: hello.cpp functions.h
   $(CC) -c hello.cpp