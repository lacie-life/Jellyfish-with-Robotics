# Overview

An Operating System (OS) is an interface between a computer user and computer hardware. An operating system is a software which performs all the basic tasks like file management, memory management, process management, handling input and output, and controlling peripheral devices such as disk drives and printers.

Some popular Operating Systems include Linux Operating System, Windows Operating System, VMS, OS/400, AIX, z/OS, etc.

## Definition 
An operating system is a program that acts as an interface between the user and the computer hardware and controls the execution of all kinds of programs.

![Figure 1](https://github.com/lacie-life/Robot/blob/master/Operating-System/2-Overview/conceptual_view.jpg?raw=true)

Following are some of important functions of an operating System.

+ Memory Management
+ Processor Management
+ Device Management
+ File Management
+ Security
+ Control over system performance
+ Job accounting
+ Error detecting aids
+ Coordination between other software and users

## Memory Management
Memory management refers to management of Primary Memory or Main Memory. Main memory is a large array of words or bytes where each word or byte has its own address.

Main memory provides a fast storage that can be accessed directly by the CPU. For a program to be executed, it must in the main memory. An Operating System does the following activities for memory management −

+ Keeps tracks of primary memory, i.e., what part of it are in use by whom, what part are not in use.

+ In multiprogramming, the OS decides which process will get memory when and how much.

+ Allocates the memory when a process requests it to do so.

+ De-allocates the memory when a process no longer needs it or has been terminated.

## Processor Management
In multiprogramming environment, the OS decides which process gets the processor when and for how much time. This function is called process scheduling. An Operating System does the following activities for processor management −

+ Keeps tracks of processor and status of process. The program responsible for this task is known as traffic controller.

+ Allocates the processor (CPU) to a process.

+ De-allocates processor when a process is no longer required

## Device Management
An Operating System manages device communication via their respective drivers. It does the following activities for device management −

+ Keeps tracks of all devices. Program responsible for this task is known as the I/O controller.

+ Decides which process gets the device when and for how much time.

+ Allocates the device in the efficient way.

+ De-allocates devices.

## File Management
A file system is normally organized into directories for easy navigation and usage. These directories may contain files and other directions.

An Operating System does the following activities for file management −

+ Keeps track of information, location, uses, status etc. The collective facilities are often known as file system.

+ Decides who gets the resources.

+ Allocates the resources.

+ De-allocates the resources.

## Other Important Activities
Following are some of the important activities that an Operating System performs −

1. Security − By means of password and similar other techniques, it prevents unauthorized access to programs and data.

2. Control over system performance − Recording delays between request for a service and response from the system.

3. Job accounting − Keeping track of time and resources used by various jobs and users.

4. Error detecting aids − Production of dumps, traces, error messages, and other debugging and error detecting aids.

5. Coordination between other softwares and users − Coordination and assignment of compilers, interpreters, assemblers and other software to the various users of the computer systems.