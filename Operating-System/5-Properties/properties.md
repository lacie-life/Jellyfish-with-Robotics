# Operating System - Properties

## Batch processing
Batch processing is a technique in which an Operating System collects the programs and data together in a batch before processing starts. An operating system does the following activities related to batch processing −
+ The OS defines a job which has predefined sequence of commands, programs and data as a single unit.
+ The OS keeps a number a jobs in memory and executes them without any manual information.
+ Jobs are processed in the order of submission, i.e., first come first served fashion.
+ When a job completes its execution, its memory is released and the output for the job gets copied into an output spool for later printing or processing.

![Figure 1](https://github.com/lacie-life/Robot/blob/master/Operating-System/5-Properties/batch_processing.jpg?raw=true)

### Advantages
+ Batch processing takes much of the work of the operator to the computer.
+ Increased performance as a new job get started as soon as the previous job is finished, without any manual intervention.

### Disadvantages
+ Difficult to debug program.
+ A job could enter an infinite loop.
+ Due to lack of protection scheme, one batch job can affect pending jobs.

## Multitasking
Multitasking is when multiple jobs are executed by the CPU simultaneously by switching between them. Switches occur so frequently that the users may interact with each program while it is running. An OS does the following activities related to multitasking −
+ The user gives instructions to the operating system or to a program directly, and receives an immediate response.
+ The OS handles multitasking in the way that it can handle multiple operations/executes multiple programs at a time.
+ Multitasking Operating Systems are also known as Time-sharing systems.
+ These Operating Systems were developed to provide interactive use of a computer system at a reasonable cost.
+ A time-shared operating system uses the concept of CPU scheduling and multiprogramming to provide each user with a small portion of a time-shared CPU.
+ Each user has at least one separate program in memory.

![Figure 2](https://github.com/lacie-life/Robot/blob/master/Operating-System/5-Properties/multitasking.jpg?raw=true)

+ A program that is loaded into memory and is executing is commonly referred to as a process.
+ When a process executes, it typically executes for only a very short time before it either finishes or needs to perform I/O.
+ Since interactive I/O typically runs at slower speeds, it may take a long time to complete. During this time, a CPU can be utilized by another process.
+ The operating system allows the users to share the computer simultaneously. Since each action or command in a time-shared system tends to be short, only a little CPU time is needed for each user.
+ As the system switches CPU rapidly from one user/program to the next, each user is given the impression that he/she has his/her own CPU, whereas actually one CPU is being shared among many users.

## Multiprogramming
Sharing the processor, when two or more programs reside in memory at the same time, is referred as multiprogramming. Multiprogramming assumes a single shared processor. Multiprogramming increases CPU utilization by organizing jobs so that the CPU always has one to execute.

The following figure shows the memory layout for a multiprogramming system.

![Figure 3](https://github.com/lacie-life/Robot/blob/master/Operating-System/5-Properties/memory_layout.jpg?raw=true)

An OS does the following activities related to multiprogramming.
+ The operating system keeps several jobs in memory at a time.
+ This set of jobs is a subset of the jobs kept in the job pool.
+ The operating system picks and begins to execute one of the jobs in the memory.
+ Multiprogramming operating systems monitor the state of all active programs and system resources using memory management programs to ensures that the CPU is never idle, unless there are no jobs to process.

### Advantages
+ High and efficient CPU utilization.
+ User feels that many programs are allotted CPU almost simultaneously.

### Disadvantages
+ CPU scheduling is required.
+ To accommodate many jobs in memory, memory management is required.

## Interactivity
Interactivity refers to the ability of users to interact with a computer system. An Operating system does the following activities related to interactivity −
+ Provides the user an interface to interact with the system.
+ Manages input devices to take inputs from the user. For example, keyboard.
+ Manages output devices to show outputs to the user. For example, Monitor.
The response time of the OS needs to be short, since the user submits and waits for the result.

## Real Time System
Real-time systems are usually dedicated, embedded systems. An operating system does the following activities related to real-time system activity.

+ In such systems, Operating Systems typically read from and react to sensor data.
+ The Operating system must guarantee response to events within fixed periods of time to ensure correct performance.

## Distributed Environment
A distributed environment refers to multiple independent CPUs or processors in a computer system. An operating system does the following activities related to distributed environment −
+ The OS distributes computation logics among several physical processors.
+ The processors do not share memory or a clock. Instead, each processor has its own local memory.
+ The OS manages the communications between the processors. They communicate with each other through various communication lines.

## Spooling
Spooling is an acronym for simultaneous peripheral operations on line. Spooling refers to putting data of various I/O jobs in a buffer. This buffer is a special area in memory or hard disk which is accessible to I/O devices.

An operating system does the following activities related to distributed environment −

+ Handles I/O device data spooling as devices have different data access rates.
+ Maintains the spooling buffer which provides a waiting station where data can rest while the slower device catches up.
+ Maintains parallel computation because of spooling process as a computer can perform I/O in parallel fashion. It becomes possible to have the computer read data from a tape, write data to disk and to write out to a tape printer while it is doing its computing task.

![Figure 4](https://github.com/lacie-life/Robot/blob/master/Operating-System/5-Properties/spooling.jpg?raw=true)

### Advantages
+ The spooling operation uses a disk as a very large buffer.
+ Spooling is capable of overlapping I/O operation for one job with processor operations for another job.