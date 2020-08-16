# Operating System - Processes

## Process
A process is basically a program in execution. The execution of a process must progress in a sequential fashion.

A process is defined as an entity which represents the basic unit of work to be implemented in the system.

To put it in simple terms, we write our computer programs in a text file and when we execute this program, it becomes a process which performs all the tasks mentioned in the program.

When a program is loaded into the memory and it becomes a process, it can be divided into four sections ─ stack, heap, text and data. The following image shows a simplified layout of a process inside main memory −

![Figure 1](https://github.com/lacie-life/Robot/blob/master/Operating-System/6-Processes/process_components.jpg?raw=true)

|S.N.|Components & Description|
|----|------------------------|
|1|Stack: The process Stack contains the temporary data such as method/function parameters, return address and local variables.|
|2|Heap: This is dynamically allocated memory to a process during its run time.|
|3|Text: This includes the current activity represented by the value of Program Counter and the contents of the processor's registers.|
|4|Data: This section contains the global and static variables.|

## Program
A program is a piece of code which may be a single line or millions of lines. A computer program is usually written by a computer programmer in a programming language. For example, here is a simple program written in C programming language −

#include <stdio.h>

int main() {
   printf("Hello, World! \n");
   return 0;
}

A computer program is a collection of instructions that performs a specific task when executed by a computer. When we compare a program with a process, we can conclude that a process is a dynamic instance of a computer program.

A part of a computer program that performs a well-defined task is known as an algorithm. A collection of computer programs, libraries and related data are referred to as a software.

## Process Life Cycle
When a process executes, it passes through different states. These stages may differ in different operating systems, and the names of these states are also not standardized.

In general, a process can have one of the following five states at a time.

|S.N.|State & Description|
|----|-------------------|
|1|Start: This is the initial state when a process is first started/created.|
|2|Ready: The process is waiting to be assigned to a processor. Ready processes are waiting to have the processor allocated to them by the operating system so that they can run. Process may come into this state after Start state or while running it by but interrupted by the scheduler to assign CPU to some other process.|
|3|Running: Once the process has been assigned to a processor by the OS scheduler, the process state is set to running and the processor executes its instructions.|
|4|Waiting: Process moves into the waiting state if it needs to wait for a resource, such as waiting for user input, or waiting for a file to become available.|
|5|Terminated or Exit: Once the process finishes its execution, or it is terminated by the operating system, it is moved to the terminated state where it waits to be removed from main memory.|

![Figure 2](https://github.com/lacie-life/Robot/blob/master/Operating-System/6-Processes/process_state.jpg?raw=true)

## Process Control Block (PCB)
A Process Control Block is a data structure maintained by the Operating System for every process. The PCB is identified by an integer process ID (PID). A PCB keeps all the information needed to keep track of a process as listed below in the table −

|S.N.|Information & Description|
|----|-------------------------|
|1|Process State: The current state of the process i.e., whether it is ready, running, waiting, or whatever.|
|2|	Process privileges: This is required to allow/disallow access to system resources.|
|3|Process ID: Unique identification for each of the process in the operating system.|
|4|	Pointer: A pointer to parent process.|
|5|Program Counter: Program Counter is a pointer to the address of the next instruction to be executed for this process.|
|6|	CPU registers: Various CPU registers where process need to be stored for execution for running state.|
|7|CPU Scheduling Information: Process priority and other scheduling information which is required to schedule the process.|
|8|Memory management information: This includes the information of page table, memory limits, Segment table depending on memory used by the operating system.|
|9|Accounting information: This includes the amount of CPU used for process execution, time limits, execution ID etc.|
|10|IO status information: This includes a list of I/O devices allocated to the process.|

The architecture of a PCB is completely dependent on Operating System and may contain different information in different operating systems. Here is a simplified diagram of a PCB −

![Figure 3](https://github.com/lacie-life/Robot/blob/master/Operating-System/6-Processes/pcb.jpg?raw=true)

The PCB is maintained for a process throughout its lifetime, and is deleted once the process terminates.