# Operating System - Virtual Memory

A computer can address more memory than the amount physically installed on the system. This extra memory is actually called virtual memory and it is a section of a hard disk that's set up to emulate the computer's RAM.

The main visible advantage of this scheme is that programs can be larger than physical memory. Virtual memory serves two purposes. First, it allows us to extend the use of physical memory by using disk. Second, it allows us to have memory protection, because each virtual address is translated to a physical address.

Following are the situations, when entire program is not required to be loaded fully in main memory.
+ User written error handling routines are used only when an error occurred in the data or computation.
+ Certain options and features of a program may be used rarely.
+ Many tables are assigned a fixed amount of address space even though only a small amount of the table is actually used.
+ The ability to execute a program that is only partially in memory would counter many benefits.
+ Less number of I/O would be needed to load or swap each user program into memory.
+ A program would no longer be constrained by the amount of physical memory that is available.
+ Each user program could take less physical memory, more programs could be run the same time, with a corresponding increase in CPU utilization and throughput.

Modern microprocessors intended for general-purpose use, a memory management unit, or MMU, is built into the hardware. The MMU's job is to translate virtual addresses into physical addresses. A basic example is given below −

![Figure 1](https://github.com/lacie-life/Robot/blob/master/Operating-System/11-Virtual-Memory/virtual_memory.jpg?raw=true)

Virtual memory is commonly implemented by demand paging. It can also be implemented in a segmentation system. Demand segmentation can also be used to provide virtual memory.

## Demand Paging

A demand paging system is quite similar to a paging system with swapping where processes reside in secondary memory and pages are loaded only on demand, not in advance. When a context switch occurs, the operating system does not copy any of the old program’s pages out to the disk or any of the new program’s pages into the main memory Instead, it just begins executing the new program after loading the first page and fetches that program’s pages as they are referenced.

![Figure 2](https://github.com/lacie-life/Robot/blob/master/Operating-System/11-Virtual-Memory/demand_paging.jpg?raw=true)

While executing a program, if the program references a page which is not available in the main memory because it was swapped out a little ago, the processor treats this invalid memory reference as a page fault and transfers control from the program to the operating system to demand the page back into the memory.

### Advantages
Following are the advantages of Demand Paging −
+ Large virtual memory.
+ More efficient use of memory.
+ There is no limit on degree of multiprogramming.

### Disadvantages
+ Number of tables and the amount of processor overhead for handling page interrupts are greater than in the case of the simple paged management techniques.

## Page Replacement Algorithm
Page replacement algorithms are the techniques using which an Operating System decides which memory pages to swap out, write to disk when a page of memory needs to be allocated. Paging happens whenever a page fault occurs and a free page cannot be used for allocation purpose accounting to reason that pages are not available or the number of free pages is lower than required pages.

When the page that was selected for replacement and was paged out, is referenced again, it has to read in from disk, and this requires for I/O completion. This process determines the quality of the page replacement algorithm: the lesser the time waiting for page-ins, the better is the algorithm.

A page replacement algorithm looks at the limited information about accessing the pages provided by hardware, and tries to select which pages should be replaced to minimize the total number of page misses, while balancing it with the costs of primary storage and processor time of the algorithm itself. There are many different page replacement algorithms. We evaluate an algorithm by running it on a particular string of memory reference and computing the number of page faults,

## Reference String
The string of memory references is called reference string. Reference strings are generated artificially or by tracing a given system and recording the address of each memory reference. The latter choice produces a large number of data, where we note two things.
+ For a given page size, we need to consider only the page number, not the entire address.
+ If we have a reference to a page p, then any immediately following references to page p will never cause a page fault. Page p will be in memory after the first reference; the immediately following references will not fault.
+ For example, consider the following sequence of addresses − 123,215,600,1234,76,96
+ If page size is 100, then the reference string is 1,2,6,12,0,0

### First In First Out (FIFO) algorithm
+ Oldest page in main memory is the one which will be selected for replacement.
+ Easy to implement, keep a list, replace pages from the tail and add new pages at the head.

![Figure 3](https://github.com/lacie-life/Robot/blob/master/Operating-System/11-Virtual-Memory/fifo.jpg?raw=true)

### Optimal Page algorithm

+ An optimal page-replacement algorithm has the lowest page-fault rate of all algorithms. An optimal page-replacement algorithm exists, and has been called OPT or MIN.
+ Replace the page that will not be used for the longest period of time. Use the time when a page is to be used.

![Figure 4](https://github.com/lacie-life/Robot/blob/master/Operating-System/11-Virtual-Memory/opr.jpg?raw=true)

### Least Recently Used (LRU) algorithm

+ Page which has not been used for the longest time in main memory is the one which will be selected for replacement.
+ Easy to implement, keep a list, replace pages by looking back into time.

![Figure 5](https://github.com/lacie-life/Robot/blob/master/Operating-System/11-Virtual-Memory/lru.jpg?raw=true)

### Page Buffering algorithm
+ To get a process start quickly, keep a pool of free frames.
+ On page fault, select a page to be replaced.
+ Write the new page in the frame of free pool, mark the page table and restart the process.
+ Now write the dirty page out of disk and place the frame holding replaced page in free pool.

### Least frequently Used(LFU) algorithm
+ The page with the smallest count is the one which will be selected for replacement.
+ This algorithm suffers from the situation in which a page is used heavily during the initial phase of a process, but then is never used again.

### Most frequently Used(MFU) algorithm
+ This algorithm is based on the argument that the page with the smallest count was probably just brought in and has yet to be used.

