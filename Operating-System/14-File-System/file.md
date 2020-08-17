# Operating System - File System

## File
A file is a named collection of related information that is recorded on secondary storage such as magnetic disks, magnetic tapes and optical disks. In general, a file is a sequence of bits, bytes, lines or records whose meaning is defined by the files creator and user.

## File Structure
A File Structure should be according to a required format that the operating system can understand.
+ A file has a certain defined structure according to its type.
+ A text file is a sequence of characters organized into lines.
+ A source file is a sequence of procedures and functions.
+ An object file is a sequence of bytes organized into blocks that are understandable by the machine.
When operating system defines different file structures, it also contains the code to support these file structure. Unix, MS-DOS support minimum number of file structure.

## File Type
File type refers to the ability of the operating system to distinguish different types of file such as text files source files and binary files etc. Many operating systems support many types of files. Operating system like MS-DOS and UNIX have the following types of files −

### Ordinary files
+ These are the files that contain user information.
+ These may have text, databases or executable program.
+ The user can apply various operations on such files like add, modify, delete or even remove the entire file.

### Directory files
+ These files contain list of file names and other information related to these files.

### Special files
+ These files are also known as device files.
+ These files represent physical device like disks, terminals, printers, networks, tape drive etc.

### These files are of two types −
+ Character special files − data is handled character by character as in case of terminals or printers.
+ Block special files − data is handled in blocks as in the case of disks and tapes.

## File Access Mechanisms
File access mechanism refers to the manner in which the records of a file may be accessed. There are several ways to access files −
+ Sequential access
+ Direct/Random access
+ Indexed sequential access

### Sequential access
A sequential access is that in which the records are accessed in some sequence, i.e., the information in the file is processed in order, one record after the other. This access method is the most primitive one. Example: Compilers usually access files in this fashion.

### Direct/Random access
+ Random access file organization provides, accessing the records directly.
+ Each record has its own address on the file with by the help of which it can be directly accessed for reading or writing.
+ The records need not be in any sequence within the file and they need not be in adjacent locations on the storage medium.

### Indexed sequential access
+ This mechanism is built up on base of sequential access.
+ An index is created for each file which contains pointers to various blocks.
+ Index is searched sequentially and its pointer is used to access the file directly.

## Space Allocation
Files are allocated disk spaces by operating system. Operating systems deploy following three main ways to allocate disk space to files.
+ Contiguous Allocation
+ Linked Allocation
+ Indexed Allocation

### Contiguous Allocation
+ Each file occupies a contiguous address space on disk.
+ Assigned disk address is in linear order.
+ Easy to implement.
+ External fragmentation is a major issue with this type of allocation technique.

### Linked Allocation
+ Each file carries a list of links to disk blocks.
+ Directory contains link / pointer to first block of a file.
+ No external fragmentation
+ Effectively used in sequential access file.
+ Inefficient in case of direct access file.

### Indexed Allocation
+ Provides solutions to problems of contiguous and linked allocation.
+ A index block is created having all pointers to files.
+ Each file has its own index block which stores the addresses of disk space occupied by the file.
+ Directory contains the addresses of index blocks of files.