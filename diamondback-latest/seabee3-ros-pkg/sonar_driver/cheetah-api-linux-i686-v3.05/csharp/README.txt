                   Total Phase Cheetah Sample Code
                   -------------------------------

Introduction
------------
This directory contains examples that use the Cheetah Rosetta .NET
language bindings.

The file cheetah.cs is the C# binding that will be compiled and
integrated into the application being developed.


Contents
--------
- Makefile for Windows, Linux, and Darwin
- Visual Studio solution for Visual Studio .NET 2003 and above

See top level EXAMPLES.txt for descriptions of each example.


Build Instructions
------------------
In each case, PROGRAM can be any of the sample programs, without the
'.cs' extension.

Windows Visual Studio .NET 2003 or later:
1) Copy all the .cs files into the VisualStudio directory
2) Place cheetah.dll in the PATH
3) Open VisualStudio/Examples.sln
4) Build | Build Solution
5) VisualStudio/bin/Debug/PROGRAM

Windows Makefile:
1) Install mingw32-make
   The latest version can be downloaded from the MinGW website: 
   http://www.mingw.org/
2) Install MSYS
   The latest version can be downloaded from the MinGW website: 
   http://www.mingw.org/
3) Make sure that 'CSC=csc' and 'OPT=/' is uncommented in the Makefile
4) Type 'make' at the command line.  This will build the examples and
   also copy cheetah.dll to _output.
5) cd _output
6) PROGRAM.exe

Darwin and Linux:
1) Make sure that 'CSC=mcs' and 'OPT=-' is uncommented in the Makefile
2) Type 'make' at the command line.  This will build the examples and
   also copy cheetah.so to _output/libcheetah.so:  note that the "lib"
   prefix is required for C# interpreter to successfully load the
   software library.
3) cd _output
4) mono PROGRAM

Note that libcheetah.so must be in either the current directory or
in a directory contained in the environment variable
LD_LIBRARY_PATH.  Otherwise, 'mono _output/PROGRAM' will fail.
