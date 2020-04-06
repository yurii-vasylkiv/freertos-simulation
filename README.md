In this README you will come to understand the basics of toolchain systems (including compilers, linkers and makefiles), building processes, difference in platforms.
You will also learn how to setup Windows Environment similar to Linux and compile Avionics-Flight-Computer project with CLion.
I will draw the difference between Atollic Studio and CLion. Also, I will explain why it is better to be able to set up CLion for development.

AtollicTrueStudio:
-----------------
This is a simple eclipse-based IDE, nothing else!

It just sets up the embedded development environment for you. Basically does the job you should be doing in order to understand
how things work. So what exactly does Atollic do?
0. Sets up the internal environment variables: path to ARMTools, etc.
1. Handles creation of Makefile for you: it provides an easier interface for "Include Directories" and other source code files organization
2. Handles creation of entry point assembly file "startup.s" that runs the hardware, setting up the registers. error handlers and makes it call your int main() function
3. Handles creating of a linker script for linker to correctly merge the object files into a target executable
4. Automates compilation process calling arm-atollic-eabi-gcc.exe with proper arguments
5. Handles debugging setup, through ST-Link driver, connecting arm-atollic-eabi-gdb.exe straight to the driver

That is why you do not need to care of anything. You just follow the right instructions and you are good to go!
All the eclipse-based IDEs are notorious for working with the internal environment only within eclipse runtime. That is
why you basically cannot do anything with the project unless you have AtollicStudio. Sometimes it is not the best solution, as it hinders you from learning and also
limits your capabilities to control the project. It is especially bad for embedded development, as you can benefit a lot from knowing how things work and to have a full control
of the project in your head. Making you capable of building and uploading the code to the board with you bare hands using native command console only.


CLion:
-----------------

This is also IDE! But it is not eclipse based, thus, you must set it up yourself. Plus, CLion does not have an internal building project tools and purely relies on CMake Project generating
tool. Benefits are that you will completely understand how project is build an compiled, will have control over the smallest details and CLion is way cooler IDE which is extremely
intelligent compared to dump and stupid eclipse which does not provide any comfort neither for editing nor for debugging. Lastly, the design of eclipse sucks!



Some theory to know before proceeding with CLion.
Before setting things up, if you happened to work on Windows OS, then I just want to let you know that setting up environment
takes more steps than for Linux OS. First thing you need to understand is that Windows works very tight with environment variables,
while Linux uses symbolic links and installation principles. Symbolic links are similar to Windows shortcuts, but the difference
is that the former can act as executable, while Windows shortcuts (also well known as lnk files) cannot be executed.
Also, in Linux, many software libraries especially those that are meant to be used in command line install the corresponding executables
into /usr/bin folder, so you end up having every single tool in one location. Therefore, if some application does not have a dedicated bin
folder that is copied into /usr/bin/ then you can create a symbolic link to its executable and put it into /usr/bin - the effect is identical.
Linux's /usr/bin folder is identical to how Windows Environment Path works. You can treat them the same way.
If you want to have a way to run your application from Linux terminal calling it by its name only, you need to make sure that
you have an executable or symbolic link symbolic link in /usr/bin/.
On Windows if you want to run your application from CMD or any terminal calling it by its name only, you need to make sure that
you have its path (where .exe file is, usually in the root directory of particular application or in some of its nested folders) set in the Environment Path.


Windows Environment Paths:
-----------------

On Windows you have two Environment Paths: System Environment Path and User Environment Path. Both are called as "Path"
So you have two versions of this variable, one is seen system-wide, one is seen user-wide.
You can check your Environment Variables by going to: Control Panel ->System and Security -> Environment -> Advanced System Settings -> Environment Variables
You will see two tables, one is dedicated for USER and one is for SYSTEM. If you want to setup command line only for yourself then use the table above, if you want them to work
for all users, use the one below.

Environment Variables:
-----------------

Both tables know the only variable called "Path" and that is the only thing your Windows system will ever be able to see.
However you can see that there are many entries in each table. Each entry has a name called "variable", so you can edit those variables and add up to a limited number of paths.
Create, for example, a variable called "MY_GAMES" and add there all the folders containing their exe files one by one separated by a semicolon.
Then confirm and apply changes, you need to open a new command line session for the change to work. However it will not work anyways, because Windows only knows about variable named "Path".
To make it work, you need to add you variable "MY_GAMES" to Path by adding "%MY_GAMES%" to it. When you open a new command line session this time, Windows will extract %MY_GAMES% and will incorporate
all the entries of MY_GAMES variables. Now you can run all your games that you added to MY_GAMES from any command line.

C/C++ Toolchains:
-----------------

To build any project from source you need to have certain tools (executables) that are meant to generate object files, link them together into an executable that you will then run on a platform.
Those tools are called compiler and linker. Compiler translate each of your code source files (*.c or *.cpp) into a corresponding object file (*.o) that contain machine code. Linkers take those binary
files and literally merges them together using certain rules and produce a single executable file. Linker's rules are defined in a text configuration file called linker script. Every link operation done
by a linker is controlled by a linker script. This script is written in the linker command language. The main purpose of the linker script is to describe how the sections in the input files should be
mapped into the output file, and to control the memory layout of the output file. Linker also produces map files (*.map) which map memory addresses to functions and variables within the executable (exe, dll, elf, etc)
For embedded systems, map files are a lot more useful. (Although you wouldn't be using Visual C++ for that ;) ) Things like knowing how close you are to running out of program/data memory, and what location
a particular variable resides in, are important.

Makefiles
-----------------

Makefile is a file (by default named "Makefile") containing a set of directives used by a make build automation tool to generate a target/goal (executable). Most often, the makefile directs
Make tool on how to compile and  link a program. A makefile works upon the principle that files only need recreating if their dependencies are newer than the file being created/recreated.
The makefile is recursively carried out (with dependency prepared before each target depending upon them) until everything has been updated (that requires updating) and the primary/ultimate target is complete.
These instructions with their dependencies are specified in a makefile. If none of the files that are prerequisites have been changed since the last time the program was compiled, no actions take place.
Using C/C++ as an example, when a C/C++ source file is changed, it must be recompiled. If a header file has changed, each C/C++ source file that includes the header file must be recompiled to be safe.
Each compilation produces an object file corresponding to the source file. Finally, if any source file has been recompiled, all the object files, whether newly made or saved from previous compilations,
must be linked together to produce the new executable program.


CMake Project Generator:
-----------------

A CMake Generator is responsible for writing the input files for a native build system.

What's a generator?
To understand what a generator is, we need to first look at what is a build system. CMake doesn't compile or link any source files. It used a generator to create configuration files for a build system.
The build system uses those files to compile and link source code files.

So what's a build system?
A build system is a broad term that groups together a set of tools used to generally compile and link source code, but it can also include auxiliary tools used during a build process.
For example, in a multi-stage build system, one executable might be built to be used in the build process of another build. Depending on the tool chain used on a system, CMake will generate multiple files
and folders to allow the building of the source files referenced in the CMakeLists.txt and supporting .cmake files.

Sometimes multiple build systems may be installed on a computer, like for Windows you could have a Visual Studio and MinGW build system. CMake allows you to specify which if these build systems to generate
configuration files for.

CMake includes a number of Command-Line, IDE, and Extra generators.

Command-Line Build Tool Generators
These generators are for command-line build tools, like Make and Ninja. The chosen tool chain must be configured prior to generating the build system with CMake.
The following are supported(**):
- Borland Makefiles
- MSYS Makefiles
- MinGW Makefiles
- NMake Makefiles
- NMake Makefiles JOM
- Ninja
- Unix Makefiles
- Watcom WMake

IDE Build Tool Generators
These generators are for Integrated Development Environments that include their own compiler. Examples are Visual Studio and Xcode which include a compiler natively.
The following are supported(**):
- Visual Studio 6
- Visual Studio 7
- Visual Studio 7 .NET 2003
- Visual Studio 8 2005
- Visual Studio 9 2008
- Visual Studio 10 2010
- Visual Studio 11 2012
- Visual Studio 12 2013
- Visual Studio 14 2015
- Visual Studio 15 2017
- Visual Studio 16 2019
- Green Hills MULTI
- Xcode


Extra Generators
These are generators that create a configuration to work with an alternative IDE tool and must be included with either an IDE or Command-Line generator.
The following are supported(**):

- CodeBlocks
- CodeLite
- Eclipse CDT4
- KDevelop3 (Unsupported after v3.10.3)
- Kate
- Sublime Text 2

If I have a set of C++ files in my project, are these the input files?
Yes, they are some of the input files. For a make build system you also have a MakeFile. For Visual Studio you have a solution file (.sln).
With both systems there are additional files needed that CMake knows how to create given a proper CMakeLists.txt file.

If I'm using Linux, what is my native build system by default? Make?
Generally, yes, but other build systems could be setup like Ninja.

Why do the input files have to be written by the generator if they already exist?
Some source files may already exist, but CMake has the ability to generate header and source files. Also as mentioned above, there are configuration
files that must be generated that depend on the source files supplied in the CMakeLists.txt file.



x86-x64, ARM, embedded ARM and cross-platform toolchain:
-----------------

A toolchain is a set of distinct software development tools that are linked (or chained) together by specific stages such as GCC, binutils and glibc (a portion of the GNU Toolchain). Optionally,
a toolchain may contain other tools such as a debugger or a compiler for a specific programming language, such as C++. Quite often, the toolchain used for embedded development is a cross toolchain,
or more commonly known as a cross compiler. All the programs (like GCC) run on a host system of a specific architecture (such as x86), but they produce binary code (executables) to run on a different
architecture (for example, ARM). This is called cross compilation and is the typical way of building embedded software. It is possible to compile natively, running GCC on your target. Before searching
for a prebuilt toolchain or building your own, it's worth checking to see if one is included with your target hardware. However, a GCC native to your host computer will not always support the instructions
for embedded ARM processors. The instructions and the number of instructions differ significantly across the processors. For example, on your computer, you have x86-x64 processor which is a CISC
(Complex Instruction Set Computing) while ARM processors are RISC (Reduced Instruction Set Computing) for example and support a different set of instructions.

Complex Instruction Set Computer (CISC) processors, like the x86, have a rich instruction set capable of doing complex things with a single instruction. Such processors often have significant amounts
of internal logic that decode machine instructions to sequences of internal operations (microcode).

RISC architectures, in contrast, have a smaller number of more general purpose instructions, that might be executed with significantly fewer transistors, making the silicon cheaper and more power efficient.
Like other RISC architectures, ARM cores have a large number of general-purpose registers and many instructions execute in a single cycle. It has simple addressing modes, where all load/store addresses can
be determined from register contents and instruction fields.

So logically, you cannot port instructions between x86-x64 and ARM, but you are just supposed to use dedicated compilers in order to translate the code into instructions known for a particular processor.
That is why for embedded development Atollic packs dedicated ARM Toolchain located in Atollic/ARMTools/bin folder to be able to translate the code using supported instruction set. This is also usually
referred as application binary interface (ABI). ABI is an interface between two binary program modules; often, one of these modules is a library or operating system facility, and the other is a program
that is being run by a user. ABI defines the low-level binary interface between two or more pieces of software on a particular architecture

ABIs cover details such as:
- a processor instruction set (with details like register file structure, stack organization, memory access types, ...)
- the sizes, layouts, and alignments of basic data types that the processor can directly access
- the calling convention, which controls how functions' arguments are passed and return values are retrieved; for example, whether all parameters are passed on the stack or some are passed in registers,
  which registers are used for which function parameters, and whether the first function parameter passed on the stack is pushed first or last onto the stack
- how an application should make system calls to the operating system and, if the ABI specifies direct system calls rather than procedure calls to system call stubs, the system call numbers
- and in the case of a complete operating system ABI, the binary format of object files, program libraries and so on.

PREFIXES: What is the difference between versions of tools' names?
If you go into Atollic/ARMTools/bin you will find that all files have prefix arm-atollic-eabi, while if you look into mingw32-64/bin/ you will see find no such prefix, also if you go to the corresponding folder
of STM32CubeIDE you will find arm-none-eabi. So, what is the difference?

arm-atollic-eabi: This toolchain targets the ARM architecture, has vendor "atollic", does not target any operating system, and complies with the ARM EABI.
arm-none-eabi: This toolchain targets the ARM architecture, has no vendor, does not target any operating system, and complies with the ARM EABI.
arm-none-linux-gnueabi or nothing: This toolchain targets the ARM architecture, has no vendor, creates binaries that run on the Linux operating system,
and uses the GNU EABI. It is used to target ARM-based Linux systems.

NOTE: that 'abi' refers to the same application binary interface (ABI)


MinGW Environment
-----------------

So, we have talked about ABI. Now we need to talk about API (Application Programming Interface).
Windows uses win32 API to provide development environment, while of course win32 forwards the calls to native API.
Native API is something used across all the platforms, it mostly provides system calls which all the other libraries, including win32 use.
Most of the Windows APIs internally invoke the Native APIs. All APIs of the various available subsystems invoke the Native APIs to perform the actual operation.
Native API is also often referred as POSIX. POSIX stands for Portable Operating System Interface, and is an IEEE standard designed to facilitate application portability.
POSIX is an attempt by a consortium of vendors to create a single standard version of UNIX. If they are successful, it will make it easier to port applications between hardware platforms.
Therefore, in order to access native C Runtime Environment, we need POSIX API for that which is provided in Minimalist GNU for Windows referred to as MinGW

MinGW, a contraction of "Minimalist GNU for Windows", is a minimalist development environment for native Microsoft Windows applications.
MinGW provides a complete Open Source programming tool set which is suitable for the development of native MS-Windows applications, and which do not depend on any 3rd-party C-Runtime DLLs.
MinGW compilers provide access to the functionality of the Microsoft C runtime and some language-specific runtimes.
Primarily intended for use by developers working on the native MS-Windows platform, but also available for cross-hosted use, MinGW includes:
- A port of the GNU Compiler Collection (GCC), including C, C++, ADA and Fortran compilers;
- GNU Binutils for Windows (assembler, linker, archive manager)
- A command-line installer, with optional GUI front-end, (mingw-get) for MinGW and MSYS deployment on MS-Windows
- A GUI first-time setup tool (mingw-get-setup), to get you up and running with mingw-get.

MinGW gives us access to native C environment, that is why we use it in the first place.


THE END of theory!
------------------------------------------------------------------------------------------------------------------------
CLion setup:
To make CLION work you need to things:
1. Setup the Environment
2. Setup CMakeLists.txt file ( already handled and should give no problem if the first step is done properly )

Setup the Environment:
This step requires some level of organization. In Linux it is easier and this step is usually omitted as Linux is a core
operating system for development.

DOWNLOAD:
-----------------

- MinGW-64
- ATollic True Studio
- CLion (latest version)
- Open OCD
- Python (latest version)

INSTALLATION:
-----------------

1. Create a folder C:\dev
2. Create a folder C:\dev\opt
3. Create a folder C:\dev\env
4. Install MinGW into C:dev\env
5. Install CLion into C:\dev\opt\clion
6. Install Atollic into C:\dev\opt\atollic (you will have to rename Atollic folder as it does not allow to rename it during the installation)
7. Install Open OCD into C:dev\opt\openocd
8. Install Python to C:\Program Files

NOTE: while installing CLion and Python do not check on ADD TO SYSTEM PATH

ENVIRONMENT VARIABLES:
-----------------

Recall what I have mentioned about system-wide Path and User-wide Path. For our purpose we will use User-Wide Path variable
1. Create a variable named "ARM_DEV_ENV"
2. Add 6 entries separated by semicolon for:
    - MinGW             C:\dev\env
    - CLion             C:\dev\opt\clion
    - CLion's CMake     C:\dev\opt\clion\cmake\win\bin
    - Open OCD          C:dev\opt\openocd\bin
    - Python            C:\<ProgramFiles>\Python\Python<VERSION>
    - Python scripts    C:\<ProgramFiles>\Python\Python<VERSION>\Scripts
    - Atollic ARMTools: C:\dev\opt\atollic\ARMTools\bin

Your ARM_DEV_ENV in the end should look something like this:
C:\dev\env;C:\dev\opt\clion;C:\dev\opt\clion\cmake\win\bin;C:dev\opt\openocd\bin;C:\<ProgramFiles>\Python\Python<VERSION>;C:\<ProgramFiles>\Python\Python<VERSION>\Scripts;C:\dev\opt\atollic\ARMTools\bin
Then add new entry %ARM_DEV_ENV% to the "Path" variable and apply changes press OK.

COMMAND TERMINAL:
-----------------

1. Open !NEW! session of any windows terminal (CMD PowerShell, etc)
2. Type 'cmake'
3. Type 'mingw32-make'
4. Type 'python'
5. Type 'arm-atollic-eabi-gcc'

NOTE: Make sure that the following commands work, if it says:
CMD: 'your command' is not recognized as an internal or external command, operable program or batch file.
PowerShell: The term 'your command' is not recognized as the name of a cmdlet, function, script file, or operable program
Then open another terminal session and do it again, try both terminals CMD and PowerShell, if the problem persists then go
back to Environment Variables and make sure you set up everything like I described and then try again.


CLion:
-----------------

When all the commands work open clion FROM THE TERMINAL (!IMPORTANT!) to make sure we have access to all the variables that
we just typed and saw working. Of course you can run it from Windows Menu and it should all see the ARM_DEV_ENV variables.
If for some reason CLion does not see ARM_DEV_ENV then run CLion from the terminal. To check whether CLion sees ARM_DEV_ENV,
open CLion, then go to View -> Tool Windows -> Terminal and type all the commands I have any command that we have tried before
and if they work it means CLion sees your ARM_DEV_ENV.

Now, go to CLion: File -> Settings -> Build, Execution, Deployment -> Toolchains. Add new Toolchain by pressing '+'.
Specify your MingGw location (C:\dev\env), then let CLion automatically detect the rest. Then change the values of
C Compiler to C:\dev\opt\atollic\ARMTools\bin\arm-atollic-eabi-gcc.exe
C++ Compiler to C:\dev\opt\atollic\ARMTools\bin\arm-atollic-eabi-g++.exe

NOTE: Recall what I said about differences in processor instruction sets. You will not be able to compile with gcc.exe or g++.exe shipped with MinGW (C:\dev\env\bin)
NOTE: You may leave the default debugger, it will be able to understand STM32 ARM debug symbols, but if you use C:\dev\env\bin\gdb.exe, then debugger will not work.
      Debuggers have certain protocols of communication, just like ABI, they need to match.
NOTE: Leave detected by CLion ming32-make.exe as it is. Do not use any other make.exe because again different Make tools have different protocols, so they need to match

The final step is to tell the CMakeLists.txt where to find the ARMTools that will be used to compile the source code for the targeted platform (ARM Cortex-M4).
Open the CMakeLists.txt found in the AvionicsSoftware-AtollicProject folder and edit the line that says "set(ARM_TOOLS_DIR    [YOUR PATH HERE])" to contain
the path for Atollic ARMTools mentioned earlier.

NOTE: You must change the "\" in the path to "/".

COMPILE AND ENJOY CLION!Now it should work!




WINDOWS Aliases:
-----------------

It is good to be able to travel from one folder to another fast, without thinking. It is extremely comfortable.
The following Aliases will ease your life

CMD:
ECHO ^@echo off > generated\startup.cmd
ECHO doskey make=%MINGW_ENV_PATH%\mingw32-make.exe >> generated\startup.cmd
ECHO doskey avionics=cd %SOURCE_PATH% >> generated\startup.cmd
ECHO doskey go_to_avionics=cd %SOURCE_PATH% >> generated\startup.cmd
reg add "HKCU\Software\Microsoft\Command Processor" /v AutoRun /t REG_EXPAND_SZ /d "%SOURCE_PATH%\generated\startup.cmd" /f

PowerShell:
New-Item -ItemType File $profile -Force
Clear-Content -Path $profile
"New-Alias make C:\dev\env\bin\mingw32-make.exe" | Add-Content  $profile
"function go_to_avionics {set-location $SOURCE_PATH}" | Add-Content  $profile
"function _avionics_build {$BUILD_FILE}" | Add-Content  $profile
"New-Alias avionics go_to_avionics" | Add-Content  $profile














EXTRAS:
-----------------
Example of the file user-configurations.yaml:
```
MINGW_ENV_PATH:         C:/dev/env/bin
CLION_CMAKE_PATH:       C:/dev/opt/clion/bin/cmake/win/bin
ARM_TOOLS_DIR:          C:/dev/opt/cubeide/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-arm-embedded.7-2018-q2-update.win32_1.0.0.201904081647/tools/bin
```


FOR WINDOWS:
```
MAKE SURE to set up CLION properly.
The configurations include: Toolchains such as MinGW make, gcc and g++.
Install STM32CubeIDE or AtttolicTrue Studio to install ARM build toolchains

If you use STM32CubeIDE:          {STM32CubeIDE_LOCATION}/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-arm-embedded.7-2018-q2-update.win32_{CHOOSE_LATEST_VERSION}/tools/bin
If you use AttolicTrueStudio:     {AttolicTrueStudioIDE_LOCATION}/ARMTools/bin

In order CLion to compile this project on Windows machine you need to:
1. Setup your environment:
 a) install MinGW environment, either follow the link https://sourceforge.net/projects/mingw-w64 or just search for MinGW-w64 - for 32 and 64 bit Windows
 b) setup the packages you need in the mingw manager (usually they are already checked for you), you will need packages such as gcc, g++, gdb, make, cmake, etc.
2. Setup the System and the System Environment Variables:
 a) Control Panel -> System And Security -> System -> Advanced System Settings -> Environment Variables [OR] My Computer -> Properties -> Advanced -> Environment Variables
 b) Add your ARM Build Toolchains binaries folder to your path (e.g. {AttolicTrueStudioIDE_LOCATION}/ARMTools/bin)
 c) Add CLion's CMake path to the system environment variables (e.g. C:/Program Files/JetBrains/CLion 2019.3.2/bin/cmake/win/bin)
 NOTE: You can either create your own variable where you would add all the development related pathes or just add it straight to the variable named \"PATH\"
 d) Create a \"make\" alias for Windows PowerShell. Search for PowerShell open it with administrator rights, then type the following:
    >> Set-ExecutionPolicy RemoteSigned (if asking confirm with \"Y\")
    >> echo $profile (hit Enter, you should receive a path e.g. C:/Users/xxxxxxx/Documents/WindowsPowerShell/Microsoft.PowerShell_profile.ps1)
       Create the path and the file if it does not exist, then open it and type:
       New-Alias make YOUR_PATH_TO_mingw32-make.exe (e.g. C:\\dev\\env\\bin\\mingw32-make.exe)
       Save file and close it
 e) open PowerShell and go to the project directory and type:
    ./build.ps1
 f) wait till it is finished, but make sure the build is successful
 g) Execute CLion from PowerShell!


3. Setup CLion Toolchains:
 a) Go to File -> Settings -> Build, Execution, Deployment -> Toolchains, let CLion detect the existing environments. then make some modifications to it
    Then for the first field \"Environment\" you specify mingw root folder (e.g. C:/Program Files/mingw-w64/x86_64-7.2.0-posix-seh-rt_v5-rev1/mingw64)
    then CLion will try to find \"Make\", \"C Compiler\" and \"CXX Compiler\" for you automatically.
 b) Change CLion CMake Genration Path from default \"cmake-build-debug\" to \"build\":
    File -> Settings -> Build, Execution, Deployment -> CMake -> \"Generation path\"
 c) Debugger. In order to enable CLion debugger we need to download \"OpenOCD\"
    OpenOCD     - https://gnutoolchains.com/arm-eabi/openocd/

    File -> Settings -> Build, Execution, Deployment -> Embedded Development.
    Specify OpenOCD Location        : (e.g. C:\\dev\\opt\\openocd\\bin\\openocd.exe)

    Lastly, do:
    Run -> Debug... -> Edit Configurations -> Add -> OpenOCD Download & Run
    Then press \"Assist\" and choose the configuration file that corresponds to the STM32 board that is used in the project
```


FOR LINUX

```
MAKE SURE to set up CLION Toolchains properly.
Install STM32CubeIDE or AtttolicTrue Studio to install ARM build toolchains
The location of the ARM Build Toolchains depends on the OS you are running CLion on:
For Linux OS, if you use STM32CubeIDE:            {STM32CubeIDE_LOCATION}/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.7-2018-q2-update.linux64_{CHOOSE_LATEST_VERSION}/tools/bin
For Linux OS, if you use AttolicTrueStudio:       {AttolicTrueStudioIDE_LOCATION}/ARMTools/bin
Setup CLion Toolchains:
a) Go to File -> Settings -> Build, Execution, Deployment -> Toolchains, let CLion detect the existing environments.
b) Change the values of the \"C Compiler\" and \"CXX Compiler\" to the corresponding ARM Build Toolchains gcc and g++.
c) Change CLion CMake Genration Path from default \"cmake-build-debug\" to \"build\":
   File -> Settings -> Build, Execution, Deployment -> CMake -> \"Generation path\"

d) Debugger. In order to enable CLion debugger we need to download \"OpenOCD\"
   OpenOCD     - https://gnutoolchains.com/arm-eabi/openocd/

   File -> Settings -> Build, Execution, Deployment -> Embedded Development.
   Specify OpenOCD Location        : (e.g. \\openocd\\bin\\openocd.exe)

   Lastly, do:
   Run -> Debug... -> Edit Configurations -> Add -> OpenOCD Download & Run
   Then press \"Assist\" and choose the configuration file that corresponds to the STM32 board that is used in the project
```










# Avionics
Flight Computer Software for the 2019 Rocket.

## Versions
#### v2.0.0 - 2020 Spaceport America Cup Flight Software
##### v2.1.0 - Major refactoring changes. File structure reorganized and encapsulation improved.

#### v1.0.0 - 2019 Spaceport America Cup Flight Software
- includes custom flight computer software
- includes PC Tools interface for configuring, and downloading data from flight computer

## How To Run Flight Computer Software

The 'Avionics Software - Atollic' folder contains the main flight software project, created in Atollic TrueSTUDIO.
The project runs on the STM32F401RE Nucleo development board.

To run the project:

  1. Open Attolic.
  
  2. Import the project by doing the following:
      - Click File -> Import .
      - Then click 'Existing Projects into Workspace' under general.
      - Click 'Browse' and navigate to the 'Avionics Software - Atollic' then click 'OK'.
      - Make sure the 'Avionics Software' project is selected then click 'Finish'.

  3. Build the project by right clicking on the project in the Project Explorer on the left and clicking 'Build Project'.    
      
  4. Setup the debug configuration:
      -Right-click on the project in the Project explorer on the left.
      - Go to Debug As -> Debug Configurations...
      - Double-click 'Embedded C/C++  Application'
      - In the 'main' tab on the right section of the Debug Configurations window, click 'search project' and then select 'Avionics Software.elf'.
      
  5. Debug the project by clicking 'Debug'.      
      
# Operations

The software in the AvionicsSoftware-AtollicProject folder is meant to be run on the UMSTAS student designed flight computer.
Instructions for uploading the software to the flight computer can be found on google drive: 

UMSATS/Rocket/2-Avionics/Avionics Setup and Use Guide


UMSATS Flight Computer Prototype:

<img src="https://i.imgur.com/smPBZTm.jpg" width="500">


When powered on the flight computer will start a 1 hour countdown.  
The user LED will toggle every 2 seconds until the halfway point, when it will start to toggle every second. When 3/4 of the time has passed, te LED will toggle every 0.5 seconds.

This time can be changed in the configuration.h file.

**After the time has elapsed, the flash memory will be erased** and the flight computer will start recording data at a rate of 10/20 Hz (BMP/IMU). 
The data rate can be changed in the configuration file.
The flight computer will record data until the flash memory is full or power is removed.

To recover data from the flight computer, power it on while pressing the S2 button. This will start recovery mode.
In recovery mode, an inteface will be provided over UART, allowing the data to be read.

---
Information about UMSATS and our new rocketry division can be found at: http://www.umsats.ca/rocketry/

