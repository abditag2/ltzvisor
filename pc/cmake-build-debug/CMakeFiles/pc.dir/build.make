# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.9

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/fardin/Downloads/clion-2017.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/fardin/Downloads/clion-2017.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/fardin/devel/pc

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fardin/devel/pc/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/pc.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pc.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pc.dir/flags.make

CMakeFiles/pc.dir/pc.c.o: CMakeFiles/pc.dir/flags.make
CMakeFiles/pc.dir/pc.c.o: ../pc.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fardin/devel/pc/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/pc.dir/pc.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pc.dir/pc.c.o   -c /home/fardin/devel/pc/pc.c

CMakeFiles/pc.dir/pc.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pc.dir/pc.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/fardin/devel/pc/pc.c > CMakeFiles/pc.dir/pc.c.i

CMakeFiles/pc.dir/pc.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pc.dir/pc.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/fardin/devel/pc/pc.c -o CMakeFiles/pc.dir/pc.c.s

CMakeFiles/pc.dir/pc.c.o.requires:

.PHONY : CMakeFiles/pc.dir/pc.c.o.requires

CMakeFiles/pc.dir/pc.c.o.provides: CMakeFiles/pc.dir/pc.c.o.requires
	$(MAKE) -f CMakeFiles/pc.dir/build.make CMakeFiles/pc.dir/pc.c.o.provides.build
.PHONY : CMakeFiles/pc.dir/pc.c.o.provides

CMakeFiles/pc.dir/pc.c.o.provides.build: CMakeFiles/pc.dir/pc.c.o


# Object files for target pc
pc_OBJECTS = \
"CMakeFiles/pc.dir/pc.c.o"

# External object files for target pc
pc_EXTERNAL_OBJECTS =

pc: CMakeFiles/pc.dir/pc.c.o
pc: CMakeFiles/pc.dir/build.make
pc: CMakeFiles/pc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fardin/devel/pc/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable pc"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pc.dir/build: pc

.PHONY : CMakeFiles/pc.dir/build

CMakeFiles/pc.dir/requires: CMakeFiles/pc.dir/pc.c.o.requires

.PHONY : CMakeFiles/pc.dir/requires

CMakeFiles/pc.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pc.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pc.dir/clean

CMakeFiles/pc.dir/depend:
	cd /home/fardin/devel/pc/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fardin/devel/pc /home/fardin/devel/pc /home/fardin/devel/pc/cmake-build-debug /home/fardin/devel/pc/cmake-build-debug /home/fardin/devel/pc/cmake-build-debug/CMakeFiles/pc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pc.dir/depend

