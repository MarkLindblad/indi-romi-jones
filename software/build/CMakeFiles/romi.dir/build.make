# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/Desktop/EECS149/indi-romi-jones/software

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Desktop/EECS149/indi-romi-jones/software/build

# Include any dependencies generated for this target.
include CMakeFiles/romi.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/romi.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/romi.dir/flags.make

CMakeFiles/romi.dir/main.c.o: CMakeFiles/romi.dir/flags.make
CMakeFiles/romi.dir/main.c.o: ../main.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/EECS149/indi-romi-jones/software/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/romi.dir/main.c.o"
	/bin/gcc-10 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/romi.dir/main.c.o -c /home/pi/Desktop/EECS149/indi-romi-jones/software/main.c

CMakeFiles/romi.dir/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/romi.dir/main.c.i"
	/bin/gcc-10 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/Desktop/EECS149/indi-romi-jones/software/main.c > CMakeFiles/romi.dir/main.c.i

CMakeFiles/romi.dir/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/romi.dir/main.c.s"
	/bin/gcc-10 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/Desktop/EECS149/indi-romi-jones/software/main.c -o CMakeFiles/romi.dir/main.c.s

# Object files for target romi
romi_OBJECTS = \
"CMakeFiles/romi.dir/main.c.o"

# External object files for target romi
romi_EXTERNAL_OBJECTS =

romi: CMakeFiles/romi.dir/main.c.o
romi: CMakeFiles/romi.dir/build.make
romi: libraries/kobuki/libkobuki.a
romi: CMakeFiles/romi.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/Desktop/EECS149/indi-romi-jones/software/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable romi"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/romi.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/romi.dir/build: romi

.PHONY : CMakeFiles/romi.dir/build

CMakeFiles/romi.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/romi.dir/cmake_clean.cmake
.PHONY : CMakeFiles/romi.dir/clean

CMakeFiles/romi.dir/depend:
	cd /home/pi/Desktop/EECS149/indi-romi-jones/software/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Desktop/EECS149/indi-romi-jones/software /home/pi/Desktop/EECS149/indi-romi-jones/software /home/pi/Desktop/EECS149/indi-romi-jones/software/build /home/pi/Desktop/EECS149/indi-romi-jones/software/build /home/pi/Desktop/EECS149/indi-romi-jones/software/build/CMakeFiles/romi.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/romi.dir/depend

