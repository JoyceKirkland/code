# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/joyce/workplace/rm/2022/KCf

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/joyce/workplace/rm/2022/KCf/build

# Include any dependencies generated for this target.
include angle_solve/CMakeFiles/basic_pnp.dir/depend.make

# Include the progress variables for this target.
include angle_solve/CMakeFiles/basic_pnp.dir/progress.make

# Include the compile flags for this target's objects.
include angle_solve/CMakeFiles/basic_pnp.dir/flags.make

angle_solve/CMakeFiles/basic_pnp.dir/basic_pnp.cpp.o: angle_solve/CMakeFiles/basic_pnp.dir/flags.make
angle_solve/CMakeFiles/basic_pnp.dir/basic_pnp.cpp.o: ../angle_solve/basic_pnp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joyce/workplace/rm/2022/KCf/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object angle_solve/CMakeFiles/basic_pnp.dir/basic_pnp.cpp.o"
	cd /home/joyce/workplace/rm/2022/KCf/build/angle_solve && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/basic_pnp.dir/basic_pnp.cpp.o -c /home/joyce/workplace/rm/2022/KCf/angle_solve/basic_pnp.cpp

angle_solve/CMakeFiles/basic_pnp.dir/basic_pnp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/basic_pnp.dir/basic_pnp.cpp.i"
	cd /home/joyce/workplace/rm/2022/KCf/build/angle_solve && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/joyce/workplace/rm/2022/KCf/angle_solve/basic_pnp.cpp > CMakeFiles/basic_pnp.dir/basic_pnp.cpp.i

angle_solve/CMakeFiles/basic_pnp.dir/basic_pnp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/basic_pnp.dir/basic_pnp.cpp.s"
	cd /home/joyce/workplace/rm/2022/KCf/build/angle_solve && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/joyce/workplace/rm/2022/KCf/angle_solve/basic_pnp.cpp -o CMakeFiles/basic_pnp.dir/basic_pnp.cpp.s

# Object files for target basic_pnp
basic_pnp_OBJECTS = \
"CMakeFiles/basic_pnp.dir/basic_pnp.cpp.o"

# External object files for target basic_pnp
basic_pnp_EXTERNAL_OBJECTS =

angle_solve/libbasic_pnp.a: angle_solve/CMakeFiles/basic_pnp.dir/basic_pnp.cpp.o
angle_solve/libbasic_pnp.a: angle_solve/CMakeFiles/basic_pnp.dir/build.make
angle_solve/libbasic_pnp.a: angle_solve/CMakeFiles/basic_pnp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/joyce/workplace/rm/2022/KCf/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libbasic_pnp.a"
	cd /home/joyce/workplace/rm/2022/KCf/build/angle_solve && $(CMAKE_COMMAND) -P CMakeFiles/basic_pnp.dir/cmake_clean_target.cmake
	cd /home/joyce/workplace/rm/2022/KCf/build/angle_solve && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/basic_pnp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
angle_solve/CMakeFiles/basic_pnp.dir/build: angle_solve/libbasic_pnp.a

.PHONY : angle_solve/CMakeFiles/basic_pnp.dir/build

angle_solve/CMakeFiles/basic_pnp.dir/clean:
	cd /home/joyce/workplace/rm/2022/KCf/build/angle_solve && $(CMAKE_COMMAND) -P CMakeFiles/basic_pnp.dir/cmake_clean.cmake
.PHONY : angle_solve/CMakeFiles/basic_pnp.dir/clean

angle_solve/CMakeFiles/basic_pnp.dir/depend:
	cd /home/joyce/workplace/rm/2022/KCf/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/joyce/workplace/rm/2022/KCf /home/joyce/workplace/rm/2022/KCf/angle_solve /home/joyce/workplace/rm/2022/KCf/build /home/joyce/workplace/rm/2022/KCf/build/angle_solve /home/joyce/workplace/rm/2022/KCf/build/angle_solve/CMakeFiles/basic_pnp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : angle_solve/CMakeFiles/basic_pnp.dir/depend
