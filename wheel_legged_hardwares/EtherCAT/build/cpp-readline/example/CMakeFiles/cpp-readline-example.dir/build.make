# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rb/four_wheelleg_EtherCAT/EtherCAT

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rb/four_wheelleg_EtherCAT/EtherCAT/build

# Include any dependencies generated for this target.
include cpp-readline/example/CMakeFiles/cpp-readline-example.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include cpp-readline/example/CMakeFiles/cpp-readline-example.dir/compiler_depend.make

# Include the progress variables for this target.
include cpp-readline/example/CMakeFiles/cpp-readline-example.dir/progress.make

# Include the compile flags for this target's objects.
include cpp-readline/example/CMakeFiles/cpp-readline-example.dir/flags.make

cpp-readline/example/CMakeFiles/cpp-readline-example.dir/main.cpp.o: cpp-readline/example/CMakeFiles/cpp-readline-example.dir/flags.make
cpp-readline/example/CMakeFiles/cpp-readline-example.dir/main.cpp.o: /home/rb/four_wheelleg_EtherCAT/EtherCAT/cpp-readline/example/main.cpp
cpp-readline/example/CMakeFiles/cpp-readline-example.dir/main.cpp.o: cpp-readline/example/CMakeFiles/cpp-readline-example.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rb/four_wheelleg_EtherCAT/EtherCAT/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object cpp-readline/example/CMakeFiles/cpp-readline-example.dir/main.cpp.o"
	cd /home/rb/four_wheelleg_EtherCAT/EtherCAT/build/cpp-readline/example && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT cpp-readline/example/CMakeFiles/cpp-readline-example.dir/main.cpp.o -MF CMakeFiles/cpp-readline-example.dir/main.cpp.o.d -o CMakeFiles/cpp-readline-example.dir/main.cpp.o -c /home/rb/four_wheelleg_EtherCAT/EtherCAT/cpp-readline/example/main.cpp

cpp-readline/example/CMakeFiles/cpp-readline-example.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/cpp-readline-example.dir/main.cpp.i"
	cd /home/rb/four_wheelleg_EtherCAT/EtherCAT/build/cpp-readline/example && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rb/four_wheelleg_EtherCAT/EtherCAT/cpp-readline/example/main.cpp > CMakeFiles/cpp-readline-example.dir/main.cpp.i

cpp-readline/example/CMakeFiles/cpp-readline-example.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/cpp-readline-example.dir/main.cpp.s"
	cd /home/rb/four_wheelleg_EtherCAT/EtherCAT/build/cpp-readline/example && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rb/four_wheelleg_EtherCAT/EtherCAT/cpp-readline/example/main.cpp -o CMakeFiles/cpp-readline-example.dir/main.cpp.s

# Object files for target cpp-readline-example
cpp__readline__example_OBJECTS = \
"CMakeFiles/cpp-readline-example.dir/main.cpp.o"

# External object files for target cpp-readline-example
cpp__readline__example_EXTERNAL_OBJECTS =

cpp-readline/example/cpp-readline-example: cpp-readline/example/CMakeFiles/cpp-readline-example.dir/main.cpp.o
cpp-readline/example/cpp-readline-example: cpp-readline/example/CMakeFiles/cpp-readline-example.dir/build.make
cpp-readline/example/cpp-readline-example: cpp-readline/src/libcpp-readline.so.0.1.0
cpp-readline/example/cpp-readline-example: /usr/lib/x86_64-linux-gnu/libreadline.so
cpp-readline/example/cpp-readline-example: cpp-readline/example/CMakeFiles/cpp-readline-example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/rb/four_wheelleg_EtherCAT/EtherCAT/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable cpp-readline-example"
	cd /home/rb/four_wheelleg_EtherCAT/EtherCAT/build/cpp-readline/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cpp-readline-example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cpp-readline/example/CMakeFiles/cpp-readline-example.dir/build: cpp-readline/example/cpp-readline-example
.PHONY : cpp-readline/example/CMakeFiles/cpp-readline-example.dir/build

cpp-readline/example/CMakeFiles/cpp-readline-example.dir/clean:
	cd /home/rb/four_wheelleg_EtherCAT/EtherCAT/build/cpp-readline/example && $(CMAKE_COMMAND) -P CMakeFiles/cpp-readline-example.dir/cmake_clean.cmake
.PHONY : cpp-readline/example/CMakeFiles/cpp-readline-example.dir/clean

cpp-readline/example/CMakeFiles/cpp-readline-example.dir/depend:
	cd /home/rb/four_wheelleg_EtherCAT/EtherCAT/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rb/four_wheelleg_EtherCAT/EtherCAT /home/rb/four_wheelleg_EtherCAT/EtherCAT/cpp-readline/example /home/rb/four_wheelleg_EtherCAT/EtherCAT/build /home/rb/four_wheelleg_EtherCAT/EtherCAT/build/cpp-readline/example /home/rb/four_wheelleg_EtherCAT/EtherCAT/build/cpp-readline/example/CMakeFiles/cpp-readline-example.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : cpp-readline/example/CMakeFiles/cpp-readline-example.dir/depend

