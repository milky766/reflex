# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/Volumes/Macintosh HD/WorkSpace/control_board_yanlin"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/Volumes/Macintosh HD/WorkSpace/control_board_yanlin/cmake-build-debug"

# Include any dependencies generated for this target.
include CMakeFiles/0705_reflex_demo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/0705_reflex_demo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/0705_reflex_demo.dir/flags.make

CMakeFiles/0705_reflex_demo.dir/src/0704_reflexdemo.cpp.o: CMakeFiles/0705_reflex_demo.dir/flags.make
CMakeFiles/0705_reflex_demo.dir/src/0704_reflexdemo.cpp.o: ../src/0704_reflexdemo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Volumes/Macintosh HD/WorkSpace/control_board_yanlin/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/0705_reflex_demo.dir/src/0704_reflexdemo.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/0705_reflex_demo.dir/src/0704_reflexdemo.cpp.o -c "/Volumes/Macintosh HD/WorkSpace/control_board_yanlin/src/0704_reflexdemo.cpp"

CMakeFiles/0705_reflex_demo.dir/src/0704_reflexdemo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/0705_reflex_demo.dir/src/0704_reflexdemo.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/Volumes/Macintosh HD/WorkSpace/control_board_yanlin/src/0704_reflexdemo.cpp" > CMakeFiles/0705_reflex_demo.dir/src/0704_reflexdemo.cpp.i

CMakeFiles/0705_reflex_demo.dir/src/0704_reflexdemo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/0705_reflex_demo.dir/src/0704_reflexdemo.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/Volumes/Macintosh HD/WorkSpace/control_board_yanlin/src/0704_reflexdemo.cpp" -o CMakeFiles/0705_reflex_demo.dir/src/0704_reflexdemo.cpp.s

# Object files for target 0705_reflex_demo
0705_reflex_demo_OBJECTS = \
"CMakeFiles/0705_reflex_demo.dir/src/0704_reflexdemo.cpp.o"

# External object files for target 0705_reflex_demo
0705_reflex_demo_EXTERNAL_OBJECTS =

0705_reflex_demo: CMakeFiles/0705_reflex_demo.dir/src/0704_reflexdemo.cpp.o
0705_reflex_demo: CMakeFiles/0705_reflex_demo.dir/build.make
0705_reflex_demo: libcontrol_board.a
0705_reflex_demo: /usr/local/lib/libboost_system-mt.dylib
0705_reflex_demo: /usr/local/lib/libboost_thread-mt.dylib
0705_reflex_demo: /usr/local/lib/libbcm2835.a
0705_reflex_demo: CMakeFiles/0705_reflex_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/Volumes/Macintosh HD/WorkSpace/control_board_yanlin/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable 0705_reflex_demo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/0705_reflex_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/0705_reflex_demo.dir/build: 0705_reflex_demo

.PHONY : CMakeFiles/0705_reflex_demo.dir/build

CMakeFiles/0705_reflex_demo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/0705_reflex_demo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/0705_reflex_demo.dir/clean

CMakeFiles/0705_reflex_demo.dir/depend:
	cd "/Volumes/Macintosh HD/WorkSpace/control_board_yanlin/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/Volumes/Macintosh HD/WorkSpace/control_board_yanlin" "/Volumes/Macintosh HD/WorkSpace/control_board_yanlin" "/Volumes/Macintosh HD/WorkSpace/control_board_yanlin/cmake-build-debug" "/Volumes/Macintosh HD/WorkSpace/control_board_yanlin/cmake-build-debug" "/Volumes/Macintosh HD/WorkSpace/control_board_yanlin/cmake-build-debug/CMakeFiles/0705_reflex_demo.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/0705_reflex_demo.dir/depend

