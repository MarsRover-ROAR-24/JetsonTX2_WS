# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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
CMAKE_COMMAND = /home/nvidia/.local/lib/python3.6/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/nvidia/.local/lib/python3.6/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nvidia/roar_ws/src/JetsonTX2_WS/localization

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nvidia/roar_ws/src/JetsonTX2_WS/build

# Include any dependencies generated for this target.
include CMakeFiles/ROAR_UKF_main.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/ROAR_UKF_main.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/ROAR_UKF_main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ROAR_UKF_main.dir/flags.make

CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF_main.cpp.o: CMakeFiles/ROAR_UKF_main.dir/flags.make
CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF_main.cpp.o: /home/nvidia/roar_ws/src/JetsonTX2_WS/localization/src/ROAR_UKF_main.cpp
CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF_main.cpp.o: CMakeFiles/ROAR_UKF_main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/nvidia/roar_ws/src/JetsonTX2_WS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF_main.cpp.o"
	/usr/bin/aarch64-linux-gnu-g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF_main.cpp.o -MF CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF_main.cpp.o.d -o CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF_main.cpp.o -c /home/nvidia/roar_ws/src/JetsonTX2_WS/localization/src/ROAR_UKF_main.cpp

CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF_main.cpp.i"
	/usr/bin/aarch64-linux-gnu-g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/roar_ws/src/JetsonTX2_WS/localization/src/ROAR_UKF_main.cpp > CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF_main.cpp.i

CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF_main.cpp.s"
	/usr/bin/aarch64-linux-gnu-g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/roar_ws/src/JetsonTX2_WS/localization/src/ROAR_UKF_main.cpp -o CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF_main.cpp.s

CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF.cpp.o: CMakeFiles/ROAR_UKF_main.dir/flags.make
CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF.cpp.o: /home/nvidia/roar_ws/src/JetsonTX2_WS/localization/src/ROAR_UKF.cpp
CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF.cpp.o: CMakeFiles/ROAR_UKF_main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/nvidia/roar_ws/src/JetsonTX2_WS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF.cpp.o"
	/usr/bin/aarch64-linux-gnu-g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF.cpp.o -MF CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF.cpp.o.d -o CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF.cpp.o -c /home/nvidia/roar_ws/src/JetsonTX2_WS/localization/src/ROAR_UKF.cpp

CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF.cpp.i"
	/usr/bin/aarch64-linux-gnu-g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/roar_ws/src/JetsonTX2_WS/localization/src/ROAR_UKF.cpp > CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF.cpp.i

CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF.cpp.s"
	/usr/bin/aarch64-linux-gnu-g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/roar_ws/src/JetsonTX2_WS/localization/src/ROAR_UKF.cpp -o CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF.cpp.s

CMakeFiles/ROAR_UKF_main.dir/src/Quaternion.cpp.o: CMakeFiles/ROAR_UKF_main.dir/flags.make
CMakeFiles/ROAR_UKF_main.dir/src/Quaternion.cpp.o: /home/nvidia/roar_ws/src/JetsonTX2_WS/localization/src/Quaternion.cpp
CMakeFiles/ROAR_UKF_main.dir/src/Quaternion.cpp.o: CMakeFiles/ROAR_UKF_main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/nvidia/roar_ws/src/JetsonTX2_WS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/ROAR_UKF_main.dir/src/Quaternion.cpp.o"
	/usr/bin/aarch64-linux-gnu-g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ROAR_UKF_main.dir/src/Quaternion.cpp.o -MF CMakeFiles/ROAR_UKF_main.dir/src/Quaternion.cpp.o.d -o CMakeFiles/ROAR_UKF_main.dir/src/Quaternion.cpp.o -c /home/nvidia/roar_ws/src/JetsonTX2_WS/localization/src/Quaternion.cpp

CMakeFiles/ROAR_UKF_main.dir/src/Quaternion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/ROAR_UKF_main.dir/src/Quaternion.cpp.i"
	/usr/bin/aarch64-linux-gnu-g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/roar_ws/src/JetsonTX2_WS/localization/src/Quaternion.cpp > CMakeFiles/ROAR_UKF_main.dir/src/Quaternion.cpp.i

CMakeFiles/ROAR_UKF_main.dir/src/Quaternion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/ROAR_UKF_main.dir/src/Quaternion.cpp.s"
	/usr/bin/aarch64-linux-gnu-g++-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/roar_ws/src/JetsonTX2_WS/localization/src/Quaternion.cpp -o CMakeFiles/ROAR_UKF_main.dir/src/Quaternion.cpp.s

# Object files for target ROAR_UKF_main
ROAR_UKF_main_OBJECTS = \
"CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF_main.cpp.o" \
"CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF.cpp.o" \
"CMakeFiles/ROAR_UKF_main.dir/src/Quaternion.cpp.o"

# External object files for target ROAR_UKF_main
ROAR_UKF_main_EXTERNAL_OBJECTS =

devel/lib/localization/ROAR_UKF_main: CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF_main.cpp.o
devel/lib/localization/ROAR_UKF_main: CMakeFiles/ROAR_UKF_main.dir/src/ROAR_UKF.cpp.o
devel/lib/localization/ROAR_UKF_main: CMakeFiles/ROAR_UKF_main.dir/src/Quaternion.cpp.o
devel/lib/localization/ROAR_UKF_main: CMakeFiles/ROAR_UKF_main.dir/build.make
devel/lib/localization/ROAR_UKF_main: /opt/ros/noetic/lib/libtf2_ros.so
devel/lib/localization/ROAR_UKF_main: /opt/ros/noetic/lib/libactionlib.so
devel/lib/localization/ROAR_UKF_main: /opt/ros/noetic/lib/libmessage_filters.so
devel/lib/localization/ROAR_UKF_main: /opt/ros/noetic/lib/libroscpp.so
devel/lib/localization/ROAR_UKF_main: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
devel/lib/localization/ROAR_UKF_main: /opt/ros/noetic/lib/librosconsole.so
devel/lib/localization/ROAR_UKF_main: /opt/ros/noetic/lib/librosconsole_log4cxx.so
devel/lib/localization/ROAR_UKF_main: /opt/ros/noetic/lib/librosconsole_backend_interface.so
devel/lib/localization/ROAR_UKF_main: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
devel/lib/localization/ROAR_UKF_main: /usr/lib/aarch64-linux-gnu/libboost_regex.so
devel/lib/localization/ROAR_UKF_main: /opt/ros/noetic/lib/libxmlrpcpp.so
devel/lib/localization/ROAR_UKF_main: /opt/ros/noetic/lib/libtf2.so
devel/lib/localization/ROAR_UKF_main: /opt/ros/noetic/lib/libroscpp_serialization.so
devel/lib/localization/ROAR_UKF_main: /opt/ros/noetic/lib/librostime.so
devel/lib/localization/ROAR_UKF_main: /opt/ros/noetic/lib/libcpp_common.so
devel/lib/localization/ROAR_UKF_main: /usr/lib/aarch64-linux-gnu/libboost_system.so
devel/lib/localization/ROAR_UKF_main: /usr/lib/aarch64-linux-gnu/libboost_thread.so
devel/lib/localization/ROAR_UKF_main: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
devel/lib/localization/ROAR_UKF_main: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
devel/lib/localization/ROAR_UKF_main: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
devel/lib/localization/ROAR_UKF_main: /usr/lib/aarch64-linux-gnu/libpthread.so
devel/lib/localization/ROAR_UKF_main: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/localization/ROAR_UKF_main: CMakeFiles/ROAR_UKF_main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/nvidia/roar_ws/src/JetsonTX2_WS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable devel/lib/localization/ROAR_UKF_main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ROAR_UKF_main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ROAR_UKF_main.dir/build: devel/lib/localization/ROAR_UKF_main
.PHONY : CMakeFiles/ROAR_UKF_main.dir/build

CMakeFiles/ROAR_UKF_main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ROAR_UKF_main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ROAR_UKF_main.dir/clean

CMakeFiles/ROAR_UKF_main.dir/depend:
	cd /home/nvidia/roar_ws/src/JetsonTX2_WS/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nvidia/roar_ws/src/JetsonTX2_WS/localization /home/nvidia/roar_ws/src/JetsonTX2_WS/localization /home/nvidia/roar_ws/src/JetsonTX2_WS/build /home/nvidia/roar_ws/src/JetsonTX2_WS/build /home/nvidia/roar_ws/src/JetsonTX2_WS/build/CMakeFiles/ROAR_UKF_main.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/ROAR_UKF_main.dir/depend
