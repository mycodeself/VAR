# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ros/VAR/p2_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ros/VAR/p2_ws/build

# Include any dependencies generated for this target.
include get_pointclouds/CMakeFiles/get_pointclouds_node.dir/depend.make

# Include the progress variables for this target.
include get_pointclouds/CMakeFiles/get_pointclouds_node.dir/progress.make

# Include the compile flags for this target's objects.
include get_pointclouds/CMakeFiles/get_pointclouds_node.dir/flags.make

get_pointclouds/CMakeFiles/get_pointclouds_node.dir/src/node.cpp.o: get_pointclouds/CMakeFiles/get_pointclouds_node.dir/flags.make
get_pointclouds/CMakeFiles/get_pointclouds_node.dir/src/node.cpp.o: /home/ros/VAR/p2_ws/src/get_pointclouds/src/node.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ros/VAR/p2_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object get_pointclouds/CMakeFiles/get_pointclouds_node.dir/src/node.cpp.o"
	cd /home/ros/VAR/p2_ws/build/get_pointclouds && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/get_pointclouds_node.dir/src/node.cpp.o -c /home/ros/VAR/p2_ws/src/get_pointclouds/src/node.cpp

get_pointclouds/CMakeFiles/get_pointclouds_node.dir/src/node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/get_pointclouds_node.dir/src/node.cpp.i"
	cd /home/ros/VAR/p2_ws/build/get_pointclouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ros/VAR/p2_ws/src/get_pointclouds/src/node.cpp > CMakeFiles/get_pointclouds_node.dir/src/node.cpp.i

get_pointclouds/CMakeFiles/get_pointclouds_node.dir/src/node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/get_pointclouds_node.dir/src/node.cpp.s"
	cd /home/ros/VAR/p2_ws/build/get_pointclouds && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ros/VAR/p2_ws/src/get_pointclouds/src/node.cpp -o CMakeFiles/get_pointclouds_node.dir/src/node.cpp.s

get_pointclouds/CMakeFiles/get_pointclouds_node.dir/src/node.cpp.o.requires:
.PHONY : get_pointclouds/CMakeFiles/get_pointclouds_node.dir/src/node.cpp.o.requires

get_pointclouds/CMakeFiles/get_pointclouds_node.dir/src/node.cpp.o.provides: get_pointclouds/CMakeFiles/get_pointclouds_node.dir/src/node.cpp.o.requires
	$(MAKE) -f get_pointclouds/CMakeFiles/get_pointclouds_node.dir/build.make get_pointclouds/CMakeFiles/get_pointclouds_node.dir/src/node.cpp.o.provides.build
.PHONY : get_pointclouds/CMakeFiles/get_pointclouds_node.dir/src/node.cpp.o.provides

get_pointclouds/CMakeFiles/get_pointclouds_node.dir/src/node.cpp.o.provides.build: get_pointclouds/CMakeFiles/get_pointclouds_node.dir/src/node.cpp.o

# Object files for target get_pointclouds_node
get_pointclouds_node_OBJECTS = \
"CMakeFiles/get_pointclouds_node.dir/src/node.cpp.o"

# External object files for target get_pointclouds_node
get_pointclouds_node_EXTERNAL_OBJECTS =

/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: get_pointclouds/CMakeFiles/get_pointclouds_node.dir/src/node.cpp.o
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libpcl_ros_filters.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libpcl_ros_io.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libpcl_ros_tf.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_common.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_kdtree.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_octree.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_search.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_io.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_sample_consensus.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_filters.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_visualization.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_outofcore.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_features.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_segmentation.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_people.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_registration.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_recognition.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_keypoints.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_surface.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_tracking.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libpcl_apps.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libboost_iostreams-mt.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libboost_serialization-mt.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libqhull.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libOpenNI.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libflann_cpp_s.a
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libvtkCommon.so.5.8.0
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libvtkRendering.so.5.8.0
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libvtkHybrid.so.5.8.0
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libvtkCharts.so.5.8.0
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libdynamic_reconfigure_config_init_mutex.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libnodeletlib.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libbondcpp.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libtinyxml.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libclass_loader.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libPocoFoundation.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/x86_64-linux-gnu/libdl.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libroslib.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/librosbag.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/librosbag_storage.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libboost_program_options-mt.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libtopic_tools.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libtf.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libtf2_ros.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libactionlib.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libmessage_filters.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libtf2.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libroscpp.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libboost_signals-mt.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libboost_filesystem-mt.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/librosconsole.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/librosconsole_log4cxx.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/librosconsole_backend_interface.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/liblog4cxx.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libboost_regex-mt.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libxmlrpcpp.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libroscpp_serialization.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/librostime.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libboost_date_time-mt.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libboost_system-mt.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/libboost_thread-mt.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libcpp_common.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: /opt/ros/hydro/lib/libconsole_bridge.so
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: get_pointclouds/CMakeFiles/get_pointclouds_node.dir/build.make
/home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node: get_pointclouds/CMakeFiles/get_pointclouds_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node"
	cd /home/ros/VAR/p2_ws/build/get_pointclouds && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/get_pointclouds_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
get_pointclouds/CMakeFiles/get_pointclouds_node.dir/build: /home/ros/VAR/p2_ws/devel/lib/get_pointclouds/get_pointclouds_node
.PHONY : get_pointclouds/CMakeFiles/get_pointclouds_node.dir/build

get_pointclouds/CMakeFiles/get_pointclouds_node.dir/requires: get_pointclouds/CMakeFiles/get_pointclouds_node.dir/src/node.cpp.o.requires
.PHONY : get_pointclouds/CMakeFiles/get_pointclouds_node.dir/requires

get_pointclouds/CMakeFiles/get_pointclouds_node.dir/clean:
	cd /home/ros/VAR/p2_ws/build/get_pointclouds && $(CMAKE_COMMAND) -P CMakeFiles/get_pointclouds_node.dir/cmake_clean.cmake
.PHONY : get_pointclouds/CMakeFiles/get_pointclouds_node.dir/clean

get_pointclouds/CMakeFiles/get_pointclouds_node.dir/depend:
	cd /home/ros/VAR/p2_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/VAR/p2_ws/src /home/ros/VAR/p2_ws/src/get_pointclouds /home/ros/VAR/p2_ws/build /home/ros/VAR/p2_ws/build/get_pointclouds /home/ros/VAR/p2_ws/build/get_pointclouds/CMakeFiles/get_pointclouds_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : get_pointclouds/CMakeFiles/get_pointclouds_node.dir/depend

