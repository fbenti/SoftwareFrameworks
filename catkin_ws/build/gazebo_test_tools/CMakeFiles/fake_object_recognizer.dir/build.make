# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/filippo/catkin_ws/src/lecture_5_new/gazebo-pkgs/gazebo_test_tools

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/filippo/catkin_ws/build/gazebo_test_tools

# Include any dependencies generated for this target.
include CMakeFiles/fake_object_recognizer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/fake_object_recognizer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/fake_object_recognizer.dir/flags.make

CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.o: CMakeFiles/fake_object_recognizer.dir/flags.make
CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.o: /home/filippo/catkin_ws/src/lecture_5_new/gazebo-pkgs/gazebo_test_tools/src/FakeObjectRecognizer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/filippo/catkin_ws/build/gazebo_test_tools/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.o -c /home/filippo/catkin_ws/src/lecture_5_new/gazebo-pkgs/gazebo_test_tools/src/FakeObjectRecognizer.cpp

CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/filippo/catkin_ws/src/lecture_5_new/gazebo-pkgs/gazebo_test_tools/src/FakeObjectRecognizer.cpp > CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.i

CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/filippo/catkin_ws/src/lecture_5_new/gazebo-pkgs/gazebo_test_tools/src/FakeObjectRecognizer.cpp -o CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.s

CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.o.requires:

.PHONY : CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.o.requires

CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.o.provides: CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.o.requires
	$(MAKE) -f CMakeFiles/fake_object_recognizer.dir/build.make CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.o.provides.build
.PHONY : CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.o.provides

CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.o.provides.build: CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.o


# Object files for target fake_object_recognizer
fake_object_recognizer_OBJECTS = \
"CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.o"

# External object files for target fake_object_recognizer
fake_object_recognizer_EXTERNAL_OBJECTS =

/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.o
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: CMakeFiles/fake_object_recognizer.dir/build.make
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/libgazebo_ros_api_plugin.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/libgazebo_ros_paths_plugin.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/libroslib.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/librospack.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/libtf.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/libtf2_ros.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/libactionlib.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/libmessage_filters.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/libtf2.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/libroscpp.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/librosconsole.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/librostime.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /opt/ros/melodic/lib/libcpp_common.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so: CMakeFiles/fake_object_recognizer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/filippo/catkin_ws/build/gazebo_test_tools/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fake_object_recognizer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/fake_object_recognizer.dir/build: /home/filippo/catkin_ws/devel/.private/gazebo_test_tools/lib/libfake_object_recognizer.so

.PHONY : CMakeFiles/fake_object_recognizer.dir/build

CMakeFiles/fake_object_recognizer.dir/requires: CMakeFiles/fake_object_recognizer.dir/src/FakeObjectRecognizer.cpp.o.requires

.PHONY : CMakeFiles/fake_object_recognizer.dir/requires

CMakeFiles/fake_object_recognizer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/fake_object_recognizer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/fake_object_recognizer.dir/clean

CMakeFiles/fake_object_recognizer.dir/depend:
	cd /home/filippo/catkin_ws/build/gazebo_test_tools && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/filippo/catkin_ws/src/lecture_5_new/gazebo-pkgs/gazebo_test_tools /home/filippo/catkin_ws/src/lecture_5_new/gazebo-pkgs/gazebo_test_tools /home/filippo/catkin_ws/build/gazebo_test_tools /home/filippo/catkin_ws/build/gazebo_test_tools /home/filippo/catkin_ws/build/gazebo_test_tools/CMakeFiles/fake_object_recognizer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/fake_object_recognizer.dir/depend
