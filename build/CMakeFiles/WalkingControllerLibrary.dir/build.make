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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rum/Sapienza/Excelent/mpc

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rum/Sapienza/Excelent/mpc/build

# Include any dependencies generated for this target.
include CMakeFiles/WalkingControllerLibrary.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/WalkingControllerLibrary.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/WalkingControllerLibrary.dir/flags.make

CMakeFiles/WalkingControllerLibrary.dir/src/DiscreteLIPDynamics.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/DiscreteLIPDynamics.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/DiscreteLIPDynamics.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/DiscreteLIPDynamics.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/DiscreteLIPDynamics.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/DiscreteLIPDynamics.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/DiscreteLIPDynamics.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/DiscreteLIPDynamics.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/DiscreteLIPDynamics.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/DiscreteLIPDynamics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/DiscreteLIPDynamics.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/DiscreteLIPDynamics.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/DiscreteLIPDynamics.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/DiscreteLIPDynamics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/DiscreteLIPDynamics.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/DiscreteLIPDynamics.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/DiscreteLIPDynamics.cpp.s

CMakeFiles/WalkingControllerLibrary.dir/src/DoubleSupportConfiguration.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/DoubleSupportConfiguration.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/DoubleSupportConfiguration.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/DoubleSupportConfiguration.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/DoubleSupportConfiguration.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/DoubleSupportConfiguration.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/DoubleSupportConfiguration.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/DoubleSupportConfiguration.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/DoubleSupportConfiguration.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/DoubleSupportConfiguration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/DoubleSupportConfiguration.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/DoubleSupportConfiguration.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/DoubleSupportConfiguration.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/DoubleSupportConfiguration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/DoubleSupportConfiguration.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/DoubleSupportConfiguration.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/DoubleSupportConfiguration.cpp.s

CMakeFiles/WalkingControllerLibrary.dir/src/FootstepPlanElement.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/FootstepPlanElement.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/FootstepPlanElement.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/FootstepPlanElement.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/FootstepPlanElement.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/FootstepPlanElement.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/FootstepPlanElement.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/FootstepPlanElement.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/FootstepPlanElement.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/FootstepPlanElement.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/FootstepPlanElement.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/FootstepPlanElement.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/FootstepPlanElement.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/FootstepPlanElement.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/FootstepPlanElement.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/FootstepPlanElement.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/FootstepPlanElement.cpp.s

CMakeFiles/WalkingControllerLibrary.dir/src/ISMPC.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/ISMPC.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/ISMPC.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/ISMPC.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/ISMPC.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/ISMPC.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/ISMPC.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/ISMPC.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/ISMPC.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/ISMPC.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/ISMPC.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/ISMPC.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/ISMPC.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/ISMPC.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/ISMPC.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/ISMPC.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/ISMPC.cpp.s

CMakeFiles/WalkingControllerLibrary.dir/src/JsonConverter.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/JsonConverter.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/JsonConverter.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/JsonConverter.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/JsonConverter.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/JsonConverter.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/JsonConverter.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/JsonConverter.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/JsonConverter.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/JsonConverter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/JsonConverter.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/JsonConverter.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/JsonConverter.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/JsonConverter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/JsonConverter.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/JsonConverter.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/JsonConverter.cpp.s

CMakeFiles/WalkingControllerLibrary.dir/src/LIPSimulator.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/LIPSimulator.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/LIPSimulator.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/LIPSimulator.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/LIPSimulator.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/LIPSimulator.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/LIPSimulator.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/LIPSimulator.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/LIPSimulator.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/LIPSimulator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/LIPSimulator.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/LIPSimulator.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/LIPSimulator.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/LIPSimulator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/LIPSimulator.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/LIPSimulator.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/LIPSimulator.cpp.s

CMakeFiles/WalkingControllerLibrary.dir/src/LIPState.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/LIPState.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/LIPState.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/LIPState.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/LIPState.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/LIPState.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/LIPState.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/LIPState.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/LIPState.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/LIPState.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/LIPState.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/LIPState.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/LIPState.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/LIPState.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/LIPState.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/LIPState.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/LIPState.cpp.s

CMakeFiles/WalkingControllerLibrary.dir/src/JointCommand.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/JointCommand.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/JointCommand.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/JointCommand.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/JointCommand.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/JointCommand.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/JointCommand.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/JointCommand.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/JointCommand.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/JointCommand.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/JointCommand.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/JointCommand.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/JointCommand.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/JointCommand.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/JointCommand.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/JointCommand.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/JointCommand.cpp.s

CMakeFiles/WalkingControllerLibrary.dir/src/JointState.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/JointState.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/JointState.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/JointState.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/JointState.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/JointState.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/JointState.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/JointState.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/JointState.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/JointState.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/JointState.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/JointState.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/JointState.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/JointState.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/JointState.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/JointState.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/JointState.cpp.s

CMakeFiles/WalkingControllerLibrary.dir/src/RobotState.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/RobotState.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/RobotState.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/RobotState.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/RobotState.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/RobotState.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/RobotState.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/RobotState.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/RobotState.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/RobotState.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/RobotState.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/RobotState.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/RobotState.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/RobotState.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/RobotState.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/RobotState.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/RobotState.cpp.s

CMakeFiles/WalkingControllerLibrary.dir/src/SE3.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/SE3.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/SE3.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/SE3.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/SE3.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/SE3.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/SE3.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/SE3.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/SE3.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/SE3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/SE3.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/SE3.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/SE3.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/SE3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/SE3.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/SE3.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/SE3.cpp.s

CMakeFiles/WalkingControllerLibrary.dir/src/utils.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/utils.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/utils.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/utils.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/utils.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/utils.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/utils.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/utils.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/utils.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/utils.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/utils.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/utils.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/utils.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/utils.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/utils.cpp.s

CMakeFiles/WalkingControllerLibrary.dir/src/WalkingData.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/WalkingData.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/WalkingData.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/WalkingData.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/WalkingData.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/WalkingData.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/WalkingData.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/WalkingData.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/WalkingData.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/WalkingData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/WalkingData.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/WalkingData.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/WalkingData.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/WalkingData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/WalkingData.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/WalkingData.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/WalkingData.cpp.s

CMakeFiles/WalkingControllerLibrary.dir/src/WalkingManager.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/WalkingManager.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/WalkingManager.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/WalkingManager.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/WalkingManager.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/WalkingManager.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/WalkingManager.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/WalkingManager.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/WalkingManager.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/WalkingManager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/WalkingManager.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/WalkingManager.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/WalkingManager.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/WalkingManager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/WalkingManager.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/WalkingManager.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/WalkingManager.cpp.s

CMakeFiles/WalkingControllerLibrary.dir/src/WalkingState.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/WalkingState.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/WalkingState.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/WalkingState.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/WalkingState.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/WalkingState.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/WalkingState.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/WalkingState.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/WalkingState.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/WalkingState.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/WalkingState.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/WalkingState.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/WalkingState.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/WalkingState.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/WalkingState.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/WalkingState.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/WalkingState.cpp.s

CMakeFiles/WalkingControllerLibrary.dir/src/WholeBodyController.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/flags.make
CMakeFiles/WalkingControllerLibrary.dir/src/WholeBodyController.cpp.o: /home/rum/Sapienza/Excelent/mpc/src/WholeBodyController.cpp
CMakeFiles/WalkingControllerLibrary.dir/src/WholeBodyController.cpp.o: CMakeFiles/WalkingControllerLibrary.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object CMakeFiles/WalkingControllerLibrary.dir/src/WholeBodyController.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WalkingControllerLibrary.dir/src/WholeBodyController.cpp.o -MF CMakeFiles/WalkingControllerLibrary.dir/src/WholeBodyController.cpp.o.d -o CMakeFiles/WalkingControllerLibrary.dir/src/WholeBodyController.cpp.o -c /home/rum/Sapienza/Excelent/mpc/src/WholeBodyController.cpp

CMakeFiles/WalkingControllerLibrary.dir/src/WholeBodyController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/WalkingControllerLibrary.dir/src/WholeBodyController.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rum/Sapienza/Excelent/mpc/src/WholeBodyController.cpp > CMakeFiles/WalkingControllerLibrary.dir/src/WholeBodyController.cpp.i

CMakeFiles/WalkingControllerLibrary.dir/src/WholeBodyController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/WalkingControllerLibrary.dir/src/WholeBodyController.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rum/Sapienza/Excelent/mpc/src/WholeBodyController.cpp -o CMakeFiles/WalkingControllerLibrary.dir/src/WholeBodyController.cpp.s

# Object files for target WalkingControllerLibrary
WalkingControllerLibrary_OBJECTS = \
"CMakeFiles/WalkingControllerLibrary.dir/src/DiscreteLIPDynamics.cpp.o" \
"CMakeFiles/WalkingControllerLibrary.dir/src/DoubleSupportConfiguration.cpp.o" \
"CMakeFiles/WalkingControllerLibrary.dir/src/FootstepPlanElement.cpp.o" \
"CMakeFiles/WalkingControllerLibrary.dir/src/ISMPC.cpp.o" \
"CMakeFiles/WalkingControllerLibrary.dir/src/JsonConverter.cpp.o" \
"CMakeFiles/WalkingControllerLibrary.dir/src/LIPSimulator.cpp.o" \
"CMakeFiles/WalkingControllerLibrary.dir/src/LIPState.cpp.o" \
"CMakeFiles/WalkingControllerLibrary.dir/src/JointCommand.cpp.o" \
"CMakeFiles/WalkingControllerLibrary.dir/src/JointState.cpp.o" \
"CMakeFiles/WalkingControllerLibrary.dir/src/RobotState.cpp.o" \
"CMakeFiles/WalkingControllerLibrary.dir/src/SE3.cpp.o" \
"CMakeFiles/WalkingControllerLibrary.dir/src/utils.cpp.o" \
"CMakeFiles/WalkingControllerLibrary.dir/src/WalkingData.cpp.o" \
"CMakeFiles/WalkingControllerLibrary.dir/src/WalkingManager.cpp.o" \
"CMakeFiles/WalkingControllerLibrary.dir/src/WalkingState.cpp.o" \
"CMakeFiles/WalkingControllerLibrary.dir/src/WholeBodyController.cpp.o"

# External object files for target WalkingControllerLibrary
WalkingControllerLibrary_EXTERNAL_OBJECTS =

libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/DiscreteLIPDynamics.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/DoubleSupportConfiguration.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/FootstepPlanElement.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/ISMPC.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/JsonConverter.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/LIPSimulator.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/LIPState.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/JointCommand.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/JointState.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/RobotState.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/SE3.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/utils.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/WalkingData.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/WalkingManager.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/WalkingState.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/src/WholeBodyController.cpp.o
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/build.make
libWalkingControllerLibrary.a: CMakeFiles/WalkingControllerLibrary.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/rum/Sapienza/Excelent/mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Linking CXX static library libWalkingControllerLibrary.a"
	$(CMAKE_COMMAND) -P CMakeFiles/WalkingControllerLibrary.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/WalkingControllerLibrary.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/WalkingControllerLibrary.dir/build: libWalkingControllerLibrary.a
.PHONY : CMakeFiles/WalkingControllerLibrary.dir/build

CMakeFiles/WalkingControllerLibrary.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/WalkingControllerLibrary.dir/cmake_clean.cmake
.PHONY : CMakeFiles/WalkingControllerLibrary.dir/clean

CMakeFiles/WalkingControllerLibrary.dir/depend:
	cd /home/rum/Sapienza/Excelent/mpc/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rum/Sapienza/Excelent/mpc /home/rum/Sapienza/Excelent/mpc /home/rum/Sapienza/Excelent/mpc/build /home/rum/Sapienza/Excelent/mpc/build /home/rum/Sapienza/Excelent/mpc/build/CMakeFiles/WalkingControllerLibrary.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/WalkingControllerLibrary.dir/depend

