# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.6

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/delpart/ba/bullet-2.82-r2704

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/delpart/ba/bullet-2.82-r2704/bullet-build

# Include any dependencies generated for this target.
include Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/depend.make

# Include the progress variables for this target.
include Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/progress.make

# Include the compile flags for this target's objects.
include Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/flags.make

Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o: Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/flags.make
Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o: ../Demos/SoftDemo/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/delpart/ba/bullet-2.82-r2704/bullet-build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o"
	cd /home/delpart/ba/bullet-2.82-r2704/bullet-build/Demos/SoftDemo && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/AppSoftBodyDemo.dir/main.o -c /home/delpart/ba/bullet-2.82-r2704/Demos/SoftDemo/main.cpp

Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AppSoftBodyDemo.dir/main.i"
	cd /home/delpart/ba/bullet-2.82-r2704/bullet-build/Demos/SoftDemo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/delpart/ba/bullet-2.82-r2704/Demos/SoftDemo/main.cpp > CMakeFiles/AppSoftBodyDemo.dir/main.i

Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AppSoftBodyDemo.dir/main.s"
	cd /home/delpart/ba/bullet-2.82-r2704/bullet-build/Demos/SoftDemo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/delpart/ba/bullet-2.82-r2704/Demos/SoftDemo/main.cpp -o CMakeFiles/AppSoftBodyDemo.dir/main.s

Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.requires:
.PHONY : Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.requires

Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.provides: Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.requires
	$(MAKE) -f Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/build.make Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.provides.build
.PHONY : Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.provides

Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.provides.build: Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o
.PHONY : Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.provides.build

Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o: Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/flags.make
Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o: ../Demos/SoftDemo/SoftDemo.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/delpart/ba/bullet-2.82-r2704/bullet-build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o"
	cd /home/delpart/ba/bullet-2.82-r2704/bullet-build/Demos/SoftDemo && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o -c /home/delpart/ba/bullet-2.82-r2704/Demos/SoftDemo/SoftDemo.cpp

Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.i"
	cd /home/delpart/ba/bullet-2.82-r2704/bullet-build/Demos/SoftDemo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/delpart/ba/bullet-2.82-r2704/Demos/SoftDemo/SoftDemo.cpp > CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.i

Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.s"
	cd /home/delpart/ba/bullet-2.82-r2704/bullet-build/Demos/SoftDemo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/delpart/ba/bullet-2.82-r2704/Demos/SoftDemo/SoftDemo.cpp -o CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.s

Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.requires:
.PHONY : Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.requires

Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.provides: Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.requires
	$(MAKE) -f Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/build.make Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.provides.build
.PHONY : Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.provides

Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.provides.build: Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o
.PHONY : Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.provides.build

# Object files for target AppSoftBodyDemo
AppSoftBodyDemo_OBJECTS = \
"CMakeFiles/AppSoftBodyDemo.dir/main.o" \
"CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o"

# External object files for target AppSoftBodyDemo
AppSoftBodyDemo_EXTERNAL_OBJECTS =

Demos/SoftDemo/AppSoftBodyDemo: Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o
Demos/SoftDemo/AppSoftBodyDemo: Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o
Demos/SoftDemo/AppSoftBodyDemo: Demos/OpenGL/libOpenGLSupport.so
Demos/SoftDemo/AppSoftBodyDemo: src/BulletSoftBody/libBulletSoftBody.so.2.82
Demos/SoftDemo/AppSoftBodyDemo: src/BulletDynamics/libBulletDynamics.so.2.82
Demos/SoftDemo/AppSoftBodyDemo: src/BulletCollision/libBulletCollision.so.2.82
Demos/SoftDemo/AppSoftBodyDemo: src/LinearMath/libLinearMath.so.2.82
Demos/SoftDemo/AppSoftBodyDemo: /usr/lib64/libglut.so
Demos/SoftDemo/AppSoftBodyDemo: /usr/lib64/libGL.so
Demos/SoftDemo/AppSoftBodyDemo: /usr/lib64/libGLU.so
Demos/SoftDemo/AppSoftBodyDemo: Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/build.make
Demos/SoftDemo/AppSoftBodyDemo: Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable AppSoftBodyDemo"
	cd /home/delpart/ba/bullet-2.82-r2704/bullet-build/Demos/SoftDemo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/AppSoftBodyDemo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/build: Demos/SoftDemo/AppSoftBodyDemo
.PHONY : Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/build

Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/requires: Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.requires
Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/requires: Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.requires
.PHONY : Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/requires

Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/clean:
	cd /home/delpart/ba/bullet-2.82-r2704/bullet-build/Demos/SoftDemo && $(CMAKE_COMMAND) -P CMakeFiles/AppSoftBodyDemo.dir/cmake_clean.cmake
.PHONY : Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/clean

Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/depend:
	cd /home/delpart/ba/bullet-2.82-r2704/bullet-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/delpart/ba/bullet-2.82-r2704 /home/delpart/ba/bullet-2.82-r2704/Demos/SoftDemo /home/delpart/ba/bullet-2.82-r2704/bullet-build /home/delpart/ba/bullet-2.82-r2704/bullet-build/Demos/SoftDemo /home/delpart/ba/bullet-2.82-r2704/bullet-build/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/depend

