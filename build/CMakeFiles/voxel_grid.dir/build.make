# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/zxy/zxy/1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zxy/zxy/1/build

# Include any dependencies generated for this target.
include CMakeFiles/voxel_grid.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/voxel_grid.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/voxel_grid.dir/flags.make

CMakeFiles/voxel_grid.dir/voxel_grid.cpp.o: CMakeFiles/voxel_grid.dir/flags.make
CMakeFiles/voxel_grid.dir/voxel_grid.cpp.o: ../voxel_grid.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zxy/zxy/1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/voxel_grid.dir/voxel_grid.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/voxel_grid.dir/voxel_grid.cpp.o -c /home/zxy/zxy/1/voxel_grid.cpp

CMakeFiles/voxel_grid.dir/voxel_grid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/voxel_grid.dir/voxel_grid.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zxy/zxy/1/voxel_grid.cpp > CMakeFiles/voxel_grid.dir/voxel_grid.cpp.i

CMakeFiles/voxel_grid.dir/voxel_grid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/voxel_grid.dir/voxel_grid.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zxy/zxy/1/voxel_grid.cpp -o CMakeFiles/voxel_grid.dir/voxel_grid.cpp.s

CMakeFiles/voxel_grid.dir/voxel_grid.cpp.o.requires:

.PHONY : CMakeFiles/voxel_grid.dir/voxel_grid.cpp.o.requires

CMakeFiles/voxel_grid.dir/voxel_grid.cpp.o.provides: CMakeFiles/voxel_grid.dir/voxel_grid.cpp.o.requires
	$(MAKE) -f CMakeFiles/voxel_grid.dir/build.make CMakeFiles/voxel_grid.dir/voxel_grid.cpp.o.provides.build
.PHONY : CMakeFiles/voxel_grid.dir/voxel_grid.cpp.o.provides

CMakeFiles/voxel_grid.dir/voxel_grid.cpp.o.provides.build: CMakeFiles/voxel_grid.dir/voxel_grid.cpp.o


# Object files for target voxel_grid
voxel_grid_OBJECTS = \
"CMakeFiles/voxel_grid.dir/voxel_grid.cpp.o"

# External object files for target voxel_grid
voxel_grid_EXTERNAL_OBJECTS =

bin/voxel_grid: CMakeFiles/voxel_grid.dir/voxel_grid.cpp.o
bin/voxel_grid: CMakeFiles/voxel_grid.dir/build.make
bin/voxel_grid: /usr/lib/libpcl_outofcore.so
bin/voxel_grid: /usr/lib/libpcl_people.so
bin/voxel_grid: /usr/lib/libpcl_apps.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libboost_system.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libboost_regex.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libqhull.so
bin/voxel_grid: /usr/lib/libOpenNI.so
bin/voxel_grid: /usr/lib/libOpenNI2.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libfreetype.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libz.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libjpeg.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libpng.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libtiff.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
bin/voxel_grid: /usr/lib/libpcl_segmentation.so
bin/voxel_grid: /usr/lib/libpcl_surface.so
bin/voxel_grid: /usr/lib/libpcl_visualization.so
bin/voxel_grid: /usr/lib/libpcl_stereo.so
bin/voxel_grid: /usr/lib/libpcl_keypoints.so
bin/voxel_grid: /usr/lib/libpcl_tracking.so
bin/voxel_grid: /usr/lib/libpcl_recognition.so
bin/voxel_grid: /usr/lib/libpcl_ml.so
bin/voxel_grid: /usr/lib/libpcl_io.so
bin/voxel_grid: /usr/lib/libpcl_registration.so
bin/voxel_grid: /usr/lib/libpcl_features.so
bin/voxel_grid: /usr/lib/libpcl_filters.so
bin/voxel_grid: /usr/lib/libpcl_sample_consensus.so
bin/voxel_grid: /usr/lib/libpcl_search.so
bin/voxel_grid: /usr/lib/libpcl_kdtree.so
bin/voxel_grid: /usr/lib/libpcl_octree.so
bin/voxel_grid: /usr/lib/libpcl_common.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkftgl-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libfreetype.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkalglib-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkIOImage-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkIOCore-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtksys-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libvtkmetaio-6.2.so.6.2.0
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libz.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libGLU.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libGL.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libSM.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libICE.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libX11.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libXext.so
bin/voxel_grid: /usr/lib/x86_64-linux-gnu/libXt.so
bin/voxel_grid: CMakeFiles/voxel_grid.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zxy/zxy/1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/voxel_grid"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/voxel_grid.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/voxel_grid.dir/build: bin/voxel_grid

.PHONY : CMakeFiles/voxel_grid.dir/build

CMakeFiles/voxel_grid.dir/requires: CMakeFiles/voxel_grid.dir/voxel_grid.cpp.o.requires

.PHONY : CMakeFiles/voxel_grid.dir/requires

CMakeFiles/voxel_grid.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/voxel_grid.dir/cmake_clean.cmake
.PHONY : CMakeFiles/voxel_grid.dir/clean

CMakeFiles/voxel_grid.dir/depend:
	cd /home/zxy/zxy/1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zxy/zxy/1 /home/zxy/zxy/1 /home/zxy/zxy/1/build /home/zxy/zxy/1/build /home/zxy/zxy/1/build/CMakeFiles/voxel_grid.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/voxel_grid.dir/depend
