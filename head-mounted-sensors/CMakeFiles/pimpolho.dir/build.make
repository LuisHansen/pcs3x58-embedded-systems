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
CMAKE_SOURCE_DIR = /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors

# Include any dependencies generated for this target.
include CMakeFiles/pimpolho.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pimpolho.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pimpolho.dir/flags.make

CMakeFiles/pimpolho.dir/src/9dof.cc.o: CMakeFiles/pimpolho.dir/flags.make
CMakeFiles/pimpolho.dir/src/9dof.cc.o: src/9dof.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pimpolho.dir/src/9dof.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pimpolho.dir/src/9dof.cc.o -c /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors/src/9dof.cc

CMakeFiles/pimpolho.dir/src/9dof.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pimpolho.dir/src/9dof.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors/src/9dof.cc > CMakeFiles/pimpolho.dir/src/9dof.cc.i

CMakeFiles/pimpolho.dir/src/9dof.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pimpolho.dir/src/9dof.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors/src/9dof.cc -o CMakeFiles/pimpolho.dir/src/9dof.cc.s

CMakeFiles/pimpolho.dir/src/9dof.cc.o.requires:

.PHONY : CMakeFiles/pimpolho.dir/src/9dof.cc.o.requires

CMakeFiles/pimpolho.dir/src/9dof.cc.o.provides: CMakeFiles/pimpolho.dir/src/9dof.cc.o.requires
	$(MAKE) -f CMakeFiles/pimpolho.dir/build.make CMakeFiles/pimpolho.dir/src/9dof.cc.o.provides.build
.PHONY : CMakeFiles/pimpolho.dir/src/9dof.cc.o.provides

CMakeFiles/pimpolho.dir/src/9dof.cc.o.provides.build: CMakeFiles/pimpolho.dir/src/9dof.cc.o


CMakeFiles/pimpolho.dir/src/main.cc.o: CMakeFiles/pimpolho.dir/flags.make
CMakeFiles/pimpolho.dir/src/main.cc.o: src/main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/pimpolho.dir/src/main.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pimpolho.dir/src/main.cc.o -c /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors/src/main.cc

CMakeFiles/pimpolho.dir/src/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pimpolho.dir/src/main.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors/src/main.cc > CMakeFiles/pimpolho.dir/src/main.cc.i

CMakeFiles/pimpolho.dir/src/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pimpolho.dir/src/main.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors/src/main.cc -o CMakeFiles/pimpolho.dir/src/main.cc.s

CMakeFiles/pimpolho.dir/src/main.cc.o.requires:

.PHONY : CMakeFiles/pimpolho.dir/src/main.cc.o.requires

CMakeFiles/pimpolho.dir/src/main.cc.o.provides: CMakeFiles/pimpolho.dir/src/main.cc.o.requires
	$(MAKE) -f CMakeFiles/pimpolho.dir/build.make CMakeFiles/pimpolho.dir/src/main.cc.o.provides.build
.PHONY : CMakeFiles/pimpolho.dir/src/main.cc.o.provides

CMakeFiles/pimpolho.dir/src/main.cc.o.provides.build: CMakeFiles/pimpolho.dir/src/main.cc.o


CMakeFiles/pimpolho.dir/src/serial.c.o: CMakeFiles/pimpolho.dir/flags.make
CMakeFiles/pimpolho.dir/src/serial.c.o: src/serial.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/pimpolho.dir/src/serial.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pimpolho.dir/src/serial.c.o   -c /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors/src/serial.c

CMakeFiles/pimpolho.dir/src/serial.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pimpolho.dir/src/serial.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors/src/serial.c > CMakeFiles/pimpolho.dir/src/serial.c.i

CMakeFiles/pimpolho.dir/src/serial.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pimpolho.dir/src/serial.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors/src/serial.c -o CMakeFiles/pimpolho.dir/src/serial.c.s

CMakeFiles/pimpolho.dir/src/serial.c.o.requires:

.PHONY : CMakeFiles/pimpolho.dir/src/serial.c.o.requires

CMakeFiles/pimpolho.dir/src/serial.c.o.provides: CMakeFiles/pimpolho.dir/src/serial.c.o.requires
	$(MAKE) -f CMakeFiles/pimpolho.dir/build.make CMakeFiles/pimpolho.dir/src/serial.c.o.provides.build
.PHONY : CMakeFiles/pimpolho.dir/src/serial.c.o.provides

CMakeFiles/pimpolho.dir/src/serial.c.o.provides.build: CMakeFiles/pimpolho.dir/src/serial.c.o


# Object files for target pimpolho
pimpolho_OBJECTS = \
"CMakeFiles/pimpolho.dir/src/9dof.cc.o" \
"CMakeFiles/pimpolho.dir/src/main.cc.o" \
"CMakeFiles/pimpolho.dir/src/serial.c.o"

# External object files for target pimpolho
pimpolho_EXTERNAL_OBJECTS =

pimpolho: CMakeFiles/pimpolho.dir/src/9dof.cc.o
pimpolho: CMakeFiles/pimpolho.dir/src/main.cc.o
pimpolho: CMakeFiles/pimpolho.dir/src/serial.c.o
pimpolho: CMakeFiles/pimpolho.dir/build.make
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
pimpolho: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
pimpolho: CMakeFiles/pimpolho.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable pimpolho"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pimpolho.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pimpolho.dir/build: pimpolho

.PHONY : CMakeFiles/pimpolho.dir/build

CMakeFiles/pimpolho.dir/requires: CMakeFiles/pimpolho.dir/src/9dof.cc.o.requires
CMakeFiles/pimpolho.dir/requires: CMakeFiles/pimpolho.dir/src/main.cc.o.requires
CMakeFiles/pimpolho.dir/requires: CMakeFiles/pimpolho.dir/src/serial.c.o.requires

.PHONY : CMakeFiles/pimpolho.dir/requires

CMakeFiles/pimpolho.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pimpolho.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pimpolho.dir/clean

CMakeFiles/pimpolho.dir/depend:
	cd /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors /home/luis/Documentos/embarcados/pcs3x58-embedded-systems/head-mounted-sensors/CMakeFiles/pimpolho.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pimpolho.dir/depend

