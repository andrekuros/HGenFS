# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/ieav/share/QLGen

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ieav/share/QLGen

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	cd /home/ieav/share/QLGen && $(CMAKE_COMMAND) -E cmake_progress_start /home/ieav/share/QLGen/CMakeFiles /home/ieav/share/QLGen/QLGen/CMakeFiles/progress.marks
	cd /home/ieav/share/QLGen && $(MAKE) -f CMakeFiles/Makefile2 QLGen/all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/ieav/share/QLGen/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	cd /home/ieav/share/QLGen && $(MAKE) -f CMakeFiles/Makefile2 QLGen/clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	cd /home/ieav/share/QLGen && $(MAKE) -f CMakeFiles/Makefile2 QLGen/preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	cd /home/ieav/share/QLGen && $(MAKE) -f CMakeFiles/Makefile2 QLGen/preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	cd /home/ieav/share/QLGen && $(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

# Convenience name for target.
QLGen/CMakeFiles/QLGen.dir/rule:
	cd /home/ieav/share/QLGen && $(MAKE) -f CMakeFiles/Makefile2 QLGen/CMakeFiles/QLGen.dir/rule
.PHONY : QLGen/CMakeFiles/QLGen.dir/rule

# Convenience name for target.
QLGen: QLGen/CMakeFiles/QLGen.dir/rule

.PHONY : QLGen

# fast build rule for target.
QLGen/fast:
	cd /home/ieav/share/QLGen && $(MAKE) -f QLGen/CMakeFiles/QLGen.dir/build.make QLGen/CMakeFiles/QLGen.dir/build
.PHONY : QLGen/fast

QLGen.o: QLGen.cpp.o

.PHONY : QLGen.o

# target to build an object file
QLGen.cpp.o:
	cd /home/ieav/share/QLGen && $(MAKE) -f QLGen/CMakeFiles/QLGen.dir/build.make QLGen/CMakeFiles/QLGen.dir/QLGen.cpp.o
.PHONY : QLGen.cpp.o

QLGen.i: QLGen.cpp.i

.PHONY : QLGen.i

# target to preprocess a source file
QLGen.cpp.i:
	cd /home/ieav/share/QLGen && $(MAKE) -f QLGen/CMakeFiles/QLGen.dir/build.make QLGen/CMakeFiles/QLGen.dir/QLGen.cpp.i
.PHONY : QLGen.cpp.i

QLGen.s: QLGen.cpp.s

.PHONY : QLGen.s

# target to generate assembly for a file
QLGen.cpp.s:
	cd /home/ieav/share/QLGen && $(MAKE) -f QLGen/CMakeFiles/QLGen.dir/build.make QLGen/CMakeFiles/QLGen.dir/QLGen.cpp.s
.PHONY : QLGen.cpp.s

individual.o: individual.cpp.o

.PHONY : individual.o

# target to build an object file
individual.cpp.o:
	cd /home/ieav/share/QLGen && $(MAKE) -f QLGen/CMakeFiles/QLGen.dir/build.make QLGen/CMakeFiles/QLGen.dir/individual.cpp.o
.PHONY : individual.cpp.o

individual.i: individual.cpp.i

.PHONY : individual.i

# target to preprocess a source file
individual.cpp.i:
	cd /home/ieav/share/QLGen && $(MAKE) -f QLGen/CMakeFiles/QLGen.dir/build.make QLGen/CMakeFiles/QLGen.dir/individual.cpp.i
.PHONY : individual.cpp.i

individual.s: individual.cpp.s

.PHONY : individual.s

# target to generate assembly for a file
individual.cpp.s:
	cd /home/ieav/share/QLGen && $(MAKE) -f QLGen/CMakeFiles/QLGen.dir/build.make QLGen/CMakeFiles/QLGen.dir/individual.cpp.s
.PHONY : individual.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... QLGen"
	@echo "... QLGen.o"
	@echo "... QLGen.i"
	@echo "... QLGen.s"
	@echo "... individual.o"
	@echo "... individual.i"
	@echo "... individual.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	cd /home/ieav/share/QLGen && $(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system
