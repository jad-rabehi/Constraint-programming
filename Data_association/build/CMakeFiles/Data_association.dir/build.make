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
CMAKE_SOURCE_DIR = /home/rabehi/Documents/github/Constraint-programming/Data_association

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rabehi/Documents/github/Constraint-programming/Data_association/build

# Include any dependencies generated for this target.
include CMakeFiles/Data_association.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Data_association.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Data_association.dir/flags.make

CMakeFiles/Data_association.dir/main.cpp.o: CMakeFiles/Data_association.dir/flags.make
CMakeFiles/Data_association.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rabehi/Documents/github/Constraint-programming/Data_association/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Data_association.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Data_association.dir/main.cpp.o -c /home/rabehi/Documents/github/Constraint-programming/Data_association/main.cpp

CMakeFiles/Data_association.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Data_association.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rabehi/Documents/github/Constraint-programming/Data_association/main.cpp > CMakeFiles/Data_association.dir/main.cpp.i

CMakeFiles/Data_association.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Data_association.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rabehi/Documents/github/Constraint-programming/Data_association/main.cpp -o CMakeFiles/Data_association.dir/main.cpp.s

CMakeFiles/Data_association.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/Data_association.dir/main.cpp.o.requires

CMakeFiles/Data_association.dir/main.cpp.o.provides: CMakeFiles/Data_association.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Data_association.dir/build.make CMakeFiles/Data_association.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/Data_association.dir/main.cpp.o.provides

CMakeFiles/Data_association.dir/main.cpp.o.provides.build: CMakeFiles/Data_association.dir/main.cpp.o


# Object files for target Data_association
Data_association_OBJECTS = \
"CMakeFiles/Data_association.dir/main.cpp.o"

# External object files for target Data_association
Data_association_EXTERNAL_OBJECTS =

Data_association: CMakeFiles/Data_association.dir/main.cpp.o
Data_association: CMakeFiles/Data_association.dir/build.make
Data_association: /usr/local/lib/libtubex.a
Data_association: /usr/local/lib/libtubex-rob.a
Data_association: /usr/local/lib/libtubex-pyibex.a
Data_association: /usr/local/lib/libibex.a
Data_association: /usr/local/lib/libtubex.a
Data_association: /usr/local/lib/libtubex-rob.a
Data_association: /usr/local/lib/libtubex-pyibex.a
Data_association: /usr/local/lib/libibex.a
Data_association: /usr/local/lib/ibex/3rd/libgaol.a
Data_association: /usr/local/lib/ibex/3rd/libgdtoa.a
Data_association: /usr/local/lib/ibex/3rd/libultim.a
Data_association: CMakeFiles/Data_association.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rabehi/Documents/github/Constraint-programming/Data_association/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Data_association"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Data_association.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Data_association.dir/build: Data_association

.PHONY : CMakeFiles/Data_association.dir/build

CMakeFiles/Data_association.dir/requires: CMakeFiles/Data_association.dir/main.cpp.o.requires

.PHONY : CMakeFiles/Data_association.dir/requires

CMakeFiles/Data_association.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Data_association.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Data_association.dir/clean

CMakeFiles/Data_association.dir/depend:
	cd /home/rabehi/Documents/github/Constraint-programming/Data_association/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rabehi/Documents/github/Constraint-programming/Data_association /home/rabehi/Documents/github/Constraint-programming/Data_association /home/rabehi/Documents/github/Constraint-programming/Data_association/build /home/rabehi/Documents/github/Constraint-programming/Data_association/build /home/rabehi/Documents/github/Constraint-programming/Data_association/build/CMakeFiles/Data_association.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Data_association.dir/depend
