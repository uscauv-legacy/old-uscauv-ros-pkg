# set EXTRA_CMAKE_FLAGS in the including Makefile in order to add tweaks
CMAKE_FLAGS= -Wdev -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake $(EXTRA_CMAKE_FLAGS)
ROS_MAKE_FLAGS=$(ROS_PARALLEL_JOBS) $(ROS_OPTIMIZATION_LEVEL)

# make sure we default to all
all:

# setup cmake
any:
	@mkdir -p build
	cd build && cmake $(CMAKE_FLAGS) ..

remake:
	make clean;

#forward all other commands, calling 'any' first if necessary
%:
	if [ -r build ]; then cd build && make $@ $(ROS_MAKE_FLAGS); else make any && make $@; fi

PACKAGE_NAME=$(shell basename $(PWD))

clean:
	-if [ -r build ]; then cd build && make clean; fi
	-rm -rf build
	-rm -rf lib
	-rm -rf bin
	# clear out auto-generated docs
	-rm -rf docs
	# clear out auto-generated messages
	-rm -rf msg/cpp
	# clear out auto-generated services
	-rm -rf srv/cpp
	# clear out any auto-generated dynamic reconfigure stuff
	-rm -rf cfg/cpp
	-rm -rf cfg/*.cfgc
	-rm -rf src/$(PACKAGE_NAME)

include $(shell rospack find mk)/buildtest.mk
