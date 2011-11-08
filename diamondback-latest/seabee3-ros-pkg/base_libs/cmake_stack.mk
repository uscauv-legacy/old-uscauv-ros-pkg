# set EXTRA_CMAKE_FLAGS in the including Makefile in order to add tweaks
CMAKE_FLAGS= -Wdev -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake $(EXTRA_CMAKE_FLAGS)

# make sure we default to all
all:
	rosmake;

remake:
	make clean;

#forward all other commands, calling 'any' first if necessary
%:
	for PACKAGE in $(shell rosstack contents $(STACK_NAME)); do if [ -r $$PACKAGE ]; then cd $$PACKAGE && make $@ && cd ..; fi; done

STACK_NAME=$(shell basename $(PWD))

package_source: all
	$(shell rospack find rosbuild)/bin/package_source.py $(CURDIR)
