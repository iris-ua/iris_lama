MAKEFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MAKEFILE_DIR  := $(dir $(MAKEFILE_PATH))

BUILD_TYPE ?= Release
CMAKE_FLAGS := -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)

# use ninja if available
CMAKE_GENERATOR ?= $(shell command -v ninja > /dev/null 2>&1 && echo "Ninja" ||  echo "Unix Makefiles")

all: cmake

cmake: build/.ran-cmake
	cmake --build build

clean: build/.ran-cmake
	cmake --build build -t clean

build/.ran-cmake:
	mkdir -p build
	cd build && cmake -G '$(CMAKE_GENERATOR)' $(CMAKE_FLAGS) $(MAKEFILE_DIR)
	@touch $@

dist-clean:
	rm -rf build
