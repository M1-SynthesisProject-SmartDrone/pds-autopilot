#!/bin/sh

set -e

# This is a simple compiler script that will create an executable in the "bin" folder

mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j
cd ..