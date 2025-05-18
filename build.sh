#!/bin/bash

mkdir build
mkdir bin

cd build
cmake ..

cd ..
cmake --build ./build
cmake --install ./build