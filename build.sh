#!/bin/bash

mkdir build
mkdir bin

cd build
cmake ..

cd ..
cmake --build ./build
cmake --install ./build

if [ $1 == "run" ]; then
  ./bin/main

elif [ $1 == "sim" ]; then
  ./bin/sim
fi