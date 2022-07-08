#!/bin/bash

cd build

cmake ..
make

cd bin

./findarmor
