#!/bin/bash

cmake -DCMAKE_BUILD_TYPE:STRING=${BUILD_TYPE} -H. -B./build

cmake --build build --config ${BUILD_TYPE} --target all -j$(nproc) --
