#!/bin/bash
set -ex

cmake -DCMAKE_BUILD_TYPE:STRING=Debug -H. -B./build && \
    cmake --build build --target all -j$(nproc) --
