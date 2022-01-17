#!/bin/bash
set -x

cmake -DCMAKE_BUILD_TYPE:STRING=Release -H. -B./build && \
    cmake --build build --target all -j$(nproc) --
