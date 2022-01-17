#!/bin/bash
set -x

cd build && ctest -j$(nproc) -T test --output-on-failure -T memcheck