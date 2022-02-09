#!/bin/bash
set -ex

cd build && ctest -j$(nproc) --overwrite MemoryCheckCommandOptions="--leak-check=full --error-exitcode=1" -T memcheck