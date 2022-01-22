#!/bin/bash
set -ex

cd build && ctest -j$(nproc) -T test --output-on-failure -T memcheck