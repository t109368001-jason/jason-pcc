#!/bin/bash
set -ex

mkdir -p ./logs

./bin/JPCCAppGroundSegmentation --configs ./cfg/app/GroundSegmentation/ZX-XS-20220330.cfg |& tee ./logs/JPCCAppGroundSegmentation.log
