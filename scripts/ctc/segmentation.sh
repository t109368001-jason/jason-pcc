#!/bin/bash
set -ex

mkdir -p ../../result/ZX-XS-20220707-segmentation

./bin/JPCCAppSegmentation --configs ./cfg/app/Segmentation/ctc.cfg |& tee ../../resukt/ZX-XS-20220707-segmentation/JPCCAppSegmentation-$(date +%Y%m%d-%H%M%S).log
