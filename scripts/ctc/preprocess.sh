#!/bin/bash
set -ex

mkdir -p ../../dataset/converted/ZX-XS-20220707-preprocess

./bin/JPCCAppDatasetPreprocess --configs ./cfg/app/DatasetPreprocess/ctc.cfg |& tee ../../dataset/converted/ZX-XS-20220707-preprocess/JPCCAppDatasetPreprocess-$(date +%Y%m%d-%H%M%S).log
