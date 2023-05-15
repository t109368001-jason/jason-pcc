#!/bin/bash
set -ex

mkdir -p ../../dataset/converted/ZX-XS-20220707-preprocess-qp8

./bin/JPCCAppDatasetQuantizer --configs ./cfg/app/DatasetQuantizer/qp8.cfg |& tee ../../dataset/converted/ZX-XS-20220707-preprocess-qp8/JPCCAppDatasetQuantizer-$(date +%Y%m%d-%H%M%S).log
