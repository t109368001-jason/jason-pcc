#!/bin/bash
set -ex

mkdir -p ../../dataset/converted/ZX-XS-20220707-preprocess-qp4

./bin/JPCCAppDatasetQuantizer --configs ./cfg/app/DatasetQuantizer/qp4.cfg |& tee ../../dataset/converted/ZX-XS-20220707-preprocess-qp4/JPCCAppDatasetQuantizer-$(date +%Y%m%d-%H%M%S).log
