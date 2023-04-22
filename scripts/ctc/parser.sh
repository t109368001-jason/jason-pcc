#!/bin/bash
set -ex

mkdir -p ../../dataset/converted/ZX-XS-20220707-ply

./bin/JPCCAppDatasetParser --configs ./cfg/app/DatasetParser/ctc.cfg |& tee ../../dataset/converted/ZX-XS-20220707-ply/JPCCAppDatasetParser-$(date +%Y%m%d-%H%M%S).log
