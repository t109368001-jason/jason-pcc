#!/bin/bash
set -ex

OUTPUT_FOLDER_PREFIX=../../dataset/
OUTPUT_FOLDER=converted-$(date +%Y%m%d)/ZX-XS-20220707-ctc-gmm-segmentation/

mkdir -p ${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}

./bin/JPCCAppSegmentation \
  --outputDataset.folder ${OUTPUT_FOLDER} \
  --configs cfg/app/Segmentation/ctc-raw.cfg |& tee ${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}JPCCAppSegmentation-$(date +%Y%m%d-%H%M%S).log
