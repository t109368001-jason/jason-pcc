#!/bin/bash
set -ex

OUTPUT_FOLDER_PREFIX=../../dataset/
OUTPUT_FOLDER=converted/ZX-XS-20220707-gmm-dynamic-staticAdded-staticRemoved[N-2048]/

mkdir -p ${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}

./bin/JPCCAppSegmentation \
  --jpccGMMSegmentationParameter.nTrain 2048 \
  --outputDataset.folder ${OUTPUT_FOLDER} \
  --configs cfg/app/Segmentation/ctc-raw.cfg |& tee ${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}JPCCAppSegmentation-$(date +%Y%m%d-%H%M%S).log
