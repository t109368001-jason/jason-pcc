#!/bin/bash
set -ex

OUTPUT_FOLDER_PREFIX=../../dataset/
OUTPUT_FOLDER=converted-$(date +%Y%m%d)/ZX-XS-20220707-ctc-gmm-segmentation[0.00328-N-1400]/

mkdir -p ${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}

./bin/JPCCAppSegmentation \
  --jpccGMMSegmentationParameter.alpha 0.00328 \
  --jpccGMMSegmentationParameter.nTrain 1400 \
  --outputDataset.folder ${OUTPUT_FOLDER} \
  --configs cfg/app/Segmentation/ctc-raw.cfg |& tee ${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}JPCCAppSegmentation-$(date +%Y%m%d-%H%M%S).log
