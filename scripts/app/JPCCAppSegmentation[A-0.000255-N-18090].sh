#!/bin/bash
set -ex

OUTPUT_FOLDER_PREFIX=../../dataset/
OUTPUT_FOLDER=converted/ZX-XS-20220707-ctc-gmm-segmentation[A-0.000255-N-18090]/

mkdir -p ${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}

./bin/JPCCAppSegmentation \
  --jpccGMMSegmentationParameter.alpha 0.000255 \
  --jpccGMMSegmentationParameter.nTrain 18090 \
  --outputDataset.folder ${OUTPUT_FOLDER} \
  --configs cfg/app/Segmentation/ctc-raw.cfg |& tee ${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}JPCCAppSegmentation-$(date +%Y%m%d-%H%M%S).log
