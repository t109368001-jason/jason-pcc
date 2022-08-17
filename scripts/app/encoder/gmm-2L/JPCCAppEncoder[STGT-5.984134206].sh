#!/bin/bash
set -ex

OUTPUT_FOLDER_PREFIX="../../dataset/"
OUTPUT_FOLDER="converted-$(date +%Y%m%d)/ZX-XS-20220707-ctc-gmm-segmentation[STGT-5.984134206]/"

mkdir -p "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}"

./bin/JPCCAppEncoder \
  --jpccGMMSegmentationParameter.nullStaticThreshold -5.984134206 \
  --jpccGMMSegmentationParameter.nullStaticThreshold -5.984134206 \
  --outputDataset.folder "${OUTPUT_FOLDER}" \
  --configs cfg/app/Segmentation/ctc-raw.cfg |& tee "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}JPCCAppEncoder-$(date +%Y%m%d-%H%M%S).log"
