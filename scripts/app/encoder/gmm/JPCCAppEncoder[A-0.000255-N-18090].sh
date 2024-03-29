#!/bin/bash
set -ex

OUTPUT_FOLDER_PREFIX="../../dataset/"
OUTPUT_FOLDER="converted-$(date +%Y%m%d-%H)/ZX-XS-20220707-ctc-gmm-segmentation[A-0.000255-N-18090]/"

mkdir -p "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}"

./bin/JPCCAppEncoder \
  --jpccGMMSegmentationParameter.alpha 0.000255 \
  --jpccGMMSegmentationParameter.nTrain 18090 \
  --outputDataset.folder "${OUTPUT_FOLDER}" \
  --configs cfg/app/Encoder/ctc-raw.cfg |& tee "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}JPCCAppEncoder-$(date +%Y%m%d-%H%M%S).log"
