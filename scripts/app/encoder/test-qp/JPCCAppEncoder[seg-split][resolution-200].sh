#!/bin/bash
set -ex

OUTPUT_FOLDER_PREFIX="../../result/"
OUTPUT_FOLDER="encoded-$(date +%Y%m%d-%H)/ZX-XS-20220707[seg-split][resolution-200]/"

mkdir -p "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}"

./bin/JPCCAppEncoder \
  --jpccGMMSegmentationParameter.outputType dynamic-staticAdded-staticRemoved \
  --jpccGMMSegmentationParameter.resolution 200.0 \
  --app.compressedStreamPath "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}output.bin" \
  --jpccMetricParameter.outputCSVFolder "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}" \
  --configs cfg/app/Encoder/ctc-raw.cfg |& tee "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}JPCCAppEncoder-$(date +%Y%m%d-%H%M%S).log"
