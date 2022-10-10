#!/bin/bash
set -ex

OUTPUT_FOLDER_PREFIX="../../result/"
OUTPUT_FOLDER="encoded-$(date +%Y%m%d-%H)/ZX-XS-20220707[seg-split][resolution-300]/"

mkdir -p "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}"

./bin/JPCCAppEncoder \
  --outputDataset.folder "${OUTPUT_FOLDER}" \
  --jpccGMMSegmentationParameter.outputType dynamic-staticAdded-staticRemoved \
  --jpccGMMSegmentationParameter.resolution 300.0 \
  --app.compressedDynamicStreamPath "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}output-dynamic.bin" \
  --app.compressedStaticStreamPath "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}output-static.bin" \
  --app.compressedStaticAddedStreamPath "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}output-staticAdded.bin" \
  --app.compressedStaticRemovedStreamPath "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}output-staticRemoved.bin" \
  --jpccMetricParameter.outputCSVFolder "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}" \
  --configs cfg/app/Encoder/ctc-raw.cfg |& tee "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}JPCCAppEncoder-$(date +%Y%m%d-%H%M%S).log"
