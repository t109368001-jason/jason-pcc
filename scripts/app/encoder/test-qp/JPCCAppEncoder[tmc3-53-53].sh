#!/bin/bash
set -ex

OUTPUT_FOLDER_PREFIX="../../result/"
OUTPUT_FOLDER="encoded-$(date +%Y%m%d-%H)/ZX-XS-20220707[tmc3-53-53]/"

mkdir -p "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}"

./bin/JPCCAppEncoder \
  --jpccGMMSegmentationParameter.type none \
  --jpccEncoderDynamic.tmc3.positionQuantisationEnabled true \
  --jpccEncoderStatic.tmc3.positionQuantisationEnabled true \
  --jpccEncoderDynamic.tmc3.positionBaseQp 53 \
  --jpccEncoderStatic.tmc3.positionBaseQp 53 \
  --jpccEncoderDynamic.tmc3.positionQuantisationOctreeDepth 0 \
  --jpccEncoderStatic.tmc3.positionQuantisationOctreeDepth 0 \
  --app.compressedStreamPath "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}output.bin" \
  --jpccMetricParameter.outputCSVFolder "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}" \
  --configs cfg/app/Encoder/ctc-raw.cfg |& tee "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}JPCCAppEncoder-$(date +%Y%m%d-%H%M%S).log"
