#!/bin/bash
set -ex

OUTPUT_FOLDER_PREFIX="../../result/"
OUTPUT_FOLDER="encoded-$(date +%Y%m%d-%H)/ZX-XS-20220707[seg][tmc3-85]/"

mkdir -p "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}"

./bin/JPCCAppEncoder \
  --outputDataset.folder "${OUTPUT_FOLDER}" \
  --jpccEncoderDynamic.tmc3.positionQuantisationEnabled true \
  --jpccEncoderStatic.tmc3.positionQuantisationEnabled true \
  --jpccEncoderDynamic.tmc3.positionBaseQp 85 \
  --jpccEncoderStatic.tmc3.positionBaseQp 85 \
  --jpccEncoderDynamic.tmc3.positionQuantisationOctreeDepth 0 \
  --jpccEncoderStatic.tmc3.positionQuantisationOctreeDepth 0 \
  --jpccMetricParameter.outputCSVFolder "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}" \
  --configs cfg/app/Encoder/ctc-raw.cfg |& tee "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}JPCCAppEncoder-$(date +%Y%m%d-%H%M%S).log"
