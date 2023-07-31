#!/bin/bash
set -ex

function JPCCAppEncoderCommon() {
  local resolution=${1}
  local qp=${2}
  local output_folder=${3}

  mkdir -p "${output_folder}"

  ./bin/JPCCAppEncoder \
    ${@:4} \
    --jpccGMMSegmentationParameter.outputType dynamic-staticAdded-staticRemoved \
    --jpccGMMSegmentationParameter.resolution ${resolution} \
    --jpccEncoderDynamic.tmc3.positionQuantisationEnabled true \
    --jpccEncoderDynamic.tmc3.positionBaseQp ${qp} \
    --jpccEncoderDynamic.tmc3.positionQuantisationOctreeDepth 0 \
    --jpccEncoderStatic.tmc3.positionBaseQp ${qp} \
    --jpccEncoderStatic.tmc3.positionQuantisationOctreeDepth 0 \
    --app.compressedStreamPath "${output_folder}/output.bin" \
    --jpccMetricParameter.outputCSVFolder "${output_folder}/" \
    --configs cfg/app/Encoder/ctc-raw.cfg |& tee "${output_folder}/JPCCAppEncoder-$(date +%Y%m%d-%H%M%S).log"
}
