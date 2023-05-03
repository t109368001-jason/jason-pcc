#!/bin/bash
set -ex

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/../JPCCAppEncoderCommon.sh"

function JPCCAppEncoderTestBest() {
  local resolution=${1}
  local qp=${2}
  local output_folder="${OUTPUT_FOLDER_PREFIX:="../../result/test-best-$(date +%Y%m%d-%H)/"}/ZX-XS-20220707[seg-split-${resolution}][tmc3-${qp}-0]/"

  JPCCAppEncoderCommon ${resolution} ${qp} ${output_folder} \
    --jpccGMMSegmentationParameter.nTrain 350 \
    --jpccGMMSegmentationParameter.nTrain 18090 \
    --jpccGMMSegmentationParameter.alpha 0.01310 \
    --jpccGMMSegmentationParameter.alpha 0.000255 \
    --jpccGMMSegmentationParameter.staticThreshold1 0.001497 \
    --jpccGMMSegmentationParameter.staticThreshold1 0.001497 \
    --jpccGMMSegmentationParameter.staticThreshold2 0.001109 \
    --jpccGMMSegmentationParameter.staticThreshold2 0.001109 \
    --jpccGMMSegmentationParameter.nullStaticThreshold1 -0.031117476 \
    --jpccGMMSegmentationParameter.nullStaticThreshold1 -0.031117476 \
    --jpccGMMSegmentationParameter.nullStaticThreshold2 -0.032713244 \
    --jpccGMMSegmentationParameter.nullStaticThreshold2 -0.032713244


}

JPCCAppEncoderTestBest 100 53 &
JPCCAppEncoderTestBest 200 60 &
JPCCAppEncoderTestBest 300 65 &
JPCCAppEncoderTestBest 400 68 &
JPCCAppEncoderTestBest 500 71 &