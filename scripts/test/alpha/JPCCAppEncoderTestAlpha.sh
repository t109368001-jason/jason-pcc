#!/bin/bash
set -ex

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/../JPCCAppEncoderCommon.sh"

function JPCCAppEncoderTestAlpha() {
  local resolution=${1}
  local qp=${2}
  local alpha=${3}
  local output_folder="${OUTPUT_FOLDER_PREFIX:="../../result/test-k-$(date +%Y%m%d-%H)/"}/ZX-XS-20220707[seg-split-${resolution}][tmc3-${qp}-0][alpha-${alpha}]/"

  JPCCAppEncoderCommon ${resolution} ${qp} ${output_folder} \
    --jpccGMMSegmentationParameter.alpha ${alpha} \
    --jpccGMMSegmentationParameter.alpha 0.000255
}
