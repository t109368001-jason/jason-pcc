#!/bin/bash
set -ex

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/../JPCCAppEncoderCommon.sh"

function JPCCAppEncoderTestN() {
  local resolution=${1}
  local qp=${2}
  local n=${3}
  local output_folder="${OUTPUT_FOLDER_PREFIX:="../../result/test-k-$(date +%Y%m%d-%H)/"}/ZX-XS-20220707[seg-split-${resolution}][tmc3-${qp}-0][n-${n}]/"

  JPCCAppEncoderCommon ${resolution} ${qp} ${output_folder} \
    --jpccGMMSegmentationParameter.nTrain ${n} \
    --jpccGMMSegmentationParameter.nTrain 18090
}
