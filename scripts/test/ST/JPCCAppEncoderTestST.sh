#!/bin/bash
set -ex

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/../JPCCAppEncoderCommon.sh"

function JPCCAppEncoderTestST() {
  local resolution=${1}
  local qp=${2}
  local st1=${3}
  local st2=${4}
  local output_folder="${OUTPUT_FOLDER_PREFIX:="../../result/test-k-$(date +%Y%m%d-%H)/"}/ZX-XS-20220707[seg-split-${resolution}][tmc3-${qp}-0][ST-${st1}-${st2}]/"

  JPCCAppEncoderCommon ${resolution} ${qp} ${output_folder} \
    --jpccGMMSegmentationParameter.staticThreshold1 ${st1} \
    --jpccGMMSegmentationParameter.staticThreshold1 ${st1} \
    --jpccGMMSegmentationParameter.staticThreshold2 ${st2} \
    --jpccGMMSegmentationParameter.staticThreshold2 ${st2}
}
