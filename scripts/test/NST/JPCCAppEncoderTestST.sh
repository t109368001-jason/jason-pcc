#!/bin/bash
set -ex

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/../JPCCAppEncoderCommon.sh"

function JPCCAppEncoderTestST() {
  local resolution=${1}
  local qp=${2}
  local nst1=${3}
  local nst2=${4}
  local output_folder="${OUTPUT_FOLDER_PREFIX:="../../result/test-k-$(date +%Y%m%d-%H)/"}/ZX-XS-20220707[seg-split-${resolution}][tmc3-${qp}-0][NST-${nst1}-${nst2}]/"

  JPCCAppEncoderCommon ${resolution} ${qp} ${output_folder} \
    --jpccGMMSegmentationParameter.nullStaticThreshold1 ${nst1} \
    --jpccGMMSegmentationParameter.nullStaticThreshold1 ${nst1} \
    --jpccGMMSegmentationParameter.nullStaticThreshold2 ${nst2} \
    --jpccGMMSegmentationParameter.nullStaticThreshold2 ${nst2}
}
