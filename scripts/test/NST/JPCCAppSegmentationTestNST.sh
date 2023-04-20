#!/bin/bash
set -ex

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/../JPCCAppSegmentationCommon.sh"

function JPCCAppSegmentationTestNST() {
  local resolution=${1}
  local nst1=${2}
  local nst2=${3}
  local output_folder="ZX-XS-20220707-segmentation[${resolution}][NST-${nst1}-${nst2}]/"

  JPCCAppSegmentationCommon ${resolution} ${output_folder} \
    --jpccGMMSegmentationParameter.nullStaticThreshold1 ${nst1} \
    --jpccGMMSegmentationParameter.nullStaticThreshold1 ${nst1} \
    --jpccGMMSegmentationParameter.nullStaticThreshold2 ${nst2} \
    --jpccGMMSegmentationParameter.nullStaticThreshold2 ${nst2}
}
