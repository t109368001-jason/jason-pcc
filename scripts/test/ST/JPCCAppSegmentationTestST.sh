#!/bin/bash
set -ex

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/../JPCCAppSegmentationCommon.sh"

function JPCCAppSegmentationTestST() {
  local resolution=${1}
  local st1=${2}
  local st2=${3}
  local output_folder="ZX-XS-20220707-segmentation[${resolution}][ST-${st1}-${st2}]/"

  JPCCAppSegmentationCommon ${resolution} ${output_folder} \
    --jpccGMMSegmentationParameter.staticThreshold1 ${st1} \
    --jpccGMMSegmentationParameter.staticThreshold1 ${st1} \
    --jpccGMMSegmentationParameter.staticThreshold2 ${st2} \
    --jpccGMMSegmentationParameter.staticThreshold2 ${st2}
}
