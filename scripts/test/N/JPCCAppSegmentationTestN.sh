#!/bin/bash
set -ex

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/../JPCCAppSegmentationCommon.sh"

function JPCCAppSegmentationTestN() {
  local resolution=${1}
  local n=${2}
  local output_folder="ZX-XS-20220707-segmentation[${resolution}][n-${n}]/"

  JPCCAppSegmentationCommon ${resolution} ${output_folder} \
    --jpccGMMSegmentationParameter.nTrain ${n} \
    --jpccGMMSegmentationParameter.nTrain 18090
}
