#!/bin/bash
set -ex

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/../JPCCAppSegmentationCommon.sh"

function JPCCAppSegmentationTestAlpha() {
  local resolution=${1}
  local alpha=${2}
  local output_folder="ZX-XS-20220707-segmentation[${resolution}][alpha-${alpha}]/"

  JPCCAppSegmentationCommon ${resolution} ${output_folder} \
    --jpccGMMSegmentationParameter.alpha ${alpha} \
    --jpccGMMSegmentationParameter.alpha 0.000255
}
