#!/bin/bash
set -ex

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/../JPCCAppSegmentationCommon.sh"

function JPCCAppSegmentationTestK() {
  local resolution=${1}
  local k=${2}
  local output_folder="ZX-XS-20220707-segmentation[${resolution}][k-${k}]/"

  JPCCAppSegmentationCommon ${resolution} ${output_folder} \
    --jpccGMMSegmentationParameter.k ${k} \
    --jpccGMMSegmentationParameter.k ${k}
}
