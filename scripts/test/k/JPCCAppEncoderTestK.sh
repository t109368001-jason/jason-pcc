#!/bin/bash
set -ex

sourcce ../JPCCAppEncoderCommon.sh

function JPCCAppEncoderTestK() {
  LOCAL resolution=${1}
  LOCAL qp=${2}
  LOCAL k=${3}
  LOCAL output_folder="${OUTPUT_FOLDER_PREFIX:="../../result/test-k-$(date +%Y%m%d-%H)/"}/ZX-XS-20220707[seg-split-${resolution}][tmc3-${qp}-0][k-${k}]/"

  JPCCAppEncoderCommon ${resolution} ${qp} ${output_folder} \
    --jpccGMMSegmentationParameter.k ${k} \
    --jpccGMMSegmentationParameter.k ${k}
}
