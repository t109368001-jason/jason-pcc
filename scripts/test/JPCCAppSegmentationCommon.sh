#!/bin/bash
set -ex

function JPCCAppSegmentationCommon() {
  local resolution=${1}
  local output_folder=${2}

  mkdir -p "../../result/${output_folder}"

  ./bin/JPCCAppSegmentation \
    ${@:-3} \
    --outputDataset.folder ${output_folder} \
    --jpccGMMSegmentationParameter.outputType dynamic-staticAdded-staticRemoved \
    --jpccGMMSegmentationParameter.resolution ${resolution} \
    --configs cfg/app/Segmentation/ctc.cfg |& tee "../../result/${output_folder}/JPCCAppSegmentation-$(date +%Y%m%d-%H%M%S).log"
}
