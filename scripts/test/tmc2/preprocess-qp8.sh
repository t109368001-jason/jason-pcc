#!/bin/bash
set -ex

function PccAppEncoderTest() {
  local condition=${1}
  local rate=${2}
  local output_folder="tmc2/ZX-XS-20220707-encoded[${condition}][${rate}]/"

  mkdir -p "../../result/${output_folder}/"

  ./bin/PccAppEncoder \
    --config="./3rdparty/mpeg-pcc-tmc2/cfg/common/ctc-common.cfg" \
    --config="./3rdparty/mpeg-pcc-tmc2/cfg/sequence/ZX-XS-20220707-preprocess-qp8.cfg" \
    --config="./3rdparty/mpeg-pcc-tmc2/cfg/condition/ctc-${condition}.cfg" \
    --config="./3rdparty/mpeg-pcc-tmc2/cfg/rate/ctc-${rate}.cfg" \
    --reconstructedDataPath="../../result/${output_folder}/rec_%05d.ply" \
    --compressedStreamPath="../../result/${output_folder}/output.bin" \
    --configurationFolder=./3rdparty/mpeg-pcc-tmc2/cfg/ \
    ${@:3} |& tee "../../result/${output_folder}/PccAppEncoder-$(date +%Y%m%d-%H%M%S).log"
}
