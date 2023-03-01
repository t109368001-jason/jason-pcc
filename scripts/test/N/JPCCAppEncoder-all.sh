#!/bin/bash
set -ex

OUTPUT_FOLDER_PREFIX=${OUTPUT_FOLDER_PREFIX:="../../result/test-N-$(date +%Y%m%d-%H)/"}

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/JPCCAppEncoderTestN.sh"

JPCCAppEncoderTestN 100 53 175 &
JPCCAppEncoderTestN 200 60 175 &
JPCCAppEncoderTestN 300 65 175 &
JPCCAppEncoderTestN 400 68 175 &
JPCCAppEncoderTestN 500 71 175 &

JPCCAppEncoderTestN 100 53 350 &
JPCCAppEncoderTestN 200 60 350 &
JPCCAppEncoderTestN 300 65 350 &
JPCCAppEncoderTestN 400 68 350 &
JPCCAppEncoderTestN 500 71 350 &

JPCCAppEncoderTestN 100 53 700 &
JPCCAppEncoderTestN 200 60 700 &
JPCCAppEncoderTestN 300 65 700 &
JPCCAppEncoderTestN 400 68 700 &
JPCCAppEncoderTestN 500 71 700 &

JPCCAppEncoderTestN 100 53 1400 &
JPCCAppEncoderTestN 200 60 1400 &
JPCCAppEncoderTestN 300 65 1400 &
JPCCAppEncoderTestN 400 68 1400 &
JPCCAppEncoderTestN 500 71 1400 &

JPCCAppEncoderTestN 100 53 2100 &
JPCCAppEncoderTestN 200 60 2100 &
JPCCAppEncoderTestN 300 65 2100 &
JPCCAppEncoderTestN 400 68 2100 &
JPCCAppEncoderTestN 500 71 2100 &

JPCCAppEncoderTestN 100 53 2800 &
JPCCAppEncoderTestN 200 60 2800 &
JPCCAppEncoderTestN 300 65 2800 &
JPCCAppEncoderTestN 400 68 2800 &
JPCCAppEncoderTestN 500 71 2800 &
