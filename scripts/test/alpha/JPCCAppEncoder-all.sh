#!/bin/bash
set -ex

OUTPUT_FOLDER_PREFIX=${OUTPUT_FOLDER_PREFIX:="../../result/test-alpha-$(date +%Y%m%d-%H)/"}

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/JPCCAppEncoderTestAlpha.sh"

JPCCAppEncoderTestAlpha 100 53 0.00099 &
JPCCAppEncoderTestAlpha 200 60 0.00099 &
JPCCAppEncoderTestAlpha 300 65 0.00099 &
JPCCAppEncoderTestAlpha 400 68 0.00099 &
JPCCAppEncoderTestAlpha 500 71 0.00099 &

JPCCAppEncoderTestAlpha 100 53 0.00198 &
JPCCAppEncoderTestAlpha 200 60 0.00198 &
JPCCAppEncoderTestAlpha 300 65 0.00198 &
JPCCAppEncoderTestAlpha 400 68 0.00198 &
JPCCAppEncoderTestAlpha 500 71 0.00198 &

JPCCAppEncoderTestAlpha 100 53 0.00328 &
JPCCAppEncoderTestAlpha 200 60 0.00328 &
JPCCAppEncoderTestAlpha 300 65 0.00328 &
JPCCAppEncoderTestAlpha 400 68 0.00328 &
JPCCAppEncoderTestAlpha 500 71 0.00328 &

JPCCAppEncoderTestAlpha 100 53 0.00395 &
JPCCAppEncoderTestAlpha 200 60 0.00395 &
JPCCAppEncoderTestAlpha 300 65 0.00395 &
JPCCAppEncoderTestAlpha 400 68 0.00395 &
JPCCAppEncoderTestAlpha 500 71 0.00395 &

JPCCAppEncoderTestAlpha 100 53 0.00656 &
JPCCAppEncoderTestAlpha 200 60 0.00656 &
JPCCAppEncoderTestAlpha 300 65 0.00656 &
JPCCAppEncoderTestAlpha 400 68 0.00656 &
JPCCAppEncoderTestAlpha 500 71 0.00656 &

JPCCAppEncoderTestAlpha 100 53 0.01310 &
JPCCAppEncoderTestAlpha 200 60 0.01310 &
JPCCAppEncoderTestAlpha 300 65 0.01310 &
JPCCAppEncoderTestAlpha 400 68 0.01310 &
JPCCAppEncoderTestAlpha 500 71 0.01310 &
