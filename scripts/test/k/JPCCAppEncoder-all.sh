#!/bin/bash
set -ex

OUTPUT_FOLDER_PREFIX = ${OUTPUT_FOLDER_PREFIX:="../../result/test-k-$(date +%Y%m%d-%H)/"}

source ./JPCCAppEncoderTestK.sh

JPCCAppEncoderTestK 100 53 2 &
JPCCAppEncoderTestK 200 60 2 &
JPCCAppEncoderTestK 300 65 2 &
JPCCAppEncoderTestK 400 68 2 &
JPCCAppEncoderTestK 500 71 2 &

JPCCAppEncoderTestK 100 53 3 &
JPCCAppEncoderTestK 200 60 3 &
JPCCAppEncoderTestK 300 65 3 &
JPCCAppEncoderTestK 400 68 3 &
JPCCAppEncoderTestK 500 71 3 &

JPCCAppEncoderTestK 100 53 4 &
JPCCAppEncoderTestK 200 60 4 &
JPCCAppEncoderTestK 300 65 4 &
JPCCAppEncoderTestK 400 68 4 &
JPCCAppEncoderTestK 500 71 4 &
