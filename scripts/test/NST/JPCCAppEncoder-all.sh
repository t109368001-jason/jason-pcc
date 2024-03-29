#!/bin/bash
set -ex

OUTPUT_FOLDER_PREFIX=${OUTPUT_FOLDER_PREFIX:="../../result/test-NST-$(date +%Y%m%d-%H)/"}

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/JPCCAppEncoderTestNST.sh"

# 2 ~ 1 % non-null
JPCCAppEncoderTestNST 100 53 -0.038298432 -0.039096316 &
JPCCAppEncoderTestNST 200 60 -0.038298432 -0.039096316 &
JPCCAppEncoderTestNST 300 65 -0.038298432 -0.039096316 &
JPCCAppEncoderTestNST 400 68 -0.038298432 -0.039096316 &
JPCCAppEncoderTestNST 500 71 -0.038298432 -0.039096316 &

# 3 ~ 2 % non-null
JPCCAppEncoderTestNST 100 53 -0.037500548 -0.038298432 &
JPCCAppEncoderTestNST 200 60 -0.037500548 -0.038298432 &
JPCCAppEncoderTestNST 300 65 -0.037500548 -0.038298432 &
JPCCAppEncoderTestNST 400 68 -0.037500548 -0.038298432 &
JPCCAppEncoderTestNST 500 71 -0.037500548 -0.038298432 &

# 4 ~ 3 % non-null
JPCCAppEncoderTestNST 100 53 -0.036702664 -0.037500548 &
JPCCAppEncoderTestNST 200 60 -0.036702664 -0.037500548 &
JPCCAppEncoderTestNST 300 65 -0.036702664 -0.037500548 &
JPCCAppEncoderTestNST 400 68 -0.036702664 -0.037500548 &
JPCCAppEncoderTestNST 500 71 -0.036702664 -0.037500548 &

# 6 ~ 4 % non-null
JPCCAppEncoderTestNST 100 53 -0.035106896 -0.036702664 &
JPCCAppEncoderTestNST 200 60 -0.035106896 -0.036702664 &
JPCCAppEncoderTestNST 300 65 -0.035106896 -0.036702664 &
JPCCAppEncoderTestNST 400 68 -0.035106896 -0.036702664 &
JPCCAppEncoderTestNST 500 71 -0.035106896 -0.036702664 &

# 11 ~ 9 % non-null
JPCCAppEncoderTestNST 100 53 -0.031117476 -0.032713244 &
JPCCAppEncoderTestNST 200 60 -0.031117476 -0.032713244 &
JPCCAppEncoderTestNST 300 65 -0.031117476 -0.032713244 &
JPCCAppEncoderTestNST 400 68 -0.031117476 -0.032713244 &
JPCCAppEncoderTestNST 500 71 -0.031117476 -0.032713244 &

# 21 ~ 19 % non-null
JPCCAppEncoderTestNST 100 53 -0.023138636 -0.024734404 &
JPCCAppEncoderTestNST 200 60 -0.023138636 -0.024734404 &
JPCCAppEncoderTestNST 300 65 -0.023138636 -0.024734404 &
JPCCAppEncoderTestNST 400 68 -0.023138636 -0.024734404 &
JPCCAppEncoderTestNST 500 71 -0.023138636 -0.024734404 &
