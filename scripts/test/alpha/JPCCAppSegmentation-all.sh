#!/bin/bash
set -ex

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/JPCCAppSegmentationTestAlpha.sh"

JPCCAppSegmentationTestAlpha 100 0.00099 &
JPCCAppSegmentationTestAlpha 200 0.00099 &
JPCCAppSegmentationTestAlpha 300 0.00099 &
JPCCAppSegmentationTestAlpha 400 0.00099 &
JPCCAppSegmentationTestAlpha 500 0.00099 &

JPCCAppSegmentationTestAlpha 100 0.00198 &
JPCCAppSegmentationTestAlpha 200 0.00198 &
JPCCAppSegmentationTestAlpha 300 0.00198 &
JPCCAppSegmentationTestAlpha 400 0.00198 &
JPCCAppSegmentationTestAlpha 500 0.00198 &

JPCCAppSegmentationTestAlpha 100 0.00328 &
JPCCAppSegmentationTestAlpha 200 0.00328 &
JPCCAppSegmentationTestAlpha 300 0.00328 &
JPCCAppSegmentationTestAlpha 400 0.00328 &
JPCCAppSegmentationTestAlpha 500 0.00328 &

JPCCAppSegmentationTestAlpha 100 0.00395 &
JPCCAppSegmentationTestAlpha 200 0.00395 &
JPCCAppSegmentationTestAlpha 300 0.00395 &
JPCCAppSegmentationTestAlpha 400 0.00395 &
JPCCAppSegmentationTestAlpha 500 0.00395 &

JPCCAppSegmentationTestAlpha 100 0.00656 &
JPCCAppSegmentationTestAlpha 200 0.00656 &
JPCCAppSegmentationTestAlpha 300 0.00656 &
JPCCAppSegmentationTestAlpha 400 0.00656 &
JPCCAppSegmentationTestAlpha 500 0.00656 &

JPCCAppSegmentationTestAlpha 100 0.01310 &
JPCCAppSegmentationTestAlpha 200 0.01310 &
JPCCAppSegmentationTestAlpha 300 0.01310 &
JPCCAppSegmentationTestAlpha 400 0.01310 &
JPCCAppSegmentationTestAlpha 500 0.01310 &
