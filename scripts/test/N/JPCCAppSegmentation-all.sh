#!/bin/bash
set -ex

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/JPCCAppSegmentationTestN.sh"

JPCCAppSegmentationTestN 100 175 &
JPCCAppSegmentationTestN 200 175 &
JPCCAppSegmentationTestN 300 175 &
JPCCAppSegmentationTestN 400 175 &
JPCCAppSegmentationTestN 500 175 &

JPCCAppSegmentationTestN 100 350 &
JPCCAppSegmentationTestN 200 350 &
JPCCAppSegmentationTestN 300 350 &
JPCCAppSegmentationTestN 400 350 &
JPCCAppSegmentationTestN 500 350 &

JPCCAppSegmentationTestN 100 700 &
JPCCAppSegmentationTestN 200 700 &
JPCCAppSegmentationTestN 300 700 &
JPCCAppSegmentationTestN 400 700 &
JPCCAppSegmentationTestN 500 700 &

JPCCAppSegmentationTestN 100 1400 &
JPCCAppSegmentationTestN 200 1400 &
JPCCAppSegmentationTestN 300 1400 &
JPCCAppSegmentationTestN 400 1400 &
JPCCAppSegmentationTestN 500 1400 &

JPCCAppSegmentationTestN 100 2100 &
JPCCAppSegmentationTestN 200 2100 &
JPCCAppSegmentationTestN 300 2100 &
JPCCAppSegmentationTestN 400 2100 &
JPCCAppSegmentationTestN 500 2100 &

JPCCAppSegmentationTestN 100 2800 &
JPCCAppSegmentationTestN 200 2800 &
JPCCAppSegmentationTestN 300 2800 &
JPCCAppSegmentationTestN 400 2800 &
JPCCAppSegmentationTestN 500 2800 &
