#!/bin/bash
set -ex

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/JPCCAppSegmentationTestK.sh"

JPCCAppSegmentationTestK 100 2 &
JPCCAppSegmentationTestK 200 2 &
JPCCAppSegmentationTestK 300 2 &
JPCCAppSegmentationTestK 400 2 &
JPCCAppSegmentationTestK 500 2 &

JPCCAppSegmentationTestK 100 3 &
JPCCAppSegmentationTestK 200 3 &
JPCCAppSegmentationTestK 300 3 &
JPCCAppSegmentationTestK 400 3 &
JPCCAppSegmentationTestK 500 3 &

JPCCAppSegmentationTestK 100 4 &
JPCCAppSegmentationTestK 200 4 &
JPCCAppSegmentationTestK 300 4 &
JPCCAppSegmentationTestK 400 4 &
JPCCAppSegmentationTestK 500 4 &

JPCCAppSegmentationTestK 100 5 &
JPCCAppSegmentationTestK 200 5 &
JPCCAppSegmentationTestK 300 5 &
JPCCAppSegmentationTestK 400 5 &
JPCCAppSegmentationTestK 500 5 &
