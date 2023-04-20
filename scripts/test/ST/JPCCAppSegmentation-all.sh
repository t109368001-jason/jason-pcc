#!/bin/bash
set -ex

source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)/JPCCAppSegmentationTestST.sh"

# 0.4 ~ 0.6 SD
JPCCAppSegmentationTestST 100 0.003682 0.003332 &
JPCCAppSegmentationTestST 200 0.003682 0.003332 &
JPCCAppSegmentationTestST 300 0.003682 0.003332 &
JPCCAppSegmentationTestST 400 0.003682 0.003332 &
JPCCAppSegmentationTestST 500 0.003682 0.003332 &

# 0.7 ~ 0.8 SD
JPCCAppSegmentationTestST 100 0.003123 0.002897 &
JPCCAppSegmentationTestST 200 0.003123 0.002897 &
JPCCAppSegmentationTestST 300 0.003123 0.002897 &
JPCCAppSegmentationTestST 400 0.003123 0.002897 &
JPCCAppSegmentationTestST 500 0.003123 0.002897 &

# 0.9 ~ 1.1 SD
JPCCAppSegmentationTestST 100 0.002661 0.002179 &
JPCCAppSegmentationTestST 200 0.002661 0.002179 &
JPCCAppSegmentationTestST 300 0.002661 0.002179 &
JPCCAppSegmentationTestST 400 0.002661 0.002179 &
JPCCAppSegmentationTestST 500 0.002661 0.002179 &

# 1.4 ~ 1.5 SD
JPCCAppSegmentationTestST 100 0.001497 0.001109 &
JPCCAppSegmentationTestST 200 0.001497 0.001109 &
JPCCAppSegmentationTestST 300 0.001497 0.001109 &
JPCCAppSegmentationTestST 400 0.001497 0.001109 &
JPCCAppSegmentationTestST 500 0.001497 0.001109 &

# 1.9 ~ 2.1 SD
JPCCAppSegmentationTestST 100 0.0006562 0.0004398 &
JPCCAppSegmentationTestST 200 0.0006562 0.0004398 &
JPCCAppSegmentationTestST 300 0.0006562 0.0004398 &
JPCCAppSegmentationTestST 400 0.0006562 0.0004398 &
JPCCAppSegmentationTestST 500 0.0006562 0.0004398 &

# 2.9 ~ 3.1 SD
JPCCAppSegmentationTestST 100 0.00005925 0.00003266 &
JPCCAppSegmentationTestST 200 0.00005925 0.00003266 &
JPCCAppSegmentationTestST 300 0.00005925 0.00003266 &
JPCCAppSegmentationTestST 400 0.00005925 0.00003266 &
JPCCAppSegmentationTestST 500 0.00005925 0.00003266 &
