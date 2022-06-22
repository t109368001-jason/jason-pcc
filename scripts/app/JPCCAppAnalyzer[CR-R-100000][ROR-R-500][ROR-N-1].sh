#!/bin/bash
set -ex

mkdir -p ./logs

OUTPUT_DIR=../../result/jason-pcc/analyzer[CR-R-100000][ROR-R-500][ROR-N-1]/

./bin/JPCCAppAnalyzer \
  --preProcess.jpccConditionalRemoval.conditions "r < 100000" \
  --preProcess.radiusOutlierRemoval.radius 500 \
  --preProcess.radiusOutlierRemoval.minNeighborsInRadius 1 \
  --outputDir ${OUTPUT_DIR} \
  --configs ./cfg/app/Analyzer/ctc-raw.cfg |&
  tee ${OUTPUT_DIR}/JPCCAppAnalyzer.log
