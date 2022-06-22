#!/bin/bash
set -ex

mkdir -p ./logs

OUTPUT_DIR=../../result/jason-pcc/analyzer[CR-R-100000][CR-Z-5000][ROR-R-500][ROR-N-5]/

./bin/JPCCAppAnalyzer \
  --preProcess.jpccConditionalRemoval.conditions "r < 100000" \
  --preProcess.jpccConditionalRemoval.conditions "z < 5000" \
  --preProcess.radiusOutlierRemoval.radius 500 \
  --preProcess.radiusOutlierRemoval.minNeighborsInRadius 5 \
  --app.outputDir ${OUTPUT_DIR} \
  --configs ./cfg/app/Analyzer/ctc-raw.cfg |& tee ${OUTPUT_DIR}/JPCCAppAnalyzer.log
