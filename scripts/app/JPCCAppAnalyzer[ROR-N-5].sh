#!/bin/bash
set -ex

OUTPUT_DIR=../../result/jason-pcc/analyzer[ROR-N-5]/

mkdir -p ${OUTPUT_DIR}

./bin/JPCCAppAnalyzer \
  --preProcess.radiusOutlierRemoval.minNeighborsInRadius 5 \
  --app.outputDir ${OUTPUT_DIR} \
  --configs ./cfg/app/Analyzer/ctc-raw.cfg |& tee ${OUTPUT_DIR}JPCCAppAnalyzer-$(date +%Y%m%d-%H%M%S).log
