#!/bin/bash
set -ex

OUTPUT_DIR=../../result/jason-pcc/analyzer/

mkdir -p ${OUTPUT_DIR}

./bin/JPCCAppAnalyzer \
  --app.outputDir ${OUTPUT_DIR} \
  --configs ./cfg/app/Analyzer/ctc-raw.cfg |& tee ${OUTPUT_DIR}JPCCAppAnalyzer-$(date +%Y%m%d-%H%M%S).log
