#!/bin/bash
set -ex

mkdir -p ./logs

OUTPUT_DIR = ../../result/jason-pcc/analyzer[CR-R-100000][ROR-R-500][ROR-N-5]/

./bin/JPCCAppAnalyzer \
  --conditions "r < 100000" \
  --radius 500 \
  --minNeighborsInRadius 5 \
  --outputDir ${OUTPUT_DIR} \
  --configs ./cfg/app/Analyzer/ctc-raw.cfg |&
  tee ${OUTPUT_DIR}/JPCCAppAnalyzer.log
