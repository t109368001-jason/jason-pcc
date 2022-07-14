#!/bin/bash
set -ex

OUTPUT_DIR=../../result/jason-pcc/JPCCAppSegmentation/

mkdir -p ${OUTPUT_DIR}

./bin/JPCCAppSegmentation \
  --configs cfg/app/Segmentation/ctc-raw.cfg |& tee ${OUTPUT_DIR}JPCCAppSegmentation-$(date +%Y%m%d-%H%M%S).log
