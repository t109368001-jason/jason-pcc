#!/bin/bash
set -ex

mkdir -p ./logs

OUTPUT_FOLDER_PREFIX="../../dataset/"
OUTPUT_FOLDER="converted-$(date +%Y%m%d-%H)/ZX-XS-20220707-ply-xyzi/"

mkdir -p "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}"

./bin/JPCCAppDatasetParser \
  --app.outputPointType "PointXYZI" \
  --outputDataset.folder "${OUTPUT_FOLDER}" \
  --configs ./cfg/app/DatasetParser/ZX-XS-20220707-ply-xyzi.cfg |& tee "${OUTPUT_FOLDER_PREFIX}${OUTPUT_FOLDER}JPCCAppDatasetParser-$(date +%Y%m%d-%H%M%S).log"
