#!/bin/bash
set -ex

mkdir -p ./logs

./bin/JPCCAppDatasetParser \
  --app.outputPointType "PointXYZ" \
  --outputDataset.folder "converted/ZX-XS-20220707-ctc-pre-processed-xyz/" \
  --configs ./cfg/app/DatasetParser/ZX-XS-20220330-to-ctc-pre-processed.cfg |& tee ./logs/ctc-raw-to-pre-processed.log
