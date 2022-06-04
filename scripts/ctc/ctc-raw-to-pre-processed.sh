#!/bin/bash
set -ex

mkdir -p ./logs

./bin/JPCCAppDatasetParser --configs ./cfg/app/DatasetParser/ZX-XS-20220330-to-ctc-pre-processed.cfg |& tee ./logs/ctc-raw-to-pre-processed.log
