#!/bin/bash
set -ex

mkdir -p ./logs

./bin/JPCCAppDatasetParser --configs ./cfg/app/DatasetParser/ZX-XS-20220330-to-ply.cfg |& tee ./logs/JPCCAppDatasetParser.log
