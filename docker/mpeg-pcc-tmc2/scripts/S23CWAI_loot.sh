#!/bin/bash
set -ex

source encode.sh

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/loot_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S23CWAI_loot/S23CWAI_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S23CWAI_loot/S23CWAI_loot_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1000 \
  --compressedStreamPath=S23CWAI_loot/S23CWAI_loot.bin |& tee S23CWAI_loot/S23CWAI_loot_decoder.log
