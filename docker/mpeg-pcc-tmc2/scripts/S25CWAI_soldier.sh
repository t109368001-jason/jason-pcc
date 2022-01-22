#!/bin/bash
set -ex

source encode.sh

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/soldier_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S25CWAI_soldier/S25CWAI_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S25CWAI_soldier/S25CWAI_soldier_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=0536 \
  --compressedStreamPath=S25CWAI_soldier/S25CWAI_soldier.bin |& tee S25CWAI_soldier/S25CWAI_soldier_decoder.log
