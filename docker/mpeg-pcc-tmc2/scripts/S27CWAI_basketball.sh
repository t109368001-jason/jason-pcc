#!/bin/bash
set -ex

source encode.sh

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/basketball_player_vox11-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S27CWAI_basketball/S27CWAI_basketball.bin \
  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 |& tee S27CWAI_basketball/S27CWAI_basketball_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=0001 \
  --compressedStreamPath=S27CWAI_basketball/S27CWAI_basketball.bin |& tee S27CWAI_basketball/S27CWAI_basketball_decoder.log
