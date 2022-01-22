#!/bin/bash
set -ex

source encode.sh

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/queen-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S22CWAI_queen/S22CWAI_queen.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
  --frameCount=250 \
  --resolution=1023 |& tee S22CWAI_queen/S22CWAI_queen_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=0000 \
  --compressedStreamPath=S22CWAI_queen/S22CWAI_queen.bin |& tee S22CWAI_queen/S22CWAI_queen_decoder.log
