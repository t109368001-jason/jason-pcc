#!/bin/bash
set -ex

source encode.sh

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/longdress_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S26CWAI_longdress/S26CWAI_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S26CWAI_longdress/S26CWAI_longdress_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1051 \
  --compressedStreamPath=S26CWAI_longdress/S26CWAI_longdress.bin |& tee S26CWAI_longdress/S26CWAI_longdress_decoder.log
