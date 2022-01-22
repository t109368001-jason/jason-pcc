#!/bin/bash
set -ex

source encode.sh

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/redandblack_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S24CWAI_redandblack/S24CWAI_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S24CWAI_redandblack/S24CWAI_redandblack_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1450 \
  --compressedStreamPath=S24CWAI_redandblack/S24CWAI_redandblack.bin |& tee S24CWAI_redandblack/S24CWAI_redandblack_decoder.log
