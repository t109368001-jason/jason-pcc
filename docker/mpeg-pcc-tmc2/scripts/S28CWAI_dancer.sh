#!/bin/bash
set -ex

source encode.sh

#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
#  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
#  --config=./cfg/sequence/dancer_vox11-lossless.cfg \
#  --config=./cfg/rate/ctc-r5-lossless.cfg \
#  --compressedStreamPath=S28CWAI_dancer/S28CWAI_dancer.bin \
#  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
#  --frameCount=64 \
#  --resolution=2047 |& tee S28CWAI_dancer/S28CWAI_dancer_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0001 \
#  --compressedStreamPath=S28CWAI_dancer/S28CWAI_dancer.bin |& tee S28CWAI_dancer/S28CWAI_dancer_decoder.log
