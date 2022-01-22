#!/bin/bash
set -ex

source encode.sh

#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
#  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
#  --config=./cfg/sequence/queen-lossless.cfg \
#  --config=./cfg/rate/ctc-r5-lossless.cfg \
#  --compressedStreamPath=S22CWAI_queen/S22CWAI_queen.bin \
#  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
#  --frameCount=250 \
#  --resolution=1023 |& tee S22CWAI_queen/S22CWAI_queen_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0000 \
#  --compressedStreamPath=S22CWAI_queen/S22CWAI_queen.bin |& tee S22CWAI_queen/S22CWAI_queen_decoder.log

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

#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
#  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
#  --config=./cfg/sequence/basketball_player_vox11-lossless.cfg \
#  --config=./cfg/rate/ctc-r5-lossless.cfg \
#  --compressedStreamPath=S27CWAI_basketball/S27CWAI_basketball.bin \
#  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
#  --frameCount=64 \
#  --resolution=2047 |& tee S27CWAI_basketball/S27CWAI_basketball_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0001 \
#  --compressedStreamPath=S27CWAI_basketball/S27CWAI_basketball.bin |& tee S27CWAI_basketball/S27CWAI_basketball_decoder.log

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
