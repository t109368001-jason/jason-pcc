#!/bin/bash
set -ex

source encode.sh

#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
#  --config=./cfg/condition/ctc-low-delay-lossless-geometry-attribute.cfg \
#  --config=./cfg/sequence/queen-lossless.cfg \
#  --config=./cfg/rate/ctc-r5-lossless.cfg \
#  --compressedStreamPath=S22CWLD_queen/S22CWLD_queen.bin \
#  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
#  --frameCount=250 \
#  --resolution=1023 |& tee S22CWLD_queen/S22CWLD_queen_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0000 \
#  --compressedStreamPath=S22CWLD_queen/S22CWLD_queen.bin |& tee S22CWLD_queen/S22CWLD_queen_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-low-delay-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/loot_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S23CWLD_loot/S23CWLD_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S23CWLD_loot/S23CWLD_loot_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1000 \
  --compressedStreamPath=S23CWLD_loot/S23CWLD_loot.bin |& tee S23CWLD_loot/S23CWLD_loot_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-low-delay-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/redandblack_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S24CWLD_redandblack/S24CWLD_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S24CWLD_redandblack/S24CWLD_redandblack_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1450 \
  --compressedStreamPath=S24CWLD_redandblack/S24CWLD_redandblack.bin |& tee S24CWLD_redandblack/S24CWLD_redandblack_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-low-delay-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/soldier_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S25CWLD_soldier/S25CWLD_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S25CWLD_soldier/S25CWLD_soldier_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=0536 \
  --compressedStreamPath=S25CWLD_soldier/S25CWLD_soldier.bin |& tee S25CWLD_soldier/S25CWLD_soldier_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-low-delay-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/longdress_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S26CWLD_longdress/S26CWLD_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S26CWLD_longdress/S26CWLD_longdress_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1051 \
  --compressedStreamPath=S26CWLD_longdress/S26CWLD_longdress.bin |& tee S26CWLD_longdress/S26CWLD_longdress_decoder.log

#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
#  --config=./cfg/condition/ctc-low-delay-lossless-geometry-attribute.cfg \
#  --config=./cfg/sequence/basketball_player_vox11-lossless.cfg \
#  --config=./cfg/rate/ctc-r5-lossless.cfg \
#  --compressedStreamPath=S27CWLD_basketball/S27CWLD_basketball.bin \
#  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
#  --frameCount=64 \
#  --resolution=2047 |& tee S27CWLD_basketball/S27CWLD_basketball_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0001 \
#  --compressedStreamPath=S27CWLD_basketball/S27CWLD_basketball.bin |& tee S27CWLD_basketball/S27CWLD_basketball_decoder.log

#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
#  --config=./cfg/condition/ctc-low-delay-lossless-geometry-attribute.cfg \
#  --config=./cfg/sequence/dancer_vox11-lossless.cfg \
#  --config=./cfg/rate/ctc-r5-lossless.cfg \
#  --compressedStreamPath=S28CWLD_dancer/S28CWLD_dancer.bin \
#  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
#  --frameCount=64 \
#  --resolution=2047 |& tee S28CWLD_dancer/S28CWLD_dancer_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0001 \
#  --compressedStreamPath=S28CWLD_dancer/S28CWLD_dancer.bin |& tee S28CWLD_dancer/S28CWLD_dancer_decoder.log
