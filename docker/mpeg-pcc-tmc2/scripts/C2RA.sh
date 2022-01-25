#!/bin/bash
set -ex

source encode.sh

#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common.cfg \
#  --config=./cfg/condition/ctc-random-access.cfg \
#  --config=./cfg/sequence/queen.cfg \
#  --config=./cfg/rate/ctc-r1.cfg \
#  --compressedStreamPath=S22C2RA_queen/S22C2RAR01_queen.bin \
#  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
#  --frameCount=250 \
#  --resolution=1023 |& tee S22C2RA_queen/S22C2RAR01_queen_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0000 \
#  --compressedStreamPath=S22C2RA_queen/S22C2RAR01_queen.bin |& tee S22C2RA_queen/S22C2RAR01_queen_decoder.log
#
#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common.cfg \
#  --config=./cfg/condition/ctc-random-access.cfg \
#  --config=./cfg/sequence/queen.cfg \
#  --config=./cfg/rate/ctc-r2.cfg \
#  --compressedStreamPath=S22C2RA_queen/S22C2RAR02_queen.bin \
#  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
#  --frameCount=250 \
#  --resolution=1023 |& tee S22C2RA_queen/S22C2RAR02_queen_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0000 \
#  --compressedStreamPath=S22C2RA_queen/S22C2RAR02_queen.bin |& tee S22C2RA_queen/S22C2RAR02_queen_decoder.log
#
#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common.cfg \
#  --config=./cfg/condition/ctc-random-access.cfg \
#  --config=./cfg/sequence/queen.cfg \
#  --config=./cfg/rate/ctc-r3.cfg \
#  --compressedStreamPath=S22C2RA_queen/S22C2RAR03_queen.bin \
#  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
#  --frameCount=250 \
#  --resolution=1023 |& tee S22C2RA_queen/S22C2RAR03_queen_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0000 \
#  --compressedStreamPath=S22C2RA_queen/S22C2RAR03_queen.bin |& tee S22C2RA_queen/S22C2RAR03_queen_decoder.log
#
#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common.cfg \
#  --config=./cfg/condition/ctc-random-access.cfg \
#  --config=./cfg/sequence/queen.cfg \
#  --config=./cfg/rate/ctc-r4.cfg \
#  --compressedStreamPath=S22C2RA_queen/S22C2RAR04_queen.bin \
#  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
#  --frameCount=250 \
#  --resolution=1023 |& tee S22C2RA_queen/S22C2RAR04_queen_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0000 \
#  --compressedStreamPath=S22C2RA_queen/S22C2RAR04_queen.bin |& tee S22C2RA_queen/S22C2RAR04_queen_decoder.log
#
#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common.cfg \
#  --config=./cfg/condition/ctc-random-access.cfg \
#  --config=./cfg/sequence/queen.cfg \
#  --config=./cfg/rate/ctc-r5.cfg \
#  --compressedStreamPath=S22C2RA_queen/S22C2RAR05_queen.bin \
#  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
#  --frameCount=250 \
#  --resolution=1023 |& tee S22C2RA_queen/S22C2RAR05_queen_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0000 \
#  --compressedStreamPath=S22C2RA_queen/S22C2RAR05_queen.bin |& tee S22C2RA_queen/S22C2RAR05_queen_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/loot_vox10.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR01_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S23C2RA_loot/S23C2RAR01_loot_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1000 \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR01_loot.bin |& tee S23C2RA_loot/S23C2RAR01_loot_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/loot_vox10.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR02_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S23C2RA_loot/S23C2RAR02_loot_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1000 \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR02_loot.bin |& tee S23C2RA_loot/S23C2RAR02_loot_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/loot_vox10.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR03_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S23C2RA_loot/S23C2RAR03_loot_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1000 \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR03_loot.bin |& tee S23C2RA_loot/S23C2RAR03_loot_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/loot_vox10.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR04_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S23C2RA_loot/S23C2RAR04_loot_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1000 \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR04_loot.bin |& tee S23C2RA_loot/S23C2RAR04_loot_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/loot_vox10.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR05_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S23C2RA_loot/S23C2RAR05_loot_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1000 \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR05_loot.bin |& tee S23C2RA_loot/S23C2RAR05_loot_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/redandblack_vox10.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR01_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S24C2RA_redandblack/S24C2RAR01_redandblack_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1450 \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR01_redandblack.bin |& tee S24C2RA_redandblack/S24C2RAR01_redandblack_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/redandblack_vox10.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR02_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S24C2RA_redandblack/S24C2RAR02_redandblack_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1450 \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR02_redandblack.bin |& tee S24C2RA_redandblack/S24C2RAR02_redandblack_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/redandblack_vox10.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR03_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S24C2RA_redandblack/S24C2RAR03_redandblack_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1450 \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR03_redandblack.bin |& tee S24C2RA_redandblack/S24C2RAR03_redandblack_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/redandblack_vox10.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR04_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S24C2RA_redandblack/S24C2RAR04_redandblack_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1450 \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR04_redandblack.bin |& tee S24C2RA_redandblack/S24C2RAR04_redandblack_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/redandblack_vox10.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR05_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S24C2RA_redandblack/S24C2RAR05_redandblack_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1450 \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR05_redandblack.bin |& tee S24C2RA_redandblack/S24C2RAR05_redandblack_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/soldier_vox10.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR01_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S25C2RA_soldier/S25C2RAR01_soldier_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=0536 \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR01_soldier.bin |& tee S25C2RA_soldier/S25C2RAR01_soldier_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/soldier_vox10.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR02_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S25C2RA_soldier/S25C2RAR02_soldier_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=0536 \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR02_soldier.bin |& tee S25C2RA_soldier/S25C2RAR02_soldier_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/soldier_vox10.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR03_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S25C2RA_soldier/S25C2RAR03_soldier_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=0536 \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR03_soldier.bin |& tee S25C2RA_soldier/S25C2RAR03_soldier_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/soldier_vox10.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR04_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S25C2RA_soldier/S25C2RAR04_soldier_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=0536 \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR04_soldier.bin |& tee S25C2RA_soldier/S25C2RAR04_soldier_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/soldier_vox10.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR05_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S25C2RA_soldier/S25C2RAR05_soldier_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=0536 \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR05_soldier.bin |& tee S25C2RA_soldier/S25C2RAR05_soldier_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/longdress_vox10.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR01_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S26C2RA_longdress/S26C2RAR01_longdress_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1051 \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR01_longdress.bin |& tee S26C2RA_longdress/S26C2RAR01_longdress_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/longdress_vox10.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR02_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S26C2RA_longdress/S26C2RAR02_longdress_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1051 \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR02_longdress.bin |& tee S26C2RA_longdress/S26C2RAR02_longdress_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/longdress_vox10.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR03_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S26C2RA_longdress/S26C2RAR03_longdress_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1051 \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR03_longdress.bin |& tee S26C2RA_longdress/S26C2RAR03_longdress_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/longdress_vox10.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR04_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S26C2RA_longdress/S26C2RAR04_longdress_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1051 \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR04_longdress.bin |& tee S26C2RA_longdress/S26C2RAR04_longdress_decoder.log

encode ./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/longdress_vox10.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR05_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 |& tee S26C2RA_longdress/S26C2RAR05_longdress_encoder.log

decode ./bin/PccAppDecoder \
  $DECODER --startFrameNumber=1051 \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR05_longdress.bin |& tee S26C2RA_longdress/S26C2RAR05_longdress_decoder.log

#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common.cfg \
#  --config=./cfg/condition/ctc-random-access.cfg \
#  --config=./cfg/sequence/basketball_player_vox11.cfg \
#  --config=./cfg/rate/ctc-r1.cfg \
#  --compressedStreamPath=S27C2RA_basketball/S27C2RAR01_basketball.bin \
#  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
#  --frameCount=64 \
#  --resolution=2047 |& tee S27C2RA_basketball/S27C2RAR01_basketball_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0001 \
#  --compressedStreamPath=S27C2RA_basketball/S27C2RAR01_basketball.bin |& tee S27C2RA_basketball/S27C2RAR01_basketball_decoder.log
#
#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common.cfg \
#  --config=./cfg/condition/ctc-random-access.cfg \
#  --config=./cfg/sequence/basketball_player_vox11.cfg \
#  --config=./cfg/rate/ctc-r2.cfg \
#  --compressedStreamPath=S27C2RA_basketball/S27C2RAR02_basketball.bin \
#  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
#  --frameCount=64 \
#  --resolution=2047 |& tee S27C2RA_basketball/S27C2RAR02_basketball_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0001 \
#  --compressedStreamPath=S27C2RA_basketball/S27C2RAR02_basketball.bin |& tee S27C2RA_basketball/S27C2RAR02_basketball_decoder.log
#
#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common.cfg \
#  --config=./cfg/condition/ctc-random-access.cfg \
#  --config=./cfg/sequence/basketball_player_vox11.cfg \
#  --config=./cfg/rate/ctc-r3.cfg \
#  --compressedStreamPath=S27C2RA_basketball/S27C2RAR03_basketball.bin \
#  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
#  --frameCount=64 \
#  --resolution=2047 |& tee S27C2RA_basketball/S27C2RAR03_basketball_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0001 \
#  --compressedStreamPath=S27C2RA_basketball/S27C2RAR03_basketball.bin |& tee S27C2RA_basketball/S27C2RAR03_basketball_decoder.log
#
#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common.cfg \
#  --config=./cfg/condition/ctc-random-access.cfg \
#  --config=./cfg/sequence/basketball_player_vox11.cfg \
#  --config=./cfg/rate/ctc-r4.cfg \
#  --compressedStreamPath=S27C2RA_basketball/S27C2RAR04_basketball.bin \
#  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
#  --frameCount=64 \
#  --resolution=2047 |& tee S27C2RA_basketball/S27C2RAR04_basketball_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0001 \
#  --compressedStreamPath=S27C2RA_basketball/S27C2RAR04_basketball.bin |& tee S27C2RA_basketball/S27C2RAR04_basketball_decoder.log
#
#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common.cfg \
#  --config=./cfg/condition/ctc-random-access.cfg \
#  --config=./cfg/sequence/basketball_player_vox11.cfg \
#  --config=./cfg/rate/ctc-r5.cfg \
#  --compressedStreamPath=S27C2RA_basketball/S27C2RAR05_basketball.bin \
#  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
#  --frameCount=64 \
#  --resolution=2047 |& tee S27C2RA_basketball/S27C2RAR05_basketball_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0001 \
#  --compressedStreamPath=S27C2RA_basketball/S27C2RAR05_basketball.bin |& tee S27C2RA_basketball/S27C2RAR05_basketball_decoder.log

#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common.cfg \
#  --config=./cfg/condition/ctc-random-access.cfg \
#  --config=./cfg/sequence/dancer_vox11.cfg \
#  --config=./cfg/rate/ctc-r1.cfg \
#  --compressedStreamPath=S28C2RA_dancer/S28C2RAR01_dancer.bin \
#  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
#  --frameCount=64 \
#  --resolution=2047 |& tee S28C2RA_dancer/S28C2RAR01_dancer_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0001 \
#  --compressedStreamPath=S28C2RA_dancer/S28C2RAR01_dancer.bin |& tee S28C2RA_dancer/S28C2RAR01_dancer_decoder.log
#
#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common.cfg \
#  --config=./cfg/condition/ctc-random-access.cfg \
#  --config=./cfg/sequence/dancer_vox11.cfg \
#  --config=./cfg/rate/ctc-r2.cfg \
#  --compressedStreamPath=S28C2RA_dancer/S28C2RAR02_dancer.bin \
#  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
#  --frameCount=64 \
#  --resolution=2047 |& tee S28C2RA_dancer/S28C2RAR02_dancer_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0001 \
#  --compressedStreamPath=S28C2RA_dancer/S28C2RAR02_dancer.bin |& tee S28C2RA_dancer/S28C2RAR02_dancer_decoder.log
#
#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common.cfg \
#  --config=./cfg/condition/ctc-random-access.cfg \
#  --config=./cfg/sequence/dancer_vox11.cfg \
#  --config=./cfg/rate/ctc-r3.cfg \
#  --compressedStreamPath=S28C2RA_dancer/S28C2RAR03_dancer.bin \
#  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
#  --frameCount=64 \
#  --resolution=2047 |& tee S28C2RA_dancer/S28C2RAR03_dancer_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0001 \
#  --compressedStreamPath=S28C2RA_dancer/S28C2RAR03_dancer.bin |& tee S28C2RA_dancer/S28C2RAR03_dancer_decoder.log
#
#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common.cfg \
#  --config=./cfg/condition/ctc-random-access.cfg \
#  --config=./cfg/sequence/dancer_vox11.cfg \
#  --config=./cfg/rate/ctc-r4.cfg \
#  --compressedStreamPath=S28C2RA_dancer/S28C2RAR04_dancer.bin \
#  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
#  --frameCount=64 \
#  --resolution=2047 |& tee S28C2RA_dancer/S28C2RAR04_dancer_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0001 \
#  --compressedStreamPath=S28C2RA_dancer/S28C2RAR04_dancer.bin |& tee S28C2RA_dancer/S28C2RAR04_dancer_decoder.log
#
#encode ./bin/PccAppEncoder \
#  $ENCODER \
#  --config=./cfg/common/ctc-common.cfg \
#  --config=./cfg/condition/ctc-random-access.cfg \
#  --config=./cfg/sequence/dancer_vox11.cfg \
#  --config=./cfg/rate/ctc-r5.cfg \
#  --compressedStreamPath=S28C2RA_dancer/S28C2RAR05_dancer.bin \
#  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
#  --frameCount=64 \
#  --resolution=2047 |& tee S28C2RA_dancer/S28C2RAR05_dancer_encoder.log
#
#decode ./bin/PccAppDecoder \
#  $DECODER --startFrameNumber=0001 \
#  --compressedStreamPath=S28C2RA_dancer/S28C2RAR05_dancer.bin |& tee S28C2RA_dancer/S28C2RAR05_dancer_decoder.log
