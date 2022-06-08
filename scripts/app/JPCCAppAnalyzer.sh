#!/bin/bash
set -ex

mkdir -p ./logs

./bin/JPCCAppAnalyzer --configs ./cfg/app/Analyzer/ctc-pre-processed.cfg |& tee ./logs/JPCCAppAnalyzer.log
