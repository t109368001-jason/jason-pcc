#!/bin/bash
set -ex

mkdir -p ./logs

./bin/JPCCAppAnalyzer --configs ./cfg/app/Analyzer/ctc-raw.cfg |& tee ./logs/JPCCAppAnalyzer.log
