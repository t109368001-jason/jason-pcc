#!/bin/bash
set -ex

#ENCODER="\
#  --configurationFolder=./cfg/ \
#  --colorSpaceConversionPath=./HDRTools/bin/HDRConvert\
#  --videoEncoderPath=./HM-16.20+SCM-8.8/bin/TAppEncoderHighBitDepthStatic \
#  --videoEncoderAuxPath=./HM-16.20+SCM-8.8/bin/TAppEncoderHighBitDepthStatic \
#  --videoEncoderOccupancyMapPath=./HM-16.20+SCM-8.8/bin/TAppEncoderHighBitDepthStatic \
#  --colorTransform=0 \
#  --nbThread=1 \
#  --uncompressedDataFolder=./mpeg_datasets/CfP/datasets/Dynamic_Objects/People/ "
ENCODER="\
  --configurationFolder=./cfg/ \
  --colorTransform=0 \
  --nbThread=1 \
  --uncompressedDataFolder=/dataset/ "

#DECODER="\
#  --videoDecoderPath=./HM-16.20+SCM-8.8/bin/TAppDecoderHighBitDepthStatic \
#  --colorSpaceConversionPath=./HDRTools/bin/HDRConvert \
#  --inverseColorSpaceConversionConfig=./cfg/hdrconvert/yuv420toyuv444_16bit.cfg \
#  --nbThread=1 \
#  --colorTransform=0 "
DECODER="\
  --nbThread=1 \
  --colorTransform=0 "

function encode() {
  ARG_STR="'$@'"
  RESULT_DIR="$(echo "${ARG_STR}" | grep -oP '(?<='--compressedStreamPath=')[^.]*')"
  SHA256SUM_FILE="${RESULT_DIR}.sha256sum-encode"
  if [ -f "${SHA256SUM_FILE}" ]; then
    OLD_SHA256SUM="$(cat "${SHA256SUM_FILE}")"
    NEW_SHA256SUM="$(echo -n "${ARG_STR}" | sha256sum)"
    if [ "${OLD_SHA256SUM}" == "${NEW_SHA256SUM}" ]; then
      return
    fi
  fi
  "$@" && echo -n "${ARG_STR}" | sha256sum >"${SHA256SUM_FILE}"
}

function decode() {
  ARG_STR="'$@'"
  RESULT_DIR="$(echo "${ARG_STR}" | grep -oP '(?<='--compressedStreamPath=')[^.]*')"
  SHA256SUM_FILE="${RESULT_DIR}.sha256sum-decode"
  if [ -f "${SHA256SUM_FILE}" ]; then
    OLD_SHA256SUM="$(cat "${SHA256SUM_FILE}")"
    NEW_SHA256SUM="$(echo -n "${ARG_STR}" | sha256sum)"
    if [ "${OLD_SHA256SUM}" == "${NEW_SHA256SUM}" ]; then
      return
    fi
  fi
  "$@" && echo -n "${ARG_STR}" | sha256sum >"${SHA256SUM_FILE}"
}
