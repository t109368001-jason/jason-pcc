#include <jpcc/encoder/JPCCEncoderTMC2Parameter.h>

namespace jpcc::encoder {

using namespace std;
using namespace po;

//////////////////////////////////////////////////////////////////////////////////////////////
PCCEncoderTMC2Parameters::PCCEncoderTMC2Parameters() {
  gridBasedSegmentation_                   = false;
  voxelDimensionGridBasedSegmentation_     = 2;
  uncompressedDataPath_                    = {};
  compressedStreamPath_                    = {};
  reconstructedDataPath_                   = {};
  configurationFolder_                     = {};
  uncompressedDataFolder_                  = {};
  startFrameNumber_                        = 0;
  frameCount_                              = 300;
  groupOfFramesSize_                       = 32;
  colorTransform_                          = COLOR_TRANSFORM_NONE;
  colorSpaceConversionPath_                = {};
  colorSpaceConversionConfig_              = {};
  inverseColorSpaceConversionConfig_       = {};
  nnNormalEstimation_                      = 16;
  normalOrientation_                       = 1;
  forcedSsvhUnitSizePrecisionBytes_        = 0;
  gridBasedRefineSegmentation_             = true;
  maxNNCountRefineSegmentation_            = gridBasedRefineSegmentation_ ? (gridBasedSegmentation_ ? 384 : 1024) : 256;
  iterationCountRefineSegmentation_        = gridBasedRefineSegmentation_ ? (gridBasedSegmentation_ ? 5 : 10) : 100;
  voxelDimensionRefineSegmentation_        = gridBasedSegmentation_ ? 2 : 4;
  searchRadiusRefineSegmentation_          = gridBasedSegmentation_ ? 128 : 192;
  occupancyResolution_                     = 16;
  enablePatchSplitting_                    = true;
  maxPatchSize_                            = 1024;
  log2QuantizerSizeX_                      = 4;
  log2QuantizerSizeY_                      = 4;
  minPointCountPerCCPatchSegmentation_     = 16;
  maxNNCountPatchSegmentation_             = 16;
  surfaceThickness_                        = 4;
  minLevel_                                = 64;  // fix value
  maxAllowedDist2RawPointsDetection_       = 9.0;
  maxAllowedDist2RawPointsSelection_       = 1.0;
  lambdaRefineSegmentation_                = 3.0;
  minimumImageWidth_                       = 1280;
  minimumImageHeight_                      = 1280;
  maxCandidateCount_                       = 4;
  occupancyPrecision_                      = 4;
  occupancyMapConfig_                      = {};
  occupancyMapQP_                          = 8;
  occupancyMapRefinement_                  = false;
  decodedAtlasInformationHash_             = 0;
  flagGeometrySmoothing_                   = true;
  patchExpansion_                          = false;
  gridSmoothing_                           = true;
  gridSize_                                = 8;
  neighborCountSmoothing_                  = 4 * 16;
  radius2Smoothing_                        = 4.0 * 16;
  radius2BoundaryDetection_                = 4.0 * 16;
  thresholdSmoothing_                      = 64.0;
  bestColorSearchRange_                    = 0;
  numNeighborsColorTransferFwd_            = 1;
  numNeighborsColorTransferBwd_            = 1;
  useDistWeightedAverageFwd_               = true;
  useDistWeightedAverageBwd_               = true;
  skipAvgIfIdenticalSourcePointPresentFwd_ = false;
  skipAvgIfIdenticalSourcePointPresentBwd_ = false;
  distOffsetFwd_                           = 0.0001;
  distOffsetBwd_                           = 0.0001;
  maxGeometryDist2Fwd_                     = 10000.0;
  maxGeometryDist2Bwd_                     = 10000.0;
  maxColorDist2Fwd_                        = 10000.0;
  maxColorDist2Bwd_                        = 10000.0;
  excludeColorOutlier_                     = false;
  thresholdColorOutlierDist_               = 10.0;
  videoEncoderOccupancyPath_               = {};
  videoEncoderGeometryPath_                = {};
  videoEncoderAttributePath_               = {};
  videoEncoderOccupancyCodecId_            = HMLIB;
  videoEncoderGeometryCodecId_             = HMLIB;
  videoEncoderAttributeCodecId_            = HMLIB;
  byteStreamVideoCoderOccupancy_           = true;
  byteStreamVideoCoderGeometry_            = true;
  byteStreamVideoCoderAttribute_           = true;
  geometryQP_                              = 28;
  attributeQP_                             = 43;
  auxGeometryQP_                           = 0;
  auxAttributeQP_                          = 0;
  geometryConfig_                          = {};
  geometry0Config_                         = {};
  geometry1Config_                         = {};
  attributeConfig_                         = {};
  attribute0Config_                        = {};
  attribute1Config_                        = {};
  rawPointsPatch_                          = false;
  noAttributes_                            = false;
  attributeVideo444_                       = false;
  useRawPointsSeparateVideo_               = false;
  geometryAuxVideoConfig_                  = {};
  attributeAuxVideoConfig_                 = {};
  nbThread_                                = 1;
  keepIntermediateFiles_                   = false;
  absoluteD1_                              = false;
  absoluteT1_                              = false;
  multipleStreams_                         = false;
  deltaQPD0_                               = 0;
  deltaQPD1_                               = 2;
  deltaQPT0_                               = 0;
  deltaQPT1_                               = 2;
  constrainedPack_                         = true;
  thresholdColorSmoothing_                 = 10.0;
  thresholdColorDifference_                = 10.0;
  thresholdColorVariation_                 = 6.0;
  flagColorSmoothing_                      = false;
  cgridSize_                               = 4;
  thresholdColorPreSmoothing_              = 10.0;
  thresholdColorPreSmoothingLocalEntropy_  = 4.5;
  radius2ColorPreSmoothing_                = 4.0 * 16;
  neighborCountColorPreSmoothing_          = 4 * 16;
  flagColorPreSmoothing_                   = true;
  groupDilation_                           = true;
  enhancedOccupancyMapCode_                = false;
  EOMFixBitCount_                          = 2;
  offsetLossyOM_                           = 0;
  thresholdLossyOM_                        = 0;
  prefilterLossyOM_                        = false;
  patchColorSubsampling_                   = false;
  mapCountMinus1_                          = 1;

  // reconstruction
  removeDuplicatePoints_      = true;
  pointLocalReconstruction_   = false;
  plrlNumberOfModes_          = 6;
  patchSize_                  = 9;
  singleMapPixelInterleaving_ = false;
  surfaceSeparation_          = false;

  // level of detail
  levelOfDetailX_ = 1;
  levelOfDetailY_ = 1;

  // Flexible Patch Packing
  packingStrategy_      = 1;
  attributeBGFill_      = 1;
  safeGuardDistance_    = 0;
  useEightOrientations_ = false;
  lowDelayEncoding_     = false;
  geometryPadding_      = 0U;

  // lossy raw points patch
  lossyRawPointsPatch_             = false;
  minNormSumOfInvDist4MPSelection_ = 0.35;

  // GPA
  globalPatchAllocation_ = 0;
  // GTP
  globalPackingStrategyGOF_         = 0;
  globalPackingStrategyReset_       = false;
  globalPackingStrategyThreshold_   = 0;
  use3dmc_                          = true;
  usePccRDO_                        = false;
  enhancedPP_                       = true;
  minWeightEPP_                     = 0.6;
  additionalProjectionPlaneMode_    = 0;
  partialAdditionalProjectionPlane_ = 0.00;

  // 3D and 2D bit depths
  geometry3dCoordinatesBitdepth_ = 10;
  geometryNominal2dBitdepth_     = 8;

  // Partitions and tiles
  enablePointCloudPartitioning_ = false;
  numTilesHor_                  = 2;
  tileHeightToWidthRatio_       = 1;

  // Sort raw points by Morton code
  mortonOrderSortRawPoints_       = false;
  attributeRawSeparateVideoWidth_ = 128;

  // Separate high gradient points
  highGradientSeparation_   = false;
  minGradient_              = 15.0;
  minNumHighGradientPoints_ = 256;

  // Patch border filtering
  pbfEnableFlag_    = false;
  pbfPassesCount_   = 0;
  pbfFilterSize_    = 0;
  pbfLog2Threshold_ = 2;

  patchPrecedenceOrderFlag_ = false;
  maxNumRefAtlasList_       = 1;
  maxNumRefAtlasFrame_      = 1;

  log2MaxAtlasFrameOrderCntLsb_ = 10;
  tileSegmentationType_         = 0;
  numMaxTilePerFrame_           = 1;
  uniformPartitionSpacing_      = true;
  tilePartitionWidth_           = 0;
  tilePartitionHeight_          = 0;
  tilePartitionWidthList_.clear();
  tilePartitionHeightList_.clear();

  // Profile tier level
  tierFlag_                 = 0;   // Low Tier
  profileCodecGroupIdc_     = 1;   // CODEC_GROUP_HEVC_MAIN10;  // HEVC Main10
  profileToolsetIdc_        = 1;   // V-PCC Basic V-PCC Extend
  profileReconstructionIdc_ = 0;   // Rec0, Rec1 or Rec2
  levelIdc_                 = 30;  // Corresponds to level 1.0 in Table A.5
  avcCodecIdIndex_          = 0;   // Index use if CMC SEI
  hevcCodecIdIndex_         = 1;   // Index use if CMC SEI
  shvcCodecIdIndex_         = 2;   // Index use if CMC SEI
  vvcCodecIdIndex_          = 3;   // Index use if CMC SEI

  // Profile toolset constraints information
  oneV3CFrameOnlyFlag_                     = 0;  // V-PCC Basic
  EOMContraintFlag_                        = false;
  maxMapCountMinus1_                       = 1;
  maxAtlasCountMinus1_                     = 0;
  multipleMapStreamsConstraintFlag_        = false;
  PLRConstraintFlag_                       = false;
  attributeMaxDimensionMinus1_             = 2;
  attributeMaxDimensionPartitionsMinus1_   = 0;
  noEightOrientationsConstraintFlag_       = 0;  // Default value, does not impose a constraint
  no45DegreeProjectionPatchConstraintFlag_ = 0;  // Default value, does not impose a constraint

  // reconstuction options
  pixelDeinterleavingType_      = 0;
  pointLocalReconstructionType_ = 0;
  reconstructEomType_           = 0;
  duplicatedPointRemovalType_   = 0;
  reconstructRawType_           = 0;
  applyGeoSmoothingType_        = 1;
  applyAttrSmoothingType_       = 0;
  attrTransferFilterType_       = 1;
  applyOccupanySynthesisType_   = 0;

  // SHVC
  shvcLayerIndex_ = 8;
  shvcRateX_      = 0;
  shvcRateY_      = 0;
}
}  // namespace jpcc::encoder