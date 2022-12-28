#include <jpcc/encoder/JPCCEncoderTMC2.h>

#include <sstream>

#include <PCCEncoderParameters.h>

using namespace pcc;

namespace jpcc::encoder {

PCCEncoderParameters toTMC2(const PCCEncoderTMC2Parameters& parameter) {
  PCCEncoderParameters result;
  result.startFrameNumber_                        = parameter.startFrameNumber_;
  result.configurationFolder_                     = parameter.configurationFolder_;
  result.uncompressedDataFolder_                  = parameter.uncompressedDataFolder_;
  result.compressedStreamPath_                    = parameter.compressedStreamPath_;
  result.reconstructedDataPath_                   = parameter.reconstructedDataPath_;
  result.colorTransform_                          = pcc::PCCColorTransform((int)parameter.colorTransform_);
  result.colorSpaceConversionPath_                = parameter.colorSpaceConversionPath_;
  result.videoEncoderOccupancyPath_               = parameter.videoEncoderOccupancyPath_;
  result.videoEncoderGeometryPath_                = parameter.videoEncoderGeometryPath_;
  result.videoEncoderAttributePath_               = parameter.videoEncoderAttributePath_;
  result.videoEncoderOccupancyCodecId_            = pcc::PCCCodecId((int)parameter.videoEncoderOccupancyCodecId_);
  result.videoEncoderGeometryCodecId_             = pcc::PCCCodecId((int)parameter.videoEncoderGeometryCodecId_);
  result.videoEncoderAttributeCodecId_            = pcc::PCCCodecId((int)parameter.videoEncoderAttributeCodecId_);
  result.byteStreamVideoCoderOccupancy_           = parameter.byteStreamVideoCoderOccupancy_;
  result.byteStreamVideoCoderGeometry_            = parameter.byteStreamVideoCoderGeometry_;
  result.byteStreamVideoCoderAttribute_           = parameter.byteStreamVideoCoderAttribute_;
  result.use3dmc_                                 = parameter.use3dmc_;
  result.usePccRDO_                               = parameter.usePccRDO_;
  result.colorSpaceConversionConfig_              = parameter.colorSpaceConversionConfig_;
  result.inverseColorSpaceConversionConfig_       = parameter.inverseColorSpaceConversionConfig_;
  result.nbThread_                                = parameter.nbThread_;
  result.frameCount_                              = parameter.frameCount_;
  result.groupOfFramesSize_                       = parameter.groupOfFramesSize_;
  result.uncompressedDataPath_                    = parameter.uncompressedDataPath_;
  result.forcedSsvhUnitSizePrecisionBytes_        = parameter.forcedSsvhUnitSizePrecisionBytes_;
  result.minimumImageWidth_                       = parameter.minimumImageWidth_;
  result.minimumImageHeight_                      = parameter.minimumImageHeight_;
  result.geometryQP_                              = parameter.geometryQP_;
  result.attributeQP_                             = parameter.attributeQP_;
  result.deltaQPD0_                               = parameter.deltaQPD0_;
  result.deltaQPD1_                               = parameter.deltaQPD1_;
  result.deltaQPT0_                               = parameter.deltaQPT0_;
  result.deltaQPT1_                               = parameter.deltaQPT1_;
  result.auxGeometryQP_                           = parameter.auxGeometryQP_;
  result.auxAttributeQP_                          = parameter.auxAttributeQP_;
  result.geometryConfig_                          = parameter.geometryConfig_;
  result.geometry0Config_                         = parameter.geometry0Config_;
  result.geometry1Config_                         = parameter.geometry1Config_;
  result.attributeConfig_                         = parameter.attributeConfig_;
  result.attribute0Config_                        = parameter.attribute0Config_;
  result.attribute1Config_                        = parameter.attribute1Config_;
  result.multipleStreams_                         = parameter.multipleStreams_;
  result.gridBasedSegmentation_                   = parameter.gridBasedSegmentation_;
  result.voxelDimensionGridBasedSegmentation_     = parameter.voxelDimensionGridBasedSegmentation_;
  result.nnNormalEstimation_                      = parameter.nnNormalEstimation_;
  result.normalOrientation_                       = parameter.normalOrientation_;
  result.gridBasedRefineSegmentation_             = parameter.gridBasedRefineSegmentation_;
  result.maxNNCountRefineSegmentation_            = parameter.maxNNCountRefineSegmentation_;
  result.iterationCountRefineSegmentation_        = parameter.iterationCountRefineSegmentation_;
  result.voxelDimensionRefineSegmentation_        = parameter.voxelDimensionRefineSegmentation_;
  result.searchRadiusRefineSegmentation_          = parameter.searchRadiusRefineSegmentation_;
  result.occupancyResolution_                     = parameter.occupancyResolution_;
  result.enablePatchSplitting_                    = parameter.enablePatchSplitting_;
  result.maxPatchSize_                            = parameter.maxPatchSize_;
  result.log2QuantizerSizeX_                      = parameter.log2QuantizerSizeX_;
  result.log2QuantizerSizeY_                      = parameter.log2QuantizerSizeY_;
  result.minPointCountPerCCPatchSegmentation_     = parameter.minPointCountPerCCPatchSegmentation_;
  result.maxNNCountPatchSegmentation_             = parameter.maxNNCountPatchSegmentation_;
  result.surfaceThickness_                        = parameter.surfaceThickness_;
  result.minLevel_                                = parameter.minLevel_;  // 8,16,32,64
  result.maxAllowedDist2RawPointsDetection_       = parameter.maxAllowedDist2RawPointsDetection_;
  result.maxAllowedDist2RawPointsSelection_       = parameter.maxAllowedDist2RawPointsSelection_;
  result.lambdaRefineSegmentation_                = parameter.lambdaRefineSegmentation_;
  result.mapCountMinus1_                          = parameter.mapCountMinus1_;
  result.maxCandidateCount_                       = parameter.maxCandidateCount_;
  result.occupancyPrecision_                      = parameter.occupancyPrecision_;
  result.occupancyMapConfig_                      = parameter.occupancyMapConfig_;
  result.occupancyMapQP_                          = parameter.occupancyMapQP_;
  result.EOMFixBitCount_                          = parameter.EOMFixBitCount_;
  result.occupancyMapRefinement_                  = parameter.occupancyMapRefinement_;
  result.decodedAtlasInformationHash_             = parameter.decodedAtlasInformationHash_;
  result.neighborCountSmoothing_                  = parameter.neighborCountSmoothing_;
  result.radius2Smoothing_                        = parameter.radius2Smoothing_;
  result.radius2BoundaryDetection_                = parameter.radius2BoundaryDetection_;
  result.thresholdSmoothing_                      = parameter.thresholdSmoothing_;
  result.gridSmoothing_                           = parameter.gridSmoothing_;
  result.gridSize_                                = parameter.gridSize_;
  result.flagGeometrySmoothing_                   = parameter.flagGeometrySmoothing_;
  result.patchExpansion_                          = parameter.patchExpansion_;
  result.thresholdColorSmoothing_                 = parameter.thresholdColorSmoothing_;
  result.thresholdColorDifference_                = parameter.thresholdColorDifference_;
  result.thresholdColorVariation_                 = parameter.thresholdColorVariation_;
  result.cgridSize_                               = parameter.cgridSize_;
  result.flagColorSmoothing_                      = parameter.flagColorSmoothing_;
  result.thresholdColorPreSmoothing_              = parameter.thresholdColorPreSmoothing_;
  result.thresholdColorPreSmoothingLocalEntropy_  = parameter.thresholdColorPreSmoothingLocalEntropy_;
  result.radius2ColorPreSmoothing_                = parameter.radius2ColorPreSmoothing_;
  result.neighborCountColorPreSmoothing_          = parameter.neighborCountColorPreSmoothing_;
  result.flagColorPreSmoothing_                   = parameter.flagColorPreSmoothing_;
  result.bestColorSearchRange_                    = parameter.bestColorSearchRange_;
  result.numNeighborsColorTransferFwd_            = parameter.numNeighborsColorTransferFwd_;
  result.numNeighborsColorTransferBwd_            = parameter.numNeighborsColorTransferBwd_;
  result.useDistWeightedAverageFwd_               = parameter.useDistWeightedAverageFwd_;
  result.useDistWeightedAverageBwd_               = parameter.useDistWeightedAverageBwd_;
  result.skipAvgIfIdenticalSourcePointPresentFwd_ = parameter.skipAvgIfIdenticalSourcePointPresentFwd_;
  result.skipAvgIfIdenticalSourcePointPresentBwd_ = parameter.skipAvgIfIdenticalSourcePointPresentBwd_;
  result.distOffsetFwd_                           = parameter.distOffsetFwd_;
  result.distOffsetBwd_                           = parameter.distOffsetBwd_;
  result.maxGeometryDist2Fwd_                     = parameter.maxGeometryDist2Fwd_;
  result.maxGeometryDist2Bwd_                     = parameter.maxGeometryDist2Bwd_;
  result.maxColorDist2Fwd_                        = parameter.maxColorDist2Fwd_;
  result.maxColorDist2Bwd_                        = parameter.maxColorDist2Bwd_;
  result.excludeColorOutlier_                     = parameter.excludeColorOutlier_;
  result.thresholdColorOutlierDist_               = parameter.thresholdColorOutlierDist_;
  result.noAttributes_                            = parameter.noAttributes_;
  result.rawPointsPatch_                          = parameter.rawPointsPatch_;
  result.attributeVideo444_                       = parameter.attributeVideo444_;
  result.useRawPointsSeparateVideo_               = parameter.useRawPointsSeparateVideo_;
  result.geometryAuxVideoConfig_                  = parameter.geometryAuxVideoConfig_;
  result.attributeAuxVideoConfig_                 = parameter.attributeAuxVideoConfig_;
  result.modelScale_                              = parameter.modelScale_;
  result.modelOrigin_              = {parameter.modelOrigin_[0], parameter.modelOrigin_[1], parameter.modelOrigin_[2]};
  result.levelOfDetailX_           = parameter.levelOfDetailX_;
  result.levelOfDetailY_           = parameter.levelOfDetailY_;
  result.keepIntermediateFiles_    = parameter.keepIntermediateFiles_;
  result.absoluteD1_               = parameter.absoluteD1_;
  result.absoluteT1_               = parameter.absoluteT1_;
  result.constrainedPack_          = parameter.constrainedPack_;
  result.groupDilation_            = parameter.groupDilation_;
  result.enhancedOccupancyMapCode_ = parameter.enhancedOccupancyMapCode_;
  result.offsetLossyOM_            = parameter.offsetLossyOM_;
  result.thresholdLossyOM_         = parameter.thresholdLossyOM_;
  result.prefilterLossyOM_         = parameter.prefilterLossyOM_;
  result.removeDuplicatePoints_    = parameter.removeDuplicatePoints_;
  result.pointLocalReconstruction_ = parameter.pointLocalReconstruction_;
  result.patchSize_                = parameter.patchSize_;
  result.plrlNumberOfModes_        = parameter.plrlNumberOfModes_;
  result.singleMapPixelInterleaving_              = parameter.singleMapPixelInterleaving_;
  result.patchColorSubsampling_                   = parameter.patchColorSubsampling_;
  result.surfaceSeparation_                       = parameter.surfaceSeparation_;
  result.highGradientSeparation_                  = parameter.highGradientSeparation_;
  result.minGradient_                             = parameter.minGradient_;
  result.minNumHighGradientPoints_                = parameter.minNumHighGradientPoints_;
  result.packingStrategy_                         = parameter.packingStrategy_;
  result.attributeBGFill_                         = parameter.attributeBGFill_;
  result.safeGuardDistance_                       = parameter.safeGuardDistance_;
  result.useEightOrientations_                    = parameter.useEightOrientations_;
  result.lossyRawPointsPatch_                     = parameter.lossyRawPointsPatch_;
  result.minNormSumOfInvDist4MPSelection_         = parameter.minNormSumOfInvDist4MPSelection_;
  result.globalPatchAllocation_                   = parameter.globalPatchAllocation_;
  result.globalPackingStrategyGOF_                = parameter.globalPackingStrategyGOF_;
  result.globalPackingStrategyReset_              = parameter.globalPackingStrategyReset_;
  result.globalPackingStrategyThreshold_          = parameter.globalPackingStrategyThreshold_;
  result.lowDelayEncoding_                        = parameter.lowDelayEncoding_;
  result.geometryPadding_                         = parameter.geometryPadding_;
  result.enhancedPP_                              = parameter.enhancedPP_;
  result.minWeightEPP_                            = parameter.minWeightEPP_;
  result.additionalProjectionPlaneMode_           = parameter.additionalProjectionPlaneMode_;
  result.partialAdditionalProjectionPlane_        = parameter.partialAdditionalProjectionPlane_;
  result.geometry3dCoordinatesBitdepth_           = parameter.geometry3dCoordinatesBitdepth_;
  result.geometryNominal2dBitdepth_               = parameter.geometryNominal2dBitdepth_;
  result.enablePointCloudPartitioning_            = parameter.enablePointCloudPartitioning_;
  result.roiBoundingBoxMinX_                      = parameter.roiBoundingBoxMinX_;
  result.roiBoundingBoxMaxX_                      = parameter.roiBoundingBoxMaxX_;
  result.roiBoundingBoxMinY_                      = parameter.roiBoundingBoxMinY_;
  result.roiBoundingBoxMaxY_                      = parameter.roiBoundingBoxMaxY_;
  result.roiBoundingBoxMinZ_                      = parameter.roiBoundingBoxMinZ_;
  result.roiBoundingBoxMaxZ_                      = parameter.roiBoundingBoxMaxZ_;
  result.numTilesHor_                             = parameter.numTilesHor_;
  result.tileHeightToWidthRatio_                  = parameter.tileHeightToWidthRatio_;
  result.numCutsAlong1stLongestAxis_              = parameter.numCutsAlong1stLongestAxis_;
  result.numCutsAlong2ndLongestAxis_              = parameter.numCutsAlong2ndLongestAxis_;
  result.numCutsAlong3rdLongestAxis_              = parameter.numCutsAlong3rdLongestAxis_;
  result.numROIs_                                 = parameter.numROIs_;
  result.mortonOrderSortRawPoints_                = parameter.mortonOrderSortRawPoints_;
  result.attributeRawSeparateVideoWidth_          = parameter.attributeRawSeparateVideoWidth_;
  result.pbfEnableFlag_                           = parameter.pbfEnableFlag_;
  result.pbfPassesCount_                          = parameter.pbfPassesCount_;
  result.pbfFilterSize_                           = parameter.pbfFilterSize_;
  result.pbfLog2Threshold_                        = parameter.pbfLog2Threshold_;
  result.patchPrecedenceOrderFlag_                = parameter.patchPrecedenceOrderFlag_;
  result.maxNumRefAtlasList_                      = parameter.maxNumRefAtlasList_;
  result.maxNumRefAtlasFrame_                     = parameter.maxNumRefAtlasFrame_;
  result.log2MaxAtlasFrameOrderCntLsb_            = parameter.log2MaxAtlasFrameOrderCntLsb_;
  result.tileSegmentationType_                    = parameter.tileSegmentationType_;
  result.numMaxTilePerFrame_                      = parameter.numMaxTilePerFrame_;
  result.uniformPartitionSpacing_                 = parameter.uniformPartitionSpacing_;
  result.tilePartitionWidth_                      = parameter.tilePartitionWidth_;
  result.tilePartitionHeight_                     = parameter.tilePartitionHeight_;
  result.tilePartitionWidthList_                  = parameter.tilePartitionWidthList_;
  result.tilePartitionHeightList_                 = parameter.tilePartitionHeightList_;
  result.tierFlag_                                = parameter.tierFlag_;
  result.profileCodecGroupIdc_                    = parameter.profileCodecGroupIdc_;
  result.profileToolsetIdc_                       = parameter.profileToolsetIdc_;
  result.profileReconstructionIdc_                = parameter.profileReconstructionIdc_;
  result.levelIdc_                                = parameter.levelIdc_;
  result.avcCodecIdIndex_                         = parameter.avcCodecIdIndex_;
  result.hevcCodecIdIndex_                        = parameter.hevcCodecIdIndex_;
  result.shvcCodecIdIndex_                        = parameter.shvcCodecIdIndex_;
  result.vvcCodecIdIndex_                         = parameter.vvcCodecIdIndex_;
  result.oneV3CFrameOnlyFlag_                     = parameter.oneV3CFrameOnlyFlag_;
  result.EOMContraintFlag_                        = parameter.EOMContraintFlag_;
  result.maxMapCountMinus1_                       = parameter.maxMapCountMinus1_;
  result.maxAtlasCountMinus1_                     = parameter.maxAtlasCountMinus1_;
  result.multipleMapStreamsConstraintFlag_        = parameter.multipleMapStreamsConstraintFlag_;
  result.PLRConstraintFlag_                       = parameter.PLRConstraintFlag_;
  result.attributeMaxDimensionMinus1_             = parameter.attributeMaxDimensionMinus1_;
  result.attributeMaxDimensionPartitionsMinus1_   = parameter.attributeMaxDimensionPartitionsMinus1_;
  result.noEightOrientationsConstraintFlag_       = parameter.noEightOrientationsConstraintFlag_;
  result.no45DegreeProjectionPatchConstraintFlag_ = parameter.no45DegreeProjectionPatchConstraintFlag_;
  result.pixelDeinterleavingType_                 = parameter.pixelDeinterleavingType_;
  result.pointLocalReconstructionType_            = parameter.pointLocalReconstructionType_;
  result.reconstructEomType_                      = parameter.reconstructEomType_;
  result.duplicatedPointRemovalType_              = parameter.duplicatedPointRemovalType_;
  result.reconstructRawType_                      = parameter.reconstructRawType_;
  result.applyGeoSmoothingType_                   = parameter.applyGeoSmoothingType_;
  result.applyAttrSmoothingType_                  = parameter.applyAttrSmoothingType_;
  result.attrTransferFilterType_                  = parameter.attrTransferFilterType_;
  result.applyOccupanySynthesisType_              = parameter.applyOccupanySynthesisType_;
  result.shvcLayerIndex_                          = parameter.shvcLayerIndex_;
  result.shvcRateX_                               = parameter.shvcRateX_;
  result.shvcRateY_                               = parameter.shvcRateY_;
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC2::JPCCEncoderTMC2(const JPCCEncoderTMC2Parameter parameter) : JPCCEncoder(), parameter_() {
  parameter_         = std::make_shared<PCCEncoderParameters>(toTMC2(parameter.encoderParams_));
  auto encoderParams = std::static_pointer_cast<PCCEncoderParameters>(parameter_);
  encoderParams->completePath();
  THROW_IF_NOT(encoderParams->check());
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderTMC2::isConvertToCoderTypeThreadSafe() {
  // TODO
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderTMC2::isEncodeThreadSafe() {
  // TODO
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC2::convertToCoderType(const FramePtr& frame, CoderFramePtr& coderFrame) {
  // TODO
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC2::encode(const CoderFramePtr& coderFrame, std::vector<char>& encodedBytes) {
  // TODO
}

}  // namespace jpcc::encoder