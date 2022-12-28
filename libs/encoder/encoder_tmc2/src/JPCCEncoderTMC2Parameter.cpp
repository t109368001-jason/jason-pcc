#include <jpcc/encoder/JPCCEncoderTMC2Parameter.h>

namespace jpcc::encoder {

using namespace std;
using namespace po;

#define configurationFolder_opt "configurationFolder"
#define uncompressedDataFolder_opt "uncompressedDataFolder"
#define uncompressedDataPath_opt "uncompressedDataPath"
#define compressedStreamPath_opt "compressedStreamPath"
#define reconstructedDataPath_opt "reconstructedDataPath"
#define forcedSsvhUnitSizePrecisionBytes_opt "forcedSsvhUnitSizePrecisionBytes"
#define startFrameNumber_opt "startFrameNumber"
#define frameCount_opt "frameCount"
#define groupOfFramesSize_opt "groupOfFramesSize"
#define colorTransform_opt "colorTransform"
#define colorSpaceConversionPath_opt "colorSpaceConversionPath"
#define colorSpaceConversionConfig_opt "colorSpaceConversionConfig"
#define inverseColorSpaceConversionConfig_opt "inverseColorSpaceConversionConfig"
#define gridBasedSegmentation_opt "gridBasedSegmentation"
#define voxelDimensionGridBasedSegmentation_opt "voxelDimensionGridBasedSegmentation"
#define nnNormalEstimation_opt "nnNormalEstimation"
#define normalOrientation_opt "normalOrientation"
#define gridBasedRefineSegmentation_opt "gridBasedRefineSegmentation"
#define maxNNCountRefineSegmentation_opt "maxNNCountRefineSegmentation"
#define iterationCountRefineSegmentation_opt "iterationCountRefineSegmentation"
#define voxelDimensionRefineSegmentation_opt "voxelDimensionRefineSegmentation"
#define searchRadiusRefineSegmentation_opt "searchRadiusRefineSegmentation"
#define occupancyResolution_opt "occupancyResolution"
#define enablePatchSplitting_opt "enablePatchSplitting"
#define maxPatchSize_opt "maxPatchSize"
#define log2QuantizerSizeX_opt "log2QuantizerSizeX"
#define log2QuantizerSizeY_opt "log2QuantizerSizeY"
#define minPointCountPerCCPatchSegmentation_opt "minPointCountPerCCPatchSegmentation"
#define maxNNCountPatchSegmentation_opt "maxNNCountPatchSegmentation"
#define surfaceThickness_opt "surfaceThickness"
#define depthQuantizationStep_opt "depthQuantizationStep"
#define maxAllowedDist2RawPointsDetection_opt "maxAllowedDist2RawPointsDetection"
#define maxAllowedDist2RawPointsSelection_opt "maxAllowedDist2RawPointsSelection"
#define lambdaRefineSegmentation_opt "lambdaRefineSegmentation"
#define minimumImageWidth_opt "minimumImageWidth"
#define minimumImageHeight_opt "minimumImageHeight"
#define maxCandidateCount_opt "maxCandidateCount"
#define occupancyPrecision_opt "occupancyPrecision"
#define occupancyMapConfig_opt "occupancyMapConfig"
#define occupancyMapQP_opt "occupancyMapQP"
#define enhancedOccupancyMapCode_opt "enhancedOccupancyMapCode"
#define EOMFixBitCount_opt "EOMFixBitCount"
#define occupancyMapRefinement_opt "occupancyMapRefinement"
#define decodedAtlasInformationHash_opt "decodedAtlasInformationHash"
#define attributeTransferFilterType_opt "attributeTransferFilterType"
#define flagGeometrySmoothing_opt "flagGeometrySmoothing"
#define neighborCountSmoothing_opt "neighborCountSmoothing"
#define radius2Smoothing_opt "radius2Smoothing"
#define radius2BoundaryDetection_opt "radius2BoundaryDetection"
#define thresholdSmoothing_opt "thresholdSmoothing"
#define patchExpansion_opt "patchExpansion"
#define gridSmoothing_opt "gridSmoothing"
#define gridSize_opt "gridSize"
#define thresholdColorSmoothing_opt "thresholdColorSmoothing"
#define cgridSize_opt "cgridSize"
#define thresholdColorDifference_opt "thresholdColorDifference"
#define thresholdColorVariation_opt "thresholdColorVariation"
#define flagColorSmoothing_opt "flagColorSmoothing"
#define thresholdColorPreSmoothing_opt "thresholdColorPreSmoothing"
#define thresholdColorPreSmoothingLocalEntropy_opt "thresholdColorPreSmoothingLocalEntropy"
#define radius2ColorPreSmoothing_opt "radius2ColorPreSmoothing"
#define neighborCountColorPreSmoothing_opt "neighborCountColorPreSmoothing"
#define flagColorPreSmoothing_opt "flagColorPreSmoothing"
#define bestColorSearchRange_opt "bestColorSearchRange"
#define numNeighborsColorTransferFwd_opt "numNeighborsColorTransferFwd"
#define numNeighborsColorTransferBwd_opt "numNeighborsColorTransferBwd"
#define useDistWeightedAverageFwd_opt "useDistWeightedAverageFwd"
#define useDistWeightedAverageBwd_opt "useDistWeightedAverageBwd"
#define skipAvgIfIdenticalSourcePointPresentFwd_opt "skipAvgIfIdenticalSourcePointPresentFwd"
#define skipAvgIfIdenticalSourcePointPresentBwd_opt "skipAvgIfIdenticalSourcePointPresentBwd"
#define distOffsetFwd_opt "distOffsetFwd"
#define distOffsetBwd_opt "distOffsetBwd"
#define maxGeometryDist2Fwd_opt "maxGeometryDist2Fwd"
#define maxGeometryDist2Bwd_opt "maxGeometryDist2Bwd"
#define maxColorDist2Fwd_opt "maxColorDist2Fwd"
#define maxColorDist2Bwd_opt "maxColorDist2Bwd"
#define excludeColorOutlier_opt "excludeColorOutlier"
#define thresholdColorOutlierDist_opt "thresholdColorOutlierDist"
#define videoEncoderOccupancyPath_opt "videoEncoderOccupancyPath"
#define videoEncoderGeometryPath_opt "videoEncoderGeometryPath"
#define videoEncoderAttributePath_opt "videoEncoderAttributePath"
#define videoEncoderOccupancyCodecId_opt "videoEncoderOccupancyCodecId"
#define videoEncoderGeometryCodecId_opt "videoEncoderGeometryCodecId"
#define videoEncoderAttributeCodecId_opt "videoEncoderAttributeCodecId"
#define byteStreamVideoEncoderOccupancy_opt "byteStreamVideoEncoderOccupancy"
#define byteStreamVideoEncoderGeometry_opt "byteStreamVideoEncoderGeometry"
#define byteStreamVideoEncoderAttribute_opt "byteStreamVideoEncoderAttribute"
#define geometryQP_opt "geometryQP"
#define attributeQP_opt "attributeQP"
#define auxGeometryQP_opt "auxGeometryQP"
#define auxAttributeQP_opt "auxAttributeQP"
#define geometryConfig_opt "geometryConfig"
#define geometry0Config_opt "geometry0Config"
#define geometry1Config_opt "geometry1Config"
#define attributeConfig_opt "attributeConfig"
#define attribute0Config_opt "attribute0Config"
#define attribute1Config_opt "attribute1Config"
#define rawPointsPatch_opt "rawPointsPatch"
#define noAttributes_opt "noAttributes"
#define attributeVideo444_opt "attributeVideo444"
#define useRawPointsSeparateVideo_opt "useRawPointsSeparateVideo"
#define attributeRawSeparateVideoWidth_opt "attributeRawSeparateVideoWidth"
#define geometryMPConfig_opt "geometryMPConfig"
#define attributeMPConfig_opt "attributeMPConfig"
#define nbThread_opt "nbThread"
#define keepIntermediateFiles_opt "keepIntermediateFiles"
#define absoluteD1_opt "absoluteD1"
#define absoluteT1_opt "absoluteT1"
#define multipleStreams_opt "multipleStreams"
#define deltaQPD0_opt "deltaQPD0"
#define deltaQPD1_opt "deltaQPD1"
#define deltaQPT0_opt "deltaQPT0"
#define deltaQPT1_opt "deltaQPT1"
#define constrainedPack_opt "constrainedPack"
#define levelOfDetailX_opt "levelOfDetailX"
#define levelOfDetailY_opt "levelOfDetailY"
#define groupDilation_opt "groupDilation"
#define offsetLossyOM_opt "offsetLossyOM"
#define thresholdLossyOM_opt "thresholdLossyOM"
#define prefilterLossyOM_opt "prefilterLossyOM"
#define shvcLayerIndex_opt "shvcLayerIndex"
#define shvcRateX_opt "shvcRateX"
#define shvcRateY_opt "shvcRateY"
#define patchColorSubsampling_opt "patchColorSubsampling"
#define maxNumRefAtalsList_opt "maxNumRefAtalsList"
#define maxNumRefAtlasFrame_opt "maxNumRefAtlasFrame"
#define pointLocalReconstruction_opt "pointLocalReconstruction"
#define mapCountMinus1_opt "mapCountMinus1"
#define singleMapPixelInterleaving_opt "singleMapPixelInterleaving"
#define removeDuplicatePoints_opt "removeDuplicatePoints"
#define surfaceSeparation_opt "surfaceSeparation"
#define highGradientSeparation_opt "highGradientSeparation"
#define minGradient_opt "minGradient"
#define minNumHighGradientPoints_opt "minNumHighGradientPoints"
#define packingStrategy_opt "packingStrategy"
#define useEightOrientations_opt "useEightOrientations"
#define safeGuardDistance_opt "safeGuardDistance"
#define attributeBGFill_opt "attributeBGFill"
#define lossyRawPointsPatch_opt "lossyRawPointsPatch"
#define minNormSumOfInvDist4MPSelection_opt "minNormSumOfInvDist4MPSelection"
#define globalPatchAllocation_opt "globalPatchAllocation"
#define globalPackingStrategyGOF_opt "globalPackingStrategyGOF"
#define globalPackingStrategyReset_opt "globalPackingStrategyReset"
#define globalPackingStrategyThreshold_opt "globalPackingStrategyThreshold"
#define patchPrecedenceOrder_opt "patchPrecedenceOrder"
#define lowDelayEncoding_opt "lowDelayEncoding"
#define geometryPadding_opt "geometryPadding"
#define apply3dMotionCompensation_opt "apply3dMotionCompensation"
#define usePccRDO_opt "usePccRDO"
#define geometry3dCoordinatesBitdepth_opt "geometry3dCoordinatesBitdepth"
#define geometryNominal2dBitdepth_opt "geometryNominal2dBitdepth"
#define nbPlrmMode_opt "nbPlrmMode"
#define patchSize_opt "patchSize"
#define enhancedProjectionPlane_opt "enhancedProjectionPlane"
#define minWeightEPP_opt "minWeightEPP"
#define additionalProjectionPlaneMode_opt "additionalProjectionPlaneMode"
#define partialAdditionalProjectionPlane_opt "partialAdditionalProjectionPlane"
#define numMaxTilePerFrame_opt "numMaxTilePerFrame"
#define uniformPartitionSpacing_opt "uniformPartitionSpacing"
#define tilePartitionWidth_opt "tilePartitionWidth"
#define tilePartitionHeight_opt "tilePartitionHeight"
#define tilePartitionWidthList_opt "tilePartitionWidthList"
#define tilePartitionHeightList_opt "tilePartitionHeightList"
#define tileSegmentationType_opt "tileSegmentationType"
#define enablePointCloudPartitioning_opt "enablePointCloudPartitioning"
#define roiBoundingBoxMinX_opt "roiBoundingBoxMinX"
#define roiBoundingBoxMaxX_opt "roiBoundingBoxMaxX"
#define roiBoundingBoxMinY_opt "roiBoundingBoxMinY"
#define roiBoundingBoxMaxY_opt "roiBoundingBoxMaxY"
#define roiBoundingBoxMinZ_opt "roiBoundingBoxMinZ"
#define roiBoundingBoxMaxZ_opt "roiBoundingBoxMaxZ"
#define numTilesHor_opt "numTilesHor"
#define tileHeightToWidthRatio_opt "tileHeightToWidthRatio"
#define numCutsAlong1stLongestAxis_opt "numCutsAlong1stLongestAxis"
#define numCutsAlong2ndLongestAxis_opt "numCutsAlong2ndLongestAxis"
#define numCutsAlong3rdLongestAxis_opt "numCutsAlong3rdLongestAxis"
#define mortonOrderSortRawPoints_opt "mortonOrderSortRawPoints"
#define pbfEnableFlag_opt "pbfEnableFlag"
#define pbfFilterSize_opt "pbfFilterSize"
#define pbfPassesCount_opt "pbfPassesCount"
#define pbfLog2Threshold_opt "pbfLog2Threshold"
#define tierFlag_opt "tierFlag"
#define profileCodecGroupIdc_opt "profileCodecGroupIdc"
#define profileToolsetIdc_opt "profileToolsetIdc"
#define profileReconstructionIdc_opt "profileReconstructionIdc"
#define levelIdc_opt "levelIdc"
#define avcCodecIdIndex_opt "avcCodecIdIndex"
#define hevcCodecIdIndex_opt "hevcCodecIdIndex"
#define shvcCodecIdIndex_opt "shvcCodecIdIndex"
#define vvcCodecIdIndex_opt "vvcCodecIdIndex"
#define oneV3CFrameOnlyFlag_opt "oneV3CFrameOnlyFlag"

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC2Parameter::JPCCEncoderTMC2Parameter() :
    JPCCEncoderTMC2Parameter(JPCC_ENCODER_TMC2_OPT_PREFIX, __FUNCTION__) {}

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC2Parameter::JPCCEncoderTMC2Parameter(  // NOLINT(cppcoreguidelines-pro-type-member-init)
    const string& prefix,
    const string& caption) :
    Parameter(prefix, caption) {
  setDefault();
  opts_.add_options()                                                                     //
      (string(prefix_ + configurationFolder_opt).c_str(),                                 //
       value<string>(&this->encoderParams_.configurationFolder_),                         //
       "Folder where the configuration files are stored,use for cfg relative paths.")     //
      (string(prefix_ + uncompressedDataFolder_opt).c_str(),                              //
       value<string>(&this->encoderParams_.uncompressedDataFolder_),                      //
       "Folder where the uncompress input data are stored, use for cfg relative paths.")  //

      // i/o
      (string(prefix_ + uncompressedDataPath_opt).c_str(),                              //
       value<string>(&this->encoderParams_.uncompressedDataPath_),                      //
       "Input pointcloud to encode. Multi-frame sequences may be represented by %04i")  //
      (string(prefix_ + compressedStreamPath_opt).c_str(),                              //
       value<string>(&this->encoderParams_.compressedStreamPath_),                      //
       "Output(encoder)/Input(decoder) compressed bitstream")                           //
      (string(prefix_ + reconstructedDataPath_opt).c_str(),                             //
       value<string>(&this->encoderParams_.reconstructedDataPath_),                     //
       "Output decoded pointcloud. Multi-frame sequences may be represented by %04i")   //
      (string(prefix_ + forcedSsvhUnitSizePrecisionBytes_opt).c_str(),                  //
       value<uint32_t>(&this->encoderParams_.forcedSsvhUnitSizePrecisionBytes_),        //
       "forced SSVH unit size precision bytes")                                         //

      // sequence configuration
      (string(prefix_ + startFrameNumber_opt).c_str(),           //
       value<size_t>(&this->encoderParams_.startFrameNumber_),   //
       "First frame number in sequence to encode/decode")        //
      (string(prefix_ + frameCount_opt).c_str(),                 //
       value<size_t>(&this->encoderParams_.frameCount_),         //
       "Number of frames to encode")                             //
      (string(prefix_ + groupOfFramesSize_opt).c_str(),          //
       value<size_t>(&this->encoderParams_.groupOfFramesSize_),  //
       "Random access period")                                   //

      // colour space conversion
      (string(prefix_ + colorTransform_opt).c_str(),                                    //
       value<PCCColorTransform>(&this->encoderParams_.colorTransform_),                 //
       "The colour transform to be applied:\n  0: none\n  1: RGB to YCbCr (Rec.709)")   //
      (string(prefix_ + colorSpaceConversionPath_opt).c_str(),                          //
       value<string>(&this->encoderParams_.colorSpaceConversionPath_),                  //
       "Path to the HDRConvert. If unset, an internal color space conversion is used")  //
      (string(prefix_ + colorSpaceConversionConfig_opt).c_str(),                        //
       value<string>(&this->encoderParams_.colorSpaceConversionConfig_),                //
       "HDRConvert configuration file used for RGB444 to YUV420 conversion")            //
      (string(prefix_ + inverseColorSpaceConversionConfig_opt).c_str(),                 //
       value<string>(&this->encoderParams_.inverseColorSpaceConversionConfig_),         //
       "HDRConvert configuration file used for YUV420 to RGB444 conversion")            //

      // segmentation
      // If enabled, change refineSegmentationGridBased() parameters according to m56857"
      (string(prefix_ + gridBasedSegmentation_opt).c_str(),                                 //
       value<bool>(&this->encoderParams_.gridBasedSegmentation_),                           //
       "Voxel dimension for grid-based segmentation (GBS)")                                 //
      (string(prefix_ + voxelDimensionGridBasedSegmentation_opt).c_str(),                   //
       value<size_t>(&this->encoderParams_.voxelDimensionGridBasedSegmentation_),           //
       "Voxel dimension for grid-based segmentation (GBS)")                                 //
      (string(prefix_ + nnNormalEstimation_opt).c_str(),                                    //
       value<size_t>(&this->encoderParams_.nnNormalEstimation_),                            //
       "Number of points used for normal estimation")                                       //
      (string(prefix_ + normalOrientation_opt).c_str(),                                     //
       value<size_t>(&this->encoderParams_.normalOrientation_),                             //
       "Normal orientation: 0: None 1: spanning tree, 2:view point, 3:cubemap projection")  //
      (string(prefix_ + gridBasedRefineSegmentation_opt).c_str(),                           //
       value<bool>(&this->encoderParams_.gridBasedRefineSegmentation_),                     //
       "Use grid-based approach for segmentation refinement")                               //
      (string(prefix_ + maxNNCountRefineSegmentation_opt).c_str(),                          //
       value<size_t>(&this->encoderParams_.maxNNCountRefineSegmentation_),                  //
       "Number of nearest neighbors used during segmentation refinement")                   //
      (string(prefix_ + iterationCountRefineSegmentation_opt).c_str(),                      //
       value<size_t>(&this->encoderParams_.iterationCountRefineSegmentation_),              //
       "Number of iterations performed during segmentation refinement")                     //
      (string(prefix_ + voxelDimensionRefineSegmentation_opt).c_str(),                      //
       value<size_t>(&this->encoderParams_.voxelDimensionRefineSegmentation_),              //
       "Voxel dimension for segmentation refinement (must be a power of 2)")                //
      (string(prefix_ + searchRadiusRefineSegmentation_opt).c_str(),                        //
       value<size_t>(&this->encoderParams_.searchRadiusRefineSegmentation_),                //
       "Search radius for segmentation refinement")                                         //
      (string(prefix_ + occupancyResolution_opt).c_str(),                                   //
       value<size_t>(&this->encoderParams_.occupancyResolution_),                           //
       "Resolution of packing block(a block contain only one patch)")                       //
      (string(prefix_ + enablePatchSplitting_opt).c_str(),                                  //
       value<bool>(&this->encoderParams_.enablePatchSplitting_),                            //
       "Enable patch splitting")                                                            //
      (string(prefix_ + maxPatchSize_opt).c_str(),                                          //
       value<size_t>(&this->encoderParams_.maxPatchSize_),                                  //
       "Maximum patch size for segmentation")                                               //
      (string(prefix_ + log2QuantizerSizeX_opt).c_str(),                                    //
       value<size_t>(&this->encoderParams_.log2QuantizerSizeX_),                            //
       "log2 of Quantization step for patch size X: 0. pixel precision 4.16 as before")     //
      (string(prefix_ + log2QuantizerSizeY_opt).c_str(),                                    //
       value<size_t>(&this->encoderParams_.log2QuantizerSizeY_),                            //
       "log2 of Quantization step for patch size Y: 0. pixel precision 4.16 as before")     //
      (string(prefix_ + minPointCountPerCCPatchSegmentation_opt).c_str(),                   //
       value<size_t>(&this->encoderParams_.minPointCountPerCCPatchSegmentation_),           //
       "Minimum number of points for a connected component to be retained as a patch")      //
      (string(prefix_ + maxNNCountPatchSegmentation_opt).c_str(),                           //
       value<size_t>(&this->encoderParams_.maxNNCountPatchSegmentation_),                   //
       "Number of nearest neighbors used during connected components extraction")           //
      (string(prefix_ + surfaceThickness_opt).c_str(),                                      //
       value<size_t>(&this->encoderParams_.surfaceThickness_),                              //
       "Surface thickness")                                                                 //
      (string(prefix_ + depthQuantizationStep_opt).c_str(),                                 //
       value<size_t>(&this->encoderParams_.minLevel_),                                      //
       "minimum level for patches")                                                         //
      (string(prefix_ + maxAllowedDist2RawPointsDetection_opt).c_str(),                     //
       value<double>(&this->encoderParams_.maxAllowedDist2RawPointsDetection_),             //
       "Maximum distance for a point to be ignored during raw points detection")            //
      (string(prefix_ + maxAllowedDist2RawPointsSelection_opt).c_str(),                     //
       value<double>(&this->encoderParams_.maxAllowedDist2RawPointsSelection_),             //
       "Maximum distance for a point to be ignored during  raw points  selection")          //
      (string(prefix_ + lambdaRefineSegmentation_opt).c_str(),                              //
       value<double>(&this->encoderParams_.lambdaRefineSegmentation_),                      //
       "Controls the smoothness of the patch boundaries  during segmentation  refinement")  //

      // packing
      (string(prefix_ + minimumImageWidth_opt).c_str(),           //
       value<size_t>(&this->encoderParams_.minimumImageWidth_),   //
       "Minimum width of packed patch frame")                     //
      (string(prefix_ + minimumImageHeight_opt).c_str(),          //
       value<size_t>(&this->encoderParams_.minimumImageHeight_),  //
       "Minimum height of packed patch frame")                    //

      // occupancy map
      (string(prefix_ + maxCandidateCount_opt).c_str(),           //
       value<size_t>(&this->encoderParams_.maxCandidateCount_),   //
       "Maximum nuber of candidates in list L")                   //
      (string(prefix_ + occupancyPrecision_opt).c_str(),          //
       value<size_t>(&this->encoderParams_.occupancyPrecision_),  //
       "Occupancy map B0 precision")                              //
      (string(prefix_ + occupancyMapConfig_opt).c_str(),          //
       value<string>(&this->encoderParams_.occupancyMapConfig_),  //
       "Occupancy map encoder config file")                       //
      (string(prefix_ + occupancyMapQP_opt).c_str(),              //
       value<size_t>(&this->encoderParams_.occupancyMapQP_),      //
       "QP for compression of occupancy map video")               //

      // EOM code
      (string(prefix_ + enhancedOccupancyMapCode_opt).c_str(),        //
       value<bool>(&this->encoderParams_.enhancedOccupancyMapCode_),  //
       "Use enhanced-delta-depth code")                               //
      (string(prefix_ + EOMFixBitCount_opt).c_str(),                  //
       value<size_t>(&this->encoderParams_.EOMFixBitCount_),          //
       "enhanced occupancy map fixed bit count")                      //
      (string(prefix_ + occupancyMapRefinement_opt).c_str(),          //
       value<bool>(&this->encoderParams_.occupancyMapRefinement_),    //
       "Use occupancy map refinement")                                //

      // hash
      (string(prefix_ + decodedAtlasInformationHash_opt).c_str(),                    //
       value<size_t>(&this->encoderParams_.decodedAtlasInformationHash_),            //
       "Enable decoded atlas information hash 0. disable 1.MD5 2.CRC 3.Checksum\n")  //

      // smoothing
      (string(prefix_ + attributeTransferFilterType_opt).c_str(),       //
       value<size_t>(&this->encoderParams_.attrTransferFilterType_),    //
       "Exclude geometry smoothing from attribute transfer\n")          //
      (string(prefix_ + flagGeometrySmoothing_opt).c_str(),             //
       value<bool>(&this->encoderParams_.flagGeometrySmoothing_),       //
       "Enable geometry smoothing\n")                                   //
      (string(prefix_ + neighborCountSmoothing_opt).c_str(),            //
       value<size_t>(&this->encoderParams_.neighborCountSmoothing_),    //
       "Neighbor count smoothing")                                      //
      (string(prefix_ + radius2Smoothing_opt).c_str(),                  //
       value<double>(&this->encoderParams_.radius2Smoothing_),          //
       "Radius to smoothing")                                           //
      (string(prefix_ + radius2BoundaryDetection_opt).c_str(),          //
       value<double>(&this->encoderParams_.radius2BoundaryDetection_),  //
       "Radius to boundary detection")                                  //
      (string(prefix_ + thresholdSmoothing_opt).c_str(),                //
       value<double>(&this->encoderParams_.thresholdSmoothing_),        //
       "Threshold smoothing")                                           //

      // Patch Expansion (m47772 CE2.12)
      (string(prefix_ + patchExpansion_opt).c_str(),        //
       value<bool>(&this->encoderParams_.patchExpansion_),  //
       "Use occupancy map refinement")                      //

      // grid smoothing (m44705 CE2.17)
      (string(prefix_ + gridSmoothing_opt).c_str(),        //
       value<bool>(&this->encoderParams_.gridSmoothing_),  //
       "Enable grid smoothing")                            //
      (string(prefix_ + gridSize_opt).c_str(),             //
       value<size_t>(&this->encoderParams_.gridSize_),     //
       "grid size for the smoothing")                      //

      // color smoothing
      (string(prefix_ + thresholdColorSmoothing_opt).c_str(),           //
       value<double>(&this->encoderParams_.thresholdColorSmoothing_),   //
       "Threshold of color smoothing")                                  //
      (string(prefix_ + cgridSize_opt).c_str(),                         //
       value<size_t>(&this->encoderParams_.cgridSize_),                 //
       "grid size for the color smoothing")                             //
      (string(prefix_ + thresholdColorDifference_opt).c_str(),          //
       value<double>(&this->encoderParams_.thresholdColorDifference_),  //
       "Threshold of color difference between cells")                   //
      (string(prefix_ + thresholdColorVariation_opt).c_str(),           //
       value<double>(&this->encoderParams_.thresholdColorVariation_),   //
       "Threshold of color variation in cells")                         //
      (string(prefix_ + flagColorSmoothing_opt).c_str(),                //
       value<bool>(&this->encoderParams_.flagColorSmoothing_),          //
       "Enable color smoothing\n")                                      //

      // color pre-smoothing
      (string(prefix_ + thresholdColorPreSmoothing_opt).c_str(),                      //
       value<double>(&this->encoderParams_.thresholdColorPreSmoothing_),              //
       "Threshold of color pre-smoothing")                                            //
      (string(prefix_ + thresholdColorPreSmoothingLocalEntropy_opt).c_str(),          //
       value<double>(&this->encoderParams_.thresholdColorPreSmoothingLocalEntropy_),  //
       "Threshold of color pre-smoothing local entropy")                              //
      (string(prefix_ + radius2ColorPreSmoothing_opt).c_str(),                        //
       value<double>(&this->encoderParams_.radius2ColorPreSmoothing_),                //
       "Redius of color pre-smoothing")                                               //
      (string(prefix_ + neighborCountColorPreSmoothing_opt).c_str(),                  //
       value<size_t>(&this->encoderParams_.neighborCountColorPreSmoothing_),          //
       "Neighbor count for color pre-smoothing")                                      //
      (string(prefix_ + flagColorPreSmoothing_opt).c_str(),                           //
       value<bool>(&this->encoderParams_.flagColorPreSmoothing_),                     //
       "Enable color pre-smoothing\n")                                                //

      // colouring
      (string(prefix_ + bestColorSearchRange_opt).c_str(),          //
       value<size_t>(&this->encoderParams_.bestColorSearchRange_),  //
       "Best color search range")                                   //

      // Improved color transfer (m49367 CE2.17)
      (string(prefix_ + numNeighborsColorTransferFwd_opt).c_str(),                   //
       value<int>(&this->encoderParams_.numNeighborsColorTransferFwd_),              //
       "Number of neighbors creating Fwd list")                                      //
      (string(prefix_ + numNeighborsColorTransferBwd_opt).c_str(),                   //
       value<int>(&this->encoderParams_.numNeighborsColorTransferBwd_),              //
       "Number of neighbors creating Bwd list")                                      //
      (string(prefix_ + useDistWeightedAverageFwd_opt).c_str(),                      //
       value<bool>(&this->encoderParams_.useDistWeightedAverageFwd_),                //
       "Distance weighted average for Fwd list")                                     //
      (string(prefix_ + useDistWeightedAverageBwd_opt).c_str(),                      //
       value<bool>(&this->encoderParams_.useDistWeightedAverageBwd_),                //
       "Distance weighted average for Bwd list")                                     //
      (string(prefix_ + skipAvgIfIdenticalSourcePointPresentFwd_opt).c_str(),        //
       value<bool>(&this->encoderParams_.skipAvgIfIdenticalSourcePointPresentFwd_),  //
       "Skip avgeraging if target is identical to a Fwd point")                      //
      (string(prefix_ + skipAvgIfIdenticalSourcePointPresentBwd_opt).c_str(),        //
       value<bool>(&this->encoderParams_.skipAvgIfIdenticalSourcePointPresentBwd_),  //
       "Skip avgeraging if target is identical to a Bwd point")                      //
      (string(prefix_ + distOffsetFwd_opt).c_str(),                                  //
       value<double>(&this->encoderParams_.distOffsetFwd_),                          //
       "Distance offset to avoid infinite weight")                                   //
      (string(prefix_ + distOffsetBwd_opt).c_str(),                                  //
       value<double>(&this->encoderParams_.distOffsetBwd_),                          //
       "Distance offset to avoid infinite weight")                                   //
      (string(prefix_ + maxGeometryDist2Fwd_opt).c_str(),                            //
       value<double>(&this->encoderParams_.maxGeometryDist2Fwd_),                    //
       "Maximum allowed distance for a Fwd point")                                   //
      (string(prefix_ + maxGeometryDist2Bwd_opt).c_str(),                            //
       value<double>(&this->encoderParams_.maxGeometryDist2Bwd_),                    //
       "Maximum allowed distance for a Bwd point")                                   //
      (string(prefix_ + maxColorDist2Fwd_opt).c_str(),                               //
       value<double>(&this->encoderParams_.maxColorDist2Fwd_),                       //
       "Maximum allowed pari-wise color distance for Fwd list")                      //
      (string(prefix_ + maxColorDist2Bwd_opt).c_str(),                               //
       value<double>(&this->encoderParams_.maxColorDist2Bwd_),                       //
       "Maximum allowed pari-wise color distance for Bwd list")                      //
      (string(prefix_ + excludeColorOutlier_opt).c_str(),                            //
       value<bool>(&this->encoderParams_.excludeColorOutlier_),                      //
       "Exclude color outliers from the NN set")                                     //
      (string(prefix_ + thresholdColorOutlierDist_opt).c_str(),                      //
       value<double>(&this->encoderParams_.thresholdColorOutlierDist_),              //
       "Threshold of color distance to exclude outliers from the NN set")            //

      // video encoding
      (string(prefix_ + videoEncoderOccupancyPath_opt).c_str(),                 //
       value<string>(&this->encoderParams_.videoEncoderOccupancyPath_),         //
       "Occupancy video encoder executable path")                               //
      (string(prefix_ + videoEncoderGeometryPath_opt).c_str(),                  //
       value<string>(&this->encoderParams_.videoEncoderGeometryPath_),          //
       "Geometry video encoder executable path")                                //
      (string(prefix_ + videoEncoderAttributePath_opt).c_str(),                 //
       value<string>(&this->encoderParams_.videoEncoderAttributePath_),         //
       "Attribute video encoder executable path")                               //
      (string(prefix_ + videoEncoderOccupancyCodecId_opt).c_str(),              //
       value<PCCCodecId>(&this->encoderParams_.videoEncoderOccupancyCodecId_),  //
       "Occupancy video encoder codec id")                                      //
      (string(prefix_ + videoEncoderGeometryCodecId_opt).c_str(),               //
       value<PCCCodecId>(&this->encoderParams_.videoEncoderGeometryCodecId_),   //
       "Geometry video encoder codec id")                                       //
      (string(prefix_ + videoEncoderAttributeCodecId_opt).c_str(),              //
       value<PCCCodecId>(&this->encoderParams_.videoEncoderAttributeCodecId_),  //
       "Attribute video encoder codec id")                                      //
      (string(prefix_ + byteStreamVideoEncoderOccupancy_opt).c_str(),           //
       value<bool>(&this->encoderParams_.byteStreamVideoCoderOccupancy_),       //
       "Attribute video encoder outputs byteStream")                            //
      (string(prefix_ + byteStreamVideoEncoderGeometry_opt).c_str(),            //
       value<bool>(&this->encoderParams_.byteStreamVideoCoderGeometry_),        //
       "Attribute video encoder outputs byteStream")                            //
      (string(prefix_ + byteStreamVideoEncoderAttribute_opt).c_str(),           //
       value<bool>(&this->encoderParams_.byteStreamVideoCoderAttribute_),       //
       "Attribute video encoder outputs byteStream")                            //

      (string(prefix_ + geometryQP_opt).c_str(),                              //
       value<int>(&this->encoderParams_.geometryQP_),                         //
       "QP for compression of geometry video")                                //
      (string(prefix_ + attributeQP_opt).c_str(),                             //
       value<int>(&this->encoderParams_.attributeQP_),                        //
       "QP for compression of attribute video")                               //
      (string(prefix_ + auxGeometryQP_opt).c_str(),                           //
       value<int>(&this->encoderParams_.auxGeometryQP_),                      //
       "QP for compression of auxiliary geometry video : "                    //
       "default=4 for lossy raw points, geometryQP for lossless raw points")  //
      (string(prefix_ + auxAttributeQP_opt).c_str(),                          //
       value<int>(&this->encoderParams_.auxAttributeQP_),                     //
       "QP for compression of auxiliary attribute video")                     //
      (string(prefix_ + geometryConfig_opt).c_str(),                          //
       value<string>(&this->encoderParams_.geometryConfig_),                  //
       "HM configuration file for geometry compression")                      //
      (string(prefix_ + geometry0Config_opt).c_str(),                         //
       value<string>(&this->encoderParams_.geometry0Config_),                 //
       "HM configuration file for geometry 0 compression")                    //
      (string(prefix_ + geometry1Config_opt).c_str(),                         //
       value<string>(&this->encoderParams_.geometry1Config_),                 //
       "HM configuration file for geometry 1 compression")                    //
      (string(prefix_ + attributeConfig_opt).c_str(),                         //
       value<string>(&this->encoderParams_.attributeConfig_),                 //
       "HM configuration file for attribute compression")                     //
      (string(prefix_ + attribute0Config_opt).c_str(),                        //
       value<string>(&this->encoderParams_.attribute0Config_),                //
       "HM configuration file for attribute 0 compression")                   //
      (string(prefix_ + attribute1Config_opt).c_str(),                        //
       value<string>(&this->encoderParams_.attribute1Config_),                //
       "HM configuration file for attribute 1 compression")                   //
      (string(prefix_ + rawPointsPatch_opt).c_str(),                          //
       value<bool>(&this->encoderParams_.rawPointsPatch_),                    //
       "Enable raw points patch\n")                                           //
      (string(prefix_ + noAttributes_opt).c_str(),                            //
       value<bool>(&this->encoderParams_.noAttributes_),                      //
       "Disable encoding of attributes")                                      //
      (string(prefix_ + attributeVideo444_opt).c_str(),                       //
       value<bool>(&this->encoderParams_.attributeVideo444_),                 //
       "Use 444 format for attribute video")                                  //
      (string(prefix_ + useRawPointsSeparateVideo_opt).c_str(),               //
       value<bool>(&this->encoderParams_.useRawPointsSeparateVideo_),         //
       "compress raw points with video codec")                                //
      (string(prefix_ + attributeRawSeparateVideoWidth_opt).c_str(),          //
       value<size_t>(&this->encoderParams_.attributeRawSeparateVideoWidth_),  //
       "Width of the MP's attribute in separate video")                       //
      (string(prefix_ + geometryMPConfig_opt).c_str(),                        //
       value<string>(&this->encoderParams_.geometryAuxVideoConfig_),          //
       "HM configuration file for raw points geometry compression")           //
      (string(prefix_ + attributeMPConfig_opt).c_str(),                       //
       value<string>(&this->encoderParams_.attributeAuxVideoConfig_),         //
       "HM configuration file for raw points attribute compression")          //

      // etc
      (string(prefix_ + nbThread_opt).c_str(),                     //
       value<size_t>(&this->encoderParams_.nbThread_),             //
       "Number of thread used for parallel processing")            //
      (string(prefix_ + keepIntermediateFiles_opt).c_str(),        //
       value<bool>(&this->encoderParams_.keepIntermediateFiles_),  //
       "Keep intermediate files: RGB, YUV and bin")                //
      (string(prefix_ + absoluteD1_opt).c_str(),                   //
       value<bool>(&this->encoderParams_.absoluteD1_),             //
       "Absolute D1")                                              //
      (string(prefix_ + absoluteT1_opt).c_str(),                   //
       value<bool>(&this->encoderParams_.absoluteT1_),             //
       "Absolute T1")                                              //
      (string(prefix_ + multipleStreams_opt).c_str(),              //
       value<bool>(&this->encoderParams_.multipleStreams_),        //

       // 0. absolute 1 delta
       "number of video(geometry and attribute) streams")   //
      (string(prefix_ + deltaQPD0_opt).c_str(),             //
       value<int>(&this->encoderParams_.deltaQPD0_),        //
       "qp adjustment for geometry0 video: 0, +3, -3...")   //
      (string(prefix_ + deltaQPD1_opt).c_str(),             //
       value<int>(&this->encoderParams_.deltaQPD1_),        //
       "qp adjustment for geometry1 video: 0, +3, -3...")   //
      (string(prefix_ + deltaQPT0_opt).c_str(),             //
       value<int>(&this->encoderParams_.deltaQPT0_),        //
       "qp adjustment for attribute0 video: 0, +3, -3...")  //
      (string(prefix_ + deltaQPT1_opt).c_str(),             //
       value<int>(&this->encoderParams_.deltaQPT1_),        //
       "qp adjustment for attribute1 video: 0, +3, -3...")  //

      (string(prefix_ + constrainedPack_opt).c_str(),                    //
       value<bool>(&this->encoderParams_.constrainedPack_),              //
       "Temporally consistent patch packing")                            //
      (string(prefix_ + levelOfDetailX_opt).c_str(),                     //
       value<size_t>(&this->encoderParams_.levelOfDetailX_),             //
       "levelOfDetail : X axis in 2D space (should be greater than 1)")  //
      (string(prefix_ + levelOfDetailY_opt).c_str(),                     //
       value<size_t>(&this->encoderParams_.levelOfDetailY_),             //
       "levelOfDetail : Y axis in 2D space (should be greater than 1)")  //
      (string(prefix_ + groupDilation_opt).c_str(),                      //
       value<bool>(&this->encoderParams_.groupDilation_),                //
       "Group Dilation")                                                 //

      // Lossy occupancy map coding
      (string(prefix_ + offsetLossyOM_opt).c_str(),                                                   //
       value<size_t>(&this->encoderParams_.offsetLossyOM_),                                           //
       "Value to be assigned to non-zero occupancy map positions (default=0)\n")                      //
      (string(prefix_ + thresholdLossyOM_opt).c_str(),                                                //
       value<size_t>(&this->encoderParams_.thresholdLossyOM_),                                        //
       "Threshold for converting non-binary occupancy map to binary (default=0)\n")                   //
      (string(prefix_ + prefilterLossyOM_opt).c_str(),                                                //
       value<bool>(&this->encoderParams_.prefilterLossyOM_),                                          //
       "Selects whether the occupany map is prefiltered before lossy compression (default=false)\n")  //

      // SHVC
      (string(prefix_ + shvcLayerIndex_opt).c_str(),                                                //
       value<size_t>(&this->encoderParams_.shvcLayerIndex_),                                        //
       "Decode Layer ID number using SHVC codec")                                                   //
      (string(prefix_ + shvcRateX_opt).c_str(),                                                     //
       value<size_t>(&this->encoderParams_.shvcRateX_),                                             //
       "SHVCRateX : reduce rate of each SHVC layer X axis in 2D space (should be greater than 1)")  //
      (string(prefix_ + shvcRateY_opt).c_str(),                                                     //
       value<size_t>(&this->encoderParams_.shvcRateY_),                                             //
       "SHVCRateY : reduce rate of each SHVC layer Y axis in 2D space (should be greater than 1)")  //

      // visual quality
      (string(prefix_ + patchColorSubsampling_opt).c_str(),             //
       value<bool>(&this->encoderParams_.patchColorSubsampling_),       //
       "Enable per patch color sub-sampling\n")                         //
      (string(prefix_ + maxNumRefAtalsList_opt).c_str(),                //
       value<size_t>(&this->encoderParams_.maxNumRefAtlasList_),        //
       "maximum Number of Reference Atlas Frame list, default: 1")      //
      (string(prefix_ + maxNumRefAtlasFrame_opt).c_str(),               //
       value<size_t>(&this->encoderParams_.maxNumRefAtlasFrame_),       //
       "maximum Number of Reference Atlas Frame per list, default: 1")  //
      (string(prefix_ + pointLocalReconstruction_opt).c_str(),          //
       value<bool>(&this->encoderParams_.pointLocalReconstruction_),    //
       "Use point local reconstruction")                                //
      (string(prefix_ + mapCountMinus1_opt).c_str(),                    //
       value<size_t>(&this->encoderParams_.mapCountMinus1_),            //
       "Numbers of layers (rename to maps?)")                           //
      (string(prefix_ + singleMapPixelInterleaving_opt).c_str(),        //
       value<bool>(&this->encoderParams_.singleMapPixelInterleaving_),  //
       "Use single layer pixel interleaving")                           //
      (string(prefix_ + removeDuplicatePoints_opt).c_str(),             //
       value<bool>(&this->encoderParams_.removeDuplicatePoints_),       //
       "Remove duplicate points( ")                                     //
      (string(prefix_ + surfaceSeparation_opt).c_str(),                 //
       value<bool>(&this->encoderParams_.surfaceSeparation_),           //
       "surface separation")                                            //

      // high gradient separation
      (string(prefix_ + highGradientSeparation_opt).c_str(),                             //
       value<bool>(&this->encoderParams_.highGradientSeparation_),                       //
       "Separate high gradient points from a patch")                                     //
      (string(prefix_ + minGradient_opt).c_str(),                                        //
       value<double>(&this->encoderParams_.minGradient_),                                //
       "Minimun gradient for a point to be separated")                                   //
      (string(prefix_ + minNumHighGradientPoints_opt).c_str(),                           //
       value<size_t>(&this->encoderParams_.minNumHighGradientPoints_),                   //
       "Minimum number of connected high gradient points to be separated from a patch")  //

      // flexible packing + safeguard + push-pull
      (string(prefix_ + packingStrategy_opt).c_str(),                                                   //
       value<size_t>(&this->encoderParams_.packingStrategy_),                                           //
       "Patches packing strategy(0: anchor packing, 1(default): flexible packing, 2: tetris packing)")  //
      (string(prefix_ + useEightOrientations_opt).c_str(),                                              //
       value<bool>(&this->encoderParams_.useEightOrientations_),                                        //
       "Allow either 2 orientations (0(default): NULL AND SWAP), or 8 orientation (1)\n")               //
      (string(prefix_ + safeGuardDistance_opt).c_str(),                                                 //
       value<size_t>(&this->encoderParams_.safeGuardDistance_),                                         //
       "Number of empty blocks that must exist between the patches (default=1)\n")                      //
      (string(prefix_ + attributeBGFill_opt).c_str(),                                                   //
       value<size_t>(&this->encoderParams_.attributeBGFill_),                                           //
       "Selects the background filling operation for attribute only (0: patch-edge extension, "         //
       "1(default): smoothed push-pull algorithm), 2: harmonic background filling ")                    //

      // lossy-raw-points patch
      (string(prefix_ + lossyRawPointsPatch_opt).c_str(),                                                    //
       value<bool>(&this->encoderParams_.lossyRawPointsPatch_),                                              //
       "Lossy raw points patch(0: no lossy raw points patch, 1: enable lossy raw points patch (default=0)")  //
      (string(prefix_ + minNormSumOfInvDist4MPSelection_opt).c_str(),                                        //
       value<double>(&this->encoderParams_.minNormSumOfInvDist4MPSelection_),                                //
       "Minimum normalized sum of inverse distance for raw points selection: "                               //
       "double value between 0.0 and 1.0 (default=0.35)")                                                    //
      (string(prefix_ + globalPatchAllocation_opt).c_str(),                                                  //
       value<int>(&this->encoderParams_.globalPatchAllocation_),                                             //
       "Global temporally consistent patch allocation."                                                      //
       "(0: anchor's packing method(default), 1: gpa algorithm, 2: gtp algorithm)")                          //
      (string(prefix_ + globalPackingStrategyGOF_opt).c_str(),                                               //
       value<int>(&this->encoderParams_.globalPackingStrategyGOF_),                                          //
       "Number of frames to pack globally (0:(entire GOF))")                                                 //
      (string(prefix_ + globalPackingStrategyReset_opt).c_str(),                                             //
       value<bool>(&this->encoderParams_.globalPackingStrategyReset_),                                       //
       "Remove the reference to the previous frame (0(default), 1)")                                         //
      (string(prefix_ + globalPackingStrategyThreshold_opt).c_str(),                                         //
       value<double>(&this->encoderParams_.globalPackingStrategyThreshold_),                                 //
       "matched patches area ratio threshold (decides if connections are valid or not, 0(default))")         //
      (string(prefix_ + patchPrecedenceOrder_opt).c_str(),                                                   //
       value<bool>(&this->encoderParams_.patchPrecedenceOrderFlag_),                                         //
       "Order of patches")                                                                                   //
      (string(prefix_ + lowDelayEncoding_opt).c_str(),                                                       //
       value<bool>(&this->encoderParams_.lowDelayEncoding_),                                                 //
       "Low Delay encoding (0(default): do nothing, 1: does not allow overlap of patches bounding boxes for low delay "
       "encoding)")                                                                                           //
      (string(prefix_ + geometryPadding_opt).c_str(),                                                         //
       value<size_t>(&this->encoderParams_.geometryPadding_),                                                 //
       "Selects the background filling operation for geometry (0: anchor, 1(default): 3D geometry padding)")  //
      (string(prefix_ + apply3dMotionCompensation_opt).c_str(),                                               //
       value<bool>(&this->encoderParams_.use3dmc_),                                                           //
       "Use auxilliary information for 3d motion compensation.(0: conventional video coding, 1: 3D motion "
       "compensated)")                                                                                             //
      (string(prefix_ + usePccRDO_opt).c_str(),                                                                    //
       value<bool>(&this->encoderParams_.usePccRDO_),                                                              //
       "Use HEVC PCC RDO optimization")                                                                            //
      (string(prefix_ + geometry3dCoordinatesBitdepth_opt).c_str(),                                                //
       value<size_t>(&this->encoderParams_.geometry3dCoordinatesBitdepth_),                                        //
       "Bit depth of geomtery 3D coordinates")                                                                     //
      (string(prefix_ + geometryNominal2dBitdepth_opt).c_str(),                                                    //
       value<size_t>(&this->encoderParams_.geometryNominal2dBitdepth_),                                            //
       "Bit depth of geometry 2D")                                                                                 //
      (string(prefix_ + nbPlrmMode_opt).c_str(),                                                                   //
       value<size_t>(&this->encoderParams_.plrlNumberOfModes_),                                                    //
       "Number of PLR mode")                                                                                       //
      (string(prefix_ + patchSize_opt).c_str(),                                                                    //
       value<size_t>(&this->encoderParams_.patchSize_),                                                            //
       "Size of Patch for PLR")                                                                                    //
      (string(prefix_ + enhancedProjectionPlane_opt).c_str(),                                                      //
       value<bool>(&this->encoderParams_.enhancedPP_),                                                             //
       "Use enhanced Projection Plane(0: OFF, 1: ON)")                                                             //
      (string(prefix_ + minWeightEPP_opt).c_str(),                                                                 //
       value<double>(&this->encoderParams_.minWeightEPP_),                                                         //
       "Minimum value")                                                                                            //
      (string(prefix_ + additionalProjectionPlaneMode_opt).c_str(),                                                //
       value<int>(&this->encoderParams_.additionalProjectionPlaneMode_),                                           //
       "additiona Projection Plane Mode 0:none 1:Y-Axis 2:X-Axis 3:Z-Axis 4:All-Axis 5:apply to portion ")         //
      (string(prefix_ + partialAdditionalProjectionPlane_opt).c_str(),                                             //
       value<double>(&this->encoderParams_.partialAdditionalProjectionPlane_),                                     //
       "The value determines the partial point cloud. It's available with only additionalProjectionPlaneMode(5)")  //
      (string(prefix_ + numMaxTilePerFrame_opt).c_str(),                                                           //
       value<size_t>(&this->encoderParams_.numMaxTilePerFrame_),                                                   //
       "number of maximum tiles in a frame")                                                                       //
      (string(prefix_ + uniformPartitionSpacing_opt).c_str(),                                                      //
       value<bool>(&this->encoderParams_.uniformPartitionSpacing_),                                                //
       "indictation of uniform partitioning")                                                                      //
      (string(prefix_ + tilePartitionWidth_opt).c_str(),                                                           //
       value<size_t>(&this->encoderParams_.tilePartitionWidth_),                                                   //
       "uniform partition width in the unit of 64 pixels")                                                         //
      (string(prefix_ + tilePartitionHeight_opt).c_str(),                                                          //
       value<size_t>(&this->encoderParams_.tilePartitionHeight_),                                                  //
       "uniform partition height in the unit of 64 pixels")                                                        //
      (string(prefix_ + tilePartitionWidthList_opt).c_str(),                                                       //
       value<vector<int32_t>>(&this->encoderParams_.tilePartitionWidthList_),                                      //
       "non uniform partition width in the unit of 64 pixels")                                                     //
      (string(prefix_ + tilePartitionHeightList_opt).c_str(),                                                      //
       value<vector<int32_t>>(&this->encoderParams_.tilePartitionHeightList_),                                     //
       "non uniform partition height in the unit of 64 pixels")                                                    //
      (string(prefix_ + tileSegmentationType_opt).c_str(),                                                         //
       value<size_t>(&this->encoderParams_.tileSegmentationType_),                                                 //
       "tile segmentaton method : 0.no tile partition 1. 3D ROI based 2.2D Patch size based ")                     //

      // Point cloud partitions (ROIs) and tiles (m47804 CE2.19)
      (string(prefix_ + enablePointCloudPartitioning_opt).c_str(),        //
       value<bool>(&this->encoderParams_.enablePointCloudPartitioning_),  //
       " ")                                                               //
      (string(prefix_ + roiBoundingBoxMinX_opt).c_str(),                  //
       value<vector<int>>(&this->encoderParams_.roiBoundingBoxMinX_),     //
       " ")                                                               //
      (string(prefix_ + roiBoundingBoxMaxX_opt).c_str(),                  //
       value<vector<int>>(&this->encoderParams_.roiBoundingBoxMaxX_),     //
       " ")                                                               //
      (string(prefix_ + roiBoundingBoxMinY_opt).c_str(),                  //
       value<vector<int>>(&this->encoderParams_.roiBoundingBoxMinY_),     //
       " ")                                                               //
      (string(prefix_ + roiBoundingBoxMaxY_opt).c_str(),                  //
       value<vector<int>>(&this->encoderParams_.roiBoundingBoxMaxY_),     //
       " ")                                                               //
      (string(prefix_ + roiBoundingBoxMinZ_opt).c_str(),                  //
       value<vector<int>>(&this->encoderParams_.roiBoundingBoxMinZ_),     //
       " ")                                                               //
      (string(prefix_ + roiBoundingBoxMaxZ_opt).c_str(),                  //
       value<vector<int>>(&this->encoderParams_.roiBoundingBoxMaxZ_),     //
       " ")                                                               //
      (string(prefix_ + numTilesHor_opt).c_str(),                         //
       value<int>(&this->encoderParams_.numTilesHor_),                    //
       " ")                                                               //
      (string(prefix_ + tileHeightToWidthRatio_opt).c_str(),              //
       value<double>(&this->encoderParams_.tileHeightToWidthRatio_),      //
       " ")                                                               //
      (string(prefix_ + numCutsAlong1stLongestAxis_opt).c_str(),          //
       value<int>(&this->encoderParams_.numCutsAlong1stLongestAxis_),     //
       " ")                                                               //
      (string(prefix_ + numCutsAlong2ndLongestAxis_opt).c_str(),          //
       value<int>(&this->encoderParams_.numCutsAlong2ndLongestAxis_),     //
       " ")                                                               //
      (string(prefix_ + numCutsAlong3rdLongestAxis_opt).c_str(),          //
       value<int>(&this->encoderParams_.numCutsAlong3rdLongestAxis_),     //
       " ")                                                               //

      // Sort raw points by Morton code (m49363 CE2.25)
      (string(prefix_ + mortonOrderSortRawPoints_opt).c_str(),        //
       value<bool>(&this->encoderParams_.mortonOrderSortRawPoints_),  //
       " ")                                                           //

      // Patch block filtering
      (string(prefix_ + pbfEnableFlag_opt).c_str(),                     //
       value<bool>(&this->encoderParams_.pbfEnableFlag_),               //
       " enable patch block filtering ")                                //
      (string(prefix_ + pbfFilterSize_opt).c_str(),                     //
       value<int16_t>(&this->encoderParams_.pbfFilterSize_),            //
       "pbfFilterSize ")                                                //
      (string(prefix_ + pbfPassesCount_opt).c_str(),                    //
       value<int16_t>(&this->encoderParams_.pbfPassesCount_),           //
       "pbfPassesCount ")                                               //
      (string(prefix_ + pbfLog2Threshold_opt).c_str(),                  //
       value<int16_t>(&this->encoderParams_.pbfLog2Threshold_),         //
       "pbfLog2Threshold ")                                             //
      (string(prefix_ + tierFlag_opt).c_str(),                          //
       value<bool>(&this->encoderParams_.tierFlag_),                    //
       "Tier Flag")                                                     //
      (string(prefix_ + profileCodecGroupIdc_opt).c_str(),              //
       value<size_t>(&this->encoderParams_.profileCodecGroupIdc_),      //
       "Profile Codec Group Idc")                                       //
      (string(prefix_ + profileToolsetIdc_opt).c_str(),                 //
       value<size_t>(&this->encoderParams_.profileToolsetIdc_),         //
       "Profile Toolset Idc")                                           //
      (string(prefix_ + profileReconstructionIdc_opt).c_str(),          //
       value<size_t>(&this->encoderParams_.profileReconstructionIdc_),  //
       "Profile Reconstruction Idc")                                    //
      (string(prefix_ + levelIdc_opt).c_str(),                          //
       value<size_t>(&this->encoderParams_.levelIdc_),                  //
       "Level Idc")                                                     //

      // ajt0526: avcCodecIdIndex/hevcCodecIdIndex/vvcCodecIdIndex when using CCM SEI and
      // profileCodecGroupIdc beeds to correspond to mp4RA?
      (string(prefix_ + avcCodecIdIndex_opt).c_str(),           //
       value<size_t>(&this->encoderParams_.avcCodecIdIndex_),   //
       "index for avc codec ")                                  //
      (string(prefix_ + hevcCodecIdIndex_opt).c_str(),          //
       value<size_t>(&this->encoderParams_.hevcCodecIdIndex_),  //
       "index for hevc codec ")                                 //
      (string(prefix_ + shvcCodecIdIndex_opt).c_str(),          //
       value<size_t>(&this->encoderParams_.shvcCodecIdIndex_),  //
       "index for shvc codec ")                                 //
      (string(prefix_ + vvcCodecIdIndex_opt).c_str(),           //
       value<size_t>(&this->encoderParams_.vvcCodecIdIndex_),   //
       "index for vvc codec ")                                  //

      // 8.3.4.6	Profile toolset constraints information syntax
      (string(prefix_ + oneV3CFrameOnlyFlag_opt).c_str(),        //
       value<bool>(&this->encoderParams_.oneV3CFrameOnlyFlag_),  //
       "One V3C Frame Only Flag")                                //
                                                                 // TODO
      ;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC2Parameter::setDefault() {
  encoderParams_.patchColorSubsampling_ = false;
  // TODO
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC2Parameter::notify() {}

//////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& out, const JPCCEncoderTMC2Parameter& obj) {
  obj.coutParameters(out)                                                                                         //
      (configurationFolder_opt, obj.encoderParams_.configurationFolder_)                                          //
      (uncompressedDataFolder_opt, obj.encoderParams_.uncompressedDataFolder_)                                    //
      (uncompressedDataPath_opt, obj.encoderParams_.uncompressedDataPath_)                                        //
      (compressedStreamPath_opt, obj.encoderParams_.compressedStreamPath_)                                        //
      (reconstructedDataPath_opt, obj.encoderParams_.reconstructedDataPath_)                                      //
      (forcedSsvhUnitSizePrecisionBytes_opt, obj.encoderParams_.forcedSsvhUnitSizePrecisionBytes_)                //
      (startFrameNumber_opt, obj.encoderParams_.startFrameNumber_)                                                //
      (frameCount_opt, obj.encoderParams_.frameCount_)                                                            //
      (groupOfFramesSize_opt, obj.encoderParams_.groupOfFramesSize_)                                              //
      (colorTransform_opt, obj.encoderParams_.colorTransform_)                                                    //
      (colorSpaceConversionPath_opt, obj.encoderParams_.colorSpaceConversionPath_)                                //
      (colorSpaceConversionConfig_opt, obj.encoderParams_.colorSpaceConversionConfig_)                            //
      (inverseColorSpaceConversionConfig_opt, obj.encoderParams_.inverseColorSpaceConversionConfig_)              //
      (gridBasedSegmentation_opt, obj.encoderParams_.gridBasedSegmentation_)                                      //
      (voxelDimensionGridBasedSegmentation_opt, obj.encoderParams_.voxelDimensionGridBasedSegmentation_)          //
      (nnNormalEstimation_opt, obj.encoderParams_.nnNormalEstimation_)                                            //
      (normalOrientation_opt, obj.encoderParams_.normalOrientation_)                                              //
      (gridBasedRefineSegmentation_opt, obj.encoderParams_.gridBasedRefineSegmentation_)                          //
      (maxNNCountRefineSegmentation_opt, obj.encoderParams_.maxNNCountRefineSegmentation_)                        //
      (iterationCountRefineSegmentation_opt, obj.encoderParams_.iterationCountRefineSegmentation_)                //
      (voxelDimensionRefineSegmentation_opt, obj.encoderParams_.voxelDimensionRefineSegmentation_)                //
      (searchRadiusRefineSegmentation_opt, obj.encoderParams_.searchRadiusRefineSegmentation_)                    //
      (occupancyResolution_opt, obj.encoderParams_.occupancyResolution_)                                          //
      (enablePatchSplitting_opt, obj.encoderParams_.enablePatchSplitting_)                                        //
      (maxPatchSize_opt, obj.encoderParams_.maxPatchSize_)                                                        //
      (log2QuantizerSizeX_opt, obj.encoderParams_.log2QuantizerSizeX_)                                            //
      (log2QuantizerSizeY_opt, obj.encoderParams_.log2QuantizerSizeY_)                                            //
      (minPointCountPerCCPatchSegmentation_opt, obj.encoderParams_.minPointCountPerCCPatchSegmentation_)          //
      (maxNNCountPatchSegmentation_opt, obj.encoderParams_.maxNNCountPatchSegmentation_)                          //
      (surfaceThickness_opt, obj.encoderParams_.surfaceThickness_)                                                //
      (depthQuantizationStep_opt, obj.encoderParams_.minLevel_)                                                   //
      (maxAllowedDist2RawPointsDetection_opt, obj.encoderParams_.maxAllowedDist2RawPointsDetection_)              //
      (maxAllowedDist2RawPointsSelection_opt, obj.encoderParams_.maxAllowedDist2RawPointsSelection_)              //
      (lambdaRefineSegmentation_opt, obj.encoderParams_.lambdaRefineSegmentation_)                                //
      (minimumImageWidth_opt, obj.encoderParams_.minimumImageWidth_)                                              //
      (minimumImageHeight_opt, obj.encoderParams_.minimumImageHeight_)                                            //
      (maxCandidateCount_opt, obj.encoderParams_.maxCandidateCount_)                                              //
      (occupancyPrecision_opt, obj.encoderParams_.occupancyPrecision_)                                            //
      (occupancyMapConfig_opt, obj.encoderParams_.occupancyMapConfig_)                                            //
      (occupancyMapQP_opt, obj.encoderParams_.occupancyMapQP_)                                                    //
      (enhancedOccupancyMapCode_opt, obj.encoderParams_.enhancedOccupancyMapCode_)                                //
      (EOMFixBitCount_opt, obj.encoderParams_.EOMFixBitCount_)                                                    //
      (occupancyMapRefinement_opt, obj.encoderParams_.occupancyMapRefinement_)                                    //
      (decodedAtlasInformationHash_opt, obj.encoderParams_.decodedAtlasInformationHash_)                          //
      (attributeTransferFilterType_opt, obj.encoderParams_.attrTransferFilterType_)                               //
      (flagGeometrySmoothing_opt, obj.encoderParams_.flagGeometrySmoothing_)                                      //
      (neighborCountSmoothing_opt, obj.encoderParams_.neighborCountSmoothing_)                                    //
      (radius2Smoothing_opt, obj.encoderParams_.radius2Smoothing_)                                                //
      (radius2BoundaryDetection_opt, obj.encoderParams_.radius2BoundaryDetection_)                                //
      (thresholdSmoothing_opt, obj.encoderParams_.thresholdSmoothing_)                                            //
      (patchExpansion_opt, obj.encoderParams_.patchExpansion_)                                                    //
      (gridSmoothing_opt, obj.encoderParams_.gridSmoothing_)                                                      //
      (gridSize_opt, obj.encoderParams_.gridSize_)                                                                //
      (thresholdColorSmoothing_opt, obj.encoderParams_.thresholdColorSmoothing_)                                  //
      (cgridSize_opt, obj.encoderParams_.cgridSize_)                                                              //
      (thresholdColorDifference_opt, obj.encoderParams_.thresholdColorDifference_)                                //
      (thresholdColorVariation_opt, obj.encoderParams_.thresholdColorVariation_)                                  //
      (flagColorSmoothing_opt, obj.encoderParams_.flagColorSmoothing_)                                            //
      (thresholdColorPreSmoothing_opt, obj.encoderParams_.thresholdColorPreSmoothing_)                            //
      (thresholdColorPreSmoothingLocalEntropy_opt, obj.encoderParams_.thresholdColorPreSmoothingLocalEntropy_)    //
      (radius2ColorPreSmoothing_opt, obj.encoderParams_.radius2ColorPreSmoothing_)                                //
      (neighborCountColorPreSmoothing_opt, obj.encoderParams_.neighborCountColorPreSmoothing_)                    //
      (flagColorPreSmoothing_opt, obj.encoderParams_.flagColorPreSmoothing_)                                      //
      (bestColorSearchRange_opt, obj.encoderParams_.bestColorSearchRange_)                                        //
      (numNeighborsColorTransferFwd_opt, obj.encoderParams_.numNeighborsColorTransferFwd_)                        //
      (numNeighborsColorTransferBwd_opt, obj.encoderParams_.numNeighborsColorTransferBwd_)                        //
      (useDistWeightedAverageFwd_opt, obj.encoderParams_.useDistWeightedAverageFwd_)                              //
      (useDistWeightedAverageBwd_opt, obj.encoderParams_.useDistWeightedAverageBwd_)                              //
      (skipAvgIfIdenticalSourcePointPresentFwd_opt, obj.encoderParams_.skipAvgIfIdenticalSourcePointPresentFwd_)  //
      (skipAvgIfIdenticalSourcePointPresentBwd_opt, obj.encoderParams_.skipAvgIfIdenticalSourcePointPresentBwd_)  //
      (distOffsetFwd_opt, obj.encoderParams_.distOffsetFwd_)                                                      //
      (distOffsetBwd_opt, obj.encoderParams_.distOffsetBwd_)                                                      //
      (maxGeometryDist2Fwd_opt, obj.encoderParams_.maxGeometryDist2Fwd_)                                          //
      (maxGeometryDist2Bwd_opt, obj.encoderParams_.maxGeometryDist2Bwd_)                                          //
      (maxColorDist2Fwd_opt, obj.encoderParams_.maxColorDist2Fwd_)                                                //
      (maxColorDist2Bwd_opt, obj.encoderParams_.maxColorDist2Bwd_)                                                //
      (excludeColorOutlier_opt, obj.encoderParams_.excludeColorOutlier_)                                          //
      (thresholdColorOutlierDist_opt, obj.encoderParams_.thresholdColorOutlierDist_)                              //
      (videoEncoderOccupancyPath_opt, obj.encoderParams_.videoEncoderOccupancyPath_)                              //
      (videoEncoderGeometryPath_opt, obj.encoderParams_.videoEncoderGeometryPath_)                                //
      (videoEncoderAttributePath_opt, obj.encoderParams_.videoEncoderAttributePath_)                              //
      (videoEncoderOccupancyCodecId_opt, obj.encoderParams_.videoEncoderOccupancyCodecId_)                        //
      (videoEncoderGeometryCodecId_opt, obj.encoderParams_.videoEncoderGeometryCodecId_)                          //
      (videoEncoderAttributeCodecId_opt, obj.encoderParams_.videoEncoderAttributeCodecId_)                        //
      (byteStreamVideoEncoderOccupancy_opt, obj.encoderParams_.byteStreamVideoCoderOccupancy_)                    //
      (byteStreamVideoEncoderGeometry_opt, obj.encoderParams_.byteStreamVideoCoderGeometry_)                      //
      (byteStreamVideoEncoderAttribute_opt, obj.encoderParams_.byteStreamVideoCoderAttribute_)                    //
      (geometryQP_opt, obj.encoderParams_.geometryQP_)                                                            //
      (attributeQP_opt, obj.encoderParams_.attributeQP_)                                                          //
      (auxGeometryQP_opt, obj.encoderParams_.auxGeometryQP_)                                                      //
      (auxAttributeQP_opt, obj.encoderParams_.auxAttributeQP_)                                                    //
      (geometryConfig_opt, obj.encoderParams_.geometryConfig_)                                                    //
      (geometry0Config_opt, obj.encoderParams_.geometry0Config_)                                                  //
      (geometry1Config_opt, obj.encoderParams_.geometry1Config_)                                                  //
      (attributeConfig_opt, obj.encoderParams_.attributeConfig_)                                                  //
      (attribute0Config_opt, obj.encoderParams_.attribute0Config_)                                                //
      (attribute1Config_opt, obj.encoderParams_.attribute1Config_)                                                //
      (rawPointsPatch_opt, obj.encoderParams_.rawPointsPatch_)                                                    //
      (noAttributes_opt, obj.encoderParams_.noAttributes_)                                                        //
      (attributeVideo444_opt, obj.encoderParams_.attributeVideo444_)                                              //
      (useRawPointsSeparateVideo_opt, obj.encoderParams_.useRawPointsSeparateVideo_)                              //
      (attributeRawSeparateVideoWidth_opt, obj.encoderParams_.attributeRawSeparateVideoWidth_)                    //
      (geometryMPConfig_opt, obj.encoderParams_.geometryAuxVideoConfig_)                                          //
      (attributeMPConfig_opt, obj.encoderParams_.attributeAuxVideoConfig_)                                        //
      (nbThread_opt, obj.encoderParams_.nbThread_)                                                                //
      (keepIntermediateFiles_opt, obj.encoderParams_.keepIntermediateFiles_)                                      //
      (absoluteD1_opt, obj.encoderParams_.absoluteD1_)                                                            //
      (absoluteT1_opt, obj.encoderParams_.absoluteT1_)                                                            //
      (multipleStreams_opt, obj.encoderParams_.multipleStreams_)                                                  //
      (deltaQPD0_opt, obj.encoderParams_.deltaQPD0_)                                                              //
      (deltaQPD1_opt, obj.encoderParams_.deltaQPD1_)                                                              //
      (deltaQPT0_opt, obj.encoderParams_.deltaQPT0_)                                                              //
      (deltaQPT1_opt, obj.encoderParams_.deltaQPT1_)                                                              //
      (constrainedPack_opt, obj.encoderParams_.constrainedPack_)                                                  //
      (levelOfDetailX_opt, obj.encoderParams_.levelOfDetailX_)                                                    //
      (levelOfDetailY_opt, obj.encoderParams_.levelOfDetailY_)                                                    //
      (groupDilation_opt, obj.encoderParams_.groupDilation_)                                                      //
      (offsetLossyOM_opt, obj.encoderParams_.offsetLossyOM_)                                                      //
      (thresholdLossyOM_opt, obj.encoderParams_.thresholdLossyOM_)                                                //
      (prefilterLossyOM_opt, obj.encoderParams_.prefilterLossyOM_)                                                //
      (shvcLayerIndex_opt, obj.encoderParams_.shvcLayerIndex_)                                                    //
      (shvcRateX_opt, obj.encoderParams_.shvcRateX_)                                                              //
      (shvcRateY_opt, obj.encoderParams_.shvcRateY_)                                                              //
      (patchColorSubsampling_opt, obj.encoderParams_.patchColorSubsampling_)                                      //
      (maxNumRefAtalsList_opt, obj.encoderParams_.maxNumRefAtlasList_)                                            //
      (maxNumRefAtlasFrame_opt, obj.encoderParams_.maxNumRefAtlasFrame_)                                          //
      (pointLocalReconstruction_opt, obj.encoderParams_.pointLocalReconstruction_)                                //
      (mapCountMinus1_opt, obj.encoderParams_.mapCountMinus1_)                                                    //
      (singleMapPixelInterleaving_opt, obj.encoderParams_.singleMapPixelInterleaving_)                            //
      (removeDuplicatePoints_opt, obj.encoderParams_.removeDuplicatePoints_)                                      //
      (surfaceSeparation_opt, obj.encoderParams_.surfaceSeparation_)                                              //
      (highGradientSeparation_opt, obj.encoderParams_.highGradientSeparation_)                                    //
      (minGradient_opt, obj.encoderParams_.minGradient_)                                                          //
      (minNumHighGradientPoints_opt, obj.encoderParams_.minNumHighGradientPoints_)                                //
      (packingStrategy_opt, obj.encoderParams_.packingStrategy_)                                                  //
      (useEightOrientations_opt, obj.encoderParams_.useEightOrientations_)                                        //
      (safeGuardDistance_opt, obj.encoderParams_.safeGuardDistance_)                                              //
      (attributeBGFill_opt, obj.encoderParams_.attributeBGFill_)                                                  //
      (lossyRawPointsPatch_opt, obj.encoderParams_.lossyRawPointsPatch_)                                          //
      (minNormSumOfInvDist4MPSelection_opt, obj.encoderParams_.minNormSumOfInvDist4MPSelection_)                  //
      (globalPatchAllocation_opt, obj.encoderParams_.globalPatchAllocation_)                                      //
      (globalPackingStrategyGOF_opt, obj.encoderParams_.globalPackingStrategyGOF_)                                //
      (globalPackingStrategyReset_opt, obj.encoderParams_.globalPackingStrategyReset_)                            //
      (globalPackingStrategyThreshold_opt, obj.encoderParams_.globalPackingStrategyThreshold_)                    //
      (patchPrecedenceOrder_opt, obj.encoderParams_.patchPrecedenceOrderFlag_)                                    //
      (lowDelayEncoding_opt, obj.encoderParams_.lowDelayEncoding_)                                                //
      (geometryPadding_opt, obj.encoderParams_.geometryPadding_)                                                  //
      (apply3dMotionCompensation_opt, obj.encoderParams_.use3dmc_)                                                //
      (usePccRDO_opt, obj.encoderParams_.usePccRDO_)                                                              //
      (geometry3dCoordinatesBitdepth_opt, obj.encoderParams_.geometry3dCoordinatesBitdepth_)                      //
      (geometryNominal2dBitdepth_opt, obj.encoderParams_.geometryNominal2dBitdepth_)                              //
      (nbPlrmMode_opt, obj.encoderParams_.plrlNumberOfModes_)                                                     //
      (patchSize_opt, obj.encoderParams_.patchSize_)                                                              //
      (enhancedProjectionPlane_opt, obj.encoderParams_.enhancedPP_)                                               //
      (minWeightEPP_opt, obj.encoderParams_.minWeightEPP_)                                                        //
      (additionalProjectionPlaneMode_opt, obj.encoderParams_.additionalProjectionPlaneMode_)                      //
      (partialAdditionalProjectionPlane_opt, obj.encoderParams_.partialAdditionalProjectionPlane_)                //
      (numMaxTilePerFrame_opt, obj.encoderParams_.numMaxTilePerFrame_)                                            //
      (uniformPartitionSpacing_opt, obj.encoderParams_.uniformPartitionSpacing_)                                  //
      (tilePartitionWidth_opt, obj.encoderParams_.tilePartitionWidth_)                                            //
      (tilePartitionHeight_opt, obj.encoderParams_.tilePartitionHeight_)                                          //
      (tilePartitionWidthList_opt, obj.encoderParams_.tilePartitionWidthList_)                                    //
      (tilePartitionHeightList_opt, obj.encoderParams_.tilePartitionHeightList_)                                  //
      (tileSegmentationType_opt, obj.encoderParams_.tileSegmentationType_)                                        //
      (enablePointCloudPartitioning_opt, obj.encoderParams_.enablePointCloudPartitioning_)                        //
      (roiBoundingBoxMinX_opt, obj.encoderParams_.roiBoundingBoxMinX_)                                            //
      (roiBoundingBoxMaxX_opt, obj.encoderParams_.roiBoundingBoxMaxX_)                                            //
      (roiBoundingBoxMinY_opt, obj.encoderParams_.roiBoundingBoxMinY_)                                            //
      (roiBoundingBoxMaxY_opt, obj.encoderParams_.roiBoundingBoxMaxY_)                                            //
      (roiBoundingBoxMinZ_opt, obj.encoderParams_.roiBoundingBoxMinZ_)                                            //
      (roiBoundingBoxMaxZ_opt, obj.encoderParams_.roiBoundingBoxMaxZ_)                                            //
      (numTilesHor_opt, obj.encoderParams_.numTilesHor_)                                                          //
      (tileHeightToWidthRatio_opt, obj.encoderParams_.tileHeightToWidthRatio_)                                    //
      (numCutsAlong1stLongestAxis_opt, obj.encoderParams_.numCutsAlong1stLongestAxis_)                            //
      (numCutsAlong2ndLongestAxis_opt, obj.encoderParams_.numCutsAlong2ndLongestAxis_)                            //
      (numCutsAlong3rdLongestAxis_opt, obj.encoderParams_.numCutsAlong3rdLongestAxis_)                            //
      (mortonOrderSortRawPoints_opt, obj.encoderParams_.mortonOrderSortRawPoints_)                                //
      (pbfEnableFlag_opt, obj.encoderParams_.pbfEnableFlag_)                                                      //
      (pbfFilterSize_opt, obj.encoderParams_.pbfFilterSize_)                                                      //
      (pbfPassesCount_opt, obj.encoderParams_.pbfPassesCount_)                                                    //
      (pbfLog2Threshold_opt, obj.encoderParams_.pbfLog2Threshold_)                                                //
      (tierFlag_opt, obj.encoderParams_.tierFlag_)                                                                //
      (profileCodecGroupIdc_opt, obj.encoderParams_.profileCodecGroupIdc_)                                        //
      (profileToolsetIdc_opt, obj.encoderParams_.profileToolsetIdc_)                                              //
      (profileReconstructionIdc_opt, obj.encoderParams_.profileReconstructionIdc_)                                //
      (levelIdc_opt, obj.encoderParams_.levelIdc_)                                                                //
      (avcCodecIdIndex_opt, obj.encoderParams_.avcCodecIdIndex_)                                                  //
      (hevcCodecIdIndex_opt, obj.encoderParams_.hevcCodecIdIndex_)                                                //
      (shvcCodecIdIndex_opt, obj.encoderParams_.shvcCodecIdIndex_)                                                //
      (vvcCodecIdIndex_opt, obj.encoderParams_.vvcCodecIdIndex_)                                                  //
      (oneV3CFrameOnlyFlag_opt, obj.encoderParams_.oneV3CFrameOnlyFlag_)                                          //
      ;
  return out;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::istream& operator>>(std::istream& in, PCCColorTransform& val) {
  unsigned int tmp;
  in >> tmp;
  val = PCCColorTransform(tmp);
  return in;
}
std::istream& operator>>(std::istream& in, PCCCodecId& val) {
  val = UNKNOWN_CODEC;
  std::string tmp;
  in >> tmp;
  if (tmp == "JMAPP" || tmp == "jmapp") { val = JMAPP; }
  if (tmp == "HMAPP" || tmp == "hmapp") { val = HMAPP; }
  if (tmp == "SHMAPP" || tmp == "shmapp" || tmp == "shm" || tmp == "shm2" || tmp == "shm3") { val = SHMAPP; }
  if (tmp == "JMLIB" || tmp == "jmlib" || tmp == "jm" || tmp == "avc") { val = JMLIB; }
  if (tmp == "HMLIB" || tmp == "hmlib" || tmp == "hm" || tmp == "hevc") { val = HMLIB; }
  if (tmp == "VTMLIB" || tmp == "vtmlib" || tmp == "vtm" || tmp == "vvc") { val = VTMLIB; }
  if (tmp == "FFMPEG" || tmp == "ffmpeg") { val = FFMPEG; }
  if (val == UNKNOWN_CODEC) { fprintf(stderr, "PCCCodecId could not be indentified from: \"%s\" \n", tmp.c_str()); }
  return in;
}
std::ostream& operator<<(std::ostream& out, const PCCColorTransform& val) {
  switch (val) {
    case PCCColorTransform::COLOR_TRANSFORM_NONE: out << int(val) << " (none)"; break;
    case PCCColorTransform::COLOR_TRANSFORM_RGB_TO_YCBCR: out << int(val) << " (RGB to YCbCr (Rec.709))"; break;
    default: out << int(val) << " (Unknown)"; break;
  }
  return out;
}
std::ostream& operator<<(std::ostream& out, const PCCCodecId& val) {
  switch (val) {
    case PCCCodecId::JMAPP: out << int(val) << "(JMAPP)"; break;
    case PCCCodecId::HMAPP: out << int(val) << "(HMAPP)"; break;
    case PCCCodecId::SHMAPP: out << int(val) << "(SHMAPP)"; break;
    case PCCCodecId::JMLIB: out << int(val) << "(JMLIB)"; break;
    case PCCCodecId::HMLIB: out << int(val) << "(HMLIB)"; break;
    case PCCCodecId::VTMLIB: out << int(val) << "(VTMLIB)"; break;
    case PCCCodecId::FFMPEG: out << int(val) << "(FFMPEG)"; break;
    default: out << int(val) << " (Unknown)"; break;
  }
  return out;
}

}  // namespace jpcc::encoder
