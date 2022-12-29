#include <jpcc/encoder/JPCCEncoderTMC2Parameter.h>

#include <PCCEncoderParameters.h>

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

using namespace std;
using namespace pcc;

//////////////////////////////////////////////////////////////////////////////////////////////
namespace pcc {
static std::istream& operator>>(std::istream& in, PCCColorTransform& val);
static std::istream& operator>>(std::istream& in, PCCCodecId& val);
static std::ostream& operator<<(std::ostream& out, const PCCColorTransform& val);
static std::ostream& operator<<(std::ostream& out, const PCCCodecId& val);
}  // namespace pcc

namespace jpcc::encoder {

using namespace po;

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC2Parameter::JPCCEncoderTMC2Parameter() :
    JPCCEncoderTMC2Parameter(JPCC_ENCODER_TMC2_OPT_PREFIX, __FUNCTION__) {}

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC2Parameter::JPCCEncoderTMC2Parameter(  // NOLINT(cppcoreguidelines-pro-type-member-init)
    const string& prefix,
    const string& caption) :
    Parameter(prefix, caption) {
  auto _params = std::make_shared<pcc::PCCEncoderParameters>();
  opts_.add_options()                                                                     //
      (string(prefix_ + configurationFolder_opt).c_str(),                                 //
       value<string>(&_params->configurationFolder_),                                     //
       "Folder where the configuration files are stored,use for cfg relative paths.")     //
      (string(prefix_ + uncompressedDataFolder_opt).c_str(),                              //
       value<string>(&_params->uncompressedDataFolder_),                                  //
       "Folder where the uncompress input data are stored, use for cfg relative paths.")  //

      // i/o
      (string(prefix_ + uncompressedDataPath_opt).c_str(),                              //
       value<string>(&_params->uncompressedDataPath_),                                  //
       "Input pointcloud to encode. Multi-frame sequences may be represented by %04i")  //
      (string(prefix_ + compressedStreamPath_opt).c_str(),                              //
       value<string>(&_params->compressedStreamPath_),                                  //
       "Output(encoder)/Input(decoder) compressed bitstream")                           //
      (string(prefix_ + reconstructedDataPath_opt).c_str(),                             //
       value<string>(&_params->reconstructedDataPath_),                                 //
       "Output decoded pointcloud. Multi-frame sequences may be represented by %04i")   //
      (string(prefix_ + forcedSsvhUnitSizePrecisionBytes_opt).c_str(),                  //
       value<uint32_t>(&_params->forcedSsvhUnitSizePrecisionBytes_),                    //
       "forced SSVH unit size precision bytes")                                         //

      // sequence configuration
      (string(prefix_ + startFrameNumber_opt).c_str(),     //
       value<size_t>(&_params->startFrameNumber_),         //
       "First frame number in sequence to encode/decode")  //
      (string(prefix_ + frameCount_opt).c_str(),           //
       value<size_t>(&_params->frameCount_),               //
       "Number of frames to encode")                       //
      (string(prefix_ + groupOfFramesSize_opt).c_str(),    //
       value<size_t>(&_params->groupOfFramesSize_),        //
       "Random access period")                             //

      // colour space conversion
      (string(prefix_ + colorTransform_opt).c_str(),                                    //
       value<PCCColorTransform>(&_params->colorTransform_),                             //
       "The colour transform to be applied:\n  0: none\n  1: RGB to YCbCr (Rec.709)")   //
      (string(prefix_ + colorSpaceConversionPath_opt).c_str(),                          //
       value<string>(&_params->colorSpaceConversionPath_),                              //
       "Path to the HDRConvert. If unset, an internal color space conversion is used")  //
      (string(prefix_ + colorSpaceConversionConfig_opt).c_str(),                        //
       value<string>(&_params->colorSpaceConversionConfig_),                            //
       "HDRConvert configuration file used for RGB444 to YUV420 conversion")            //
      (string(prefix_ + inverseColorSpaceConversionConfig_opt).c_str(),                 //
       value<string>(&_params->inverseColorSpaceConversionConfig_),                     //
       "HDRConvert configuration file used for YUV420 to RGB444 conversion")            //

      // segmentation
      // If enabled, change refineSegmentationGridBased() parameters according to m56857"
      (string(prefix_ + gridBasedSegmentation_opt).c_str(),                                 //
       value<bool>(&_params->gridBasedSegmentation_),                                       //
       "Voxel dimension for grid-based segmentation (GBS)")                                 //
      (string(prefix_ + voxelDimensionGridBasedSegmentation_opt).c_str(),                   //
       value<size_t>(&_params->voxelDimensionGridBasedSegmentation_),                       //
       "Voxel dimension for grid-based segmentation (GBS)")                                 //
      (string(prefix_ + nnNormalEstimation_opt).c_str(),                                    //
       value<size_t>(&_params->nnNormalEstimation_),                                        //
       "Number of points used for normal estimation")                                       //
      (string(prefix_ + normalOrientation_opt).c_str(),                                     //
       value<size_t>(&_params->normalOrientation_),                                         //
       "Normal orientation: 0: None 1: spanning tree, 2:view point, 3:cubemap projection")  //
      (string(prefix_ + gridBasedRefineSegmentation_opt).c_str(),                           //
       value<bool>(&_params->gridBasedRefineSegmentation_),                                 //
       "Use grid-based approach for segmentation refinement")                               //
      (string(prefix_ + maxNNCountRefineSegmentation_opt).c_str(),                          //
       value<size_t>(&_params->maxNNCountRefineSegmentation_),                              //
       "Number of nearest neighbors used during segmentation refinement")                   //
      (string(prefix_ + iterationCountRefineSegmentation_opt).c_str(),                      //
       value<size_t>(&_params->iterationCountRefineSegmentation_),                          //
       "Number of iterations performed during segmentation refinement")                     //
      (string(prefix_ + voxelDimensionRefineSegmentation_opt).c_str(),                      //
       value<size_t>(&_params->voxelDimensionRefineSegmentation_),                          //
       "Voxel dimension for segmentation refinement (must be a power of 2)")                //
      (string(prefix_ + searchRadiusRefineSegmentation_opt).c_str(),                        //
       value<size_t>(&_params->searchRadiusRefineSegmentation_),                            //
       "Search radius for segmentation refinement")                                         //
      (string(prefix_ + occupancyResolution_opt).c_str(),                                   //
       value<size_t>(&_params->occupancyResolution_),                                       //
       "Resolution of packing block(a block contain only one patch)")                       //
      (string(prefix_ + enablePatchSplitting_opt).c_str(),                                  //
       value<bool>(&_params->enablePatchSplitting_),                                        //
       "Enable patch splitting")                                                            //
      (string(prefix_ + maxPatchSize_opt).c_str(),                                          //
       value<size_t>(&_params->maxPatchSize_),                                              //
       "Maximum patch size for segmentation")                                               //
      (string(prefix_ + log2QuantizerSizeX_opt).c_str(),                                    //
       value<size_t>(&_params->log2QuantizerSizeX_),                                        //
       "log2 of Quantization step for patch size X: 0. pixel precision 4.16 as before")     //
      (string(prefix_ + log2QuantizerSizeY_opt).c_str(),                                    //
       value<size_t>(&_params->log2QuantizerSizeY_),                                        //
       "log2 of Quantization step for patch size Y: 0. pixel precision 4.16 as before")     //
      (string(prefix_ + minPointCountPerCCPatchSegmentation_opt).c_str(),                   //
       value<size_t>(&_params->minPointCountPerCCPatchSegmentation_),                       //
       "Minimum number of points for a connected component to be retained as a patch")      //
      (string(prefix_ + maxNNCountPatchSegmentation_opt).c_str(),                           //
       value<size_t>(&_params->maxNNCountPatchSegmentation_),                               //
       "Number of nearest neighbors used during connected components extraction")           //
      (string(prefix_ + surfaceThickness_opt).c_str(),                                      //
       value<size_t>(&_params->surfaceThickness_),                                          //
       "Surface thickness")                                                                 //
      (string(prefix_ + depthQuantizationStep_opt).c_str(),                                 //
       value<size_t>(&_params->minLevel_),                                                  //
       "minimum level for patches")                                                         //
      (string(prefix_ + maxAllowedDist2RawPointsDetection_opt).c_str(),                     //
       value<double>(&_params->maxAllowedDist2RawPointsDetection_),                         //
       "Maximum distance for a point to be ignored during raw points detection")            //
      (string(prefix_ + maxAllowedDist2RawPointsSelection_opt).c_str(),                     //
       value<double>(&_params->maxAllowedDist2RawPointsSelection_),                         //
       "Maximum distance for a point to be ignored during  raw points  selection")          //
      (string(prefix_ + lambdaRefineSegmentation_opt).c_str(),                              //
       value<double>(&_params->lambdaRefineSegmentation_),                                  //
       "Controls the smoothness of the patch boundaries  during segmentation  refinement")  //

      // packing
      (string(prefix_ + minimumImageWidth_opt).c_str(),   //
       value<size_t>(&_params->minimumImageWidth_),       //
       "Minimum width of packed patch frame")             //
      (string(prefix_ + minimumImageHeight_opt).c_str(),  //
       value<size_t>(&_params->minimumImageHeight_),      //
       "Minimum height of packed patch frame")            //

      // occupancy map
      (string(prefix_ + maxCandidateCount_opt).c_str(),   //
       value<size_t>(&_params->maxCandidateCount_),       //
       "Maximum nuber of candidates in list L")           //
      (string(prefix_ + occupancyPrecision_opt).c_str(),  //
       value<size_t>(&_params->occupancyPrecision_),      //
       "Occupancy map B0 precision")                      //
      (string(prefix_ + occupancyMapConfig_opt).c_str(),  //
       value<string>(&_params->occupancyMapConfig_),      //
       "Occupancy map encoder config file")               //
      (string(prefix_ + occupancyMapQP_opt).c_str(),      //
       value<size_t>(&_params->occupancyMapQP_),          //
       "QP for compression of occupancy map video")       //

      // EOM code
      (string(prefix_ + enhancedOccupancyMapCode_opt).c_str(),  //
       value<bool>(&_params->enhancedOccupancyMapCode_),        //
       "Use enhanced-delta-depth code")                         //
      (string(prefix_ + EOMFixBitCount_opt).c_str(),            //
       value<size_t>(&_params->EOMFixBitCount_),                //
       "enhanced occupancy map fixed bit count")                //
      (string(prefix_ + occupancyMapRefinement_opt).c_str(),    //
       value<bool>(&_params->occupancyMapRefinement_),          //
       "Use occupancy map refinement")                          //

      // hash
      (string(prefix_ + decodedAtlasInformationHash_opt).c_str(),                    //
       value<size_t>(&_params->decodedAtlasInformationHash_),                        //
       "Enable decoded atlas information hash 0. disable 1.MD5 2.CRC 3.Checksum\n")  //

      // smoothing
      (string(prefix_ + attributeTransferFilterType_opt).c_str(),  //
       value<size_t>(&_params->attrTransferFilterType_),           //
       "Exclude geometry smoothing from attribute transfer\n")     //
      (string(prefix_ + flagGeometrySmoothing_opt).c_str(),        //
       value<bool>(&_params->flagGeometrySmoothing_),              //
       "Enable geometry smoothing\n")                              //
      (string(prefix_ + neighborCountSmoothing_opt).c_str(),       //
       value<size_t>(&_params->neighborCountSmoothing_),           //
       "Neighbor count smoothing")                                 //
      (string(prefix_ + radius2Smoothing_opt).c_str(),             //
       value<double>(&_params->radius2Smoothing_),                 //
       "Radius to smoothing")                                      //
      (string(prefix_ + radius2BoundaryDetection_opt).c_str(),     //
       value<double>(&_params->radius2BoundaryDetection_),         //
       "Radius to boundary detection")                             //
      (string(prefix_ + thresholdSmoothing_opt).c_str(),           //
       value<double>(&_params->thresholdSmoothing_),               //
       "Threshold smoothing")                                      //

      // Patch Expansion (m47772 CE2.12)
      (string(prefix_ + patchExpansion_opt).c_str(),  //
       value<bool>(&_params->patchExpansion_),        //
       "Use occupancy map refinement")                //

      // grid smoothing (m44705 CE2.17)
      (string(prefix_ + gridSmoothing_opt).c_str(),  //
       value<bool>(&_params->gridSmoothing_),        //
       "Enable grid smoothing")                      //
      (string(prefix_ + gridSize_opt).c_str(),       //
       value<size_t>(&_params->gridSize_),           //
       "grid size for the smoothing")                //

      // color smoothing
      (string(prefix_ + thresholdColorSmoothing_opt).c_str(),   //
       value<double>(&_params->thresholdColorSmoothing_),       //
       "Threshold of color smoothing")                          //
      (string(prefix_ + cgridSize_opt).c_str(),                 //
       value<size_t>(&_params->cgridSize_),                     //
       "grid size for the color smoothing")                     //
      (string(prefix_ + thresholdColorDifference_opt).c_str(),  //
       value<double>(&_params->thresholdColorDifference_),      //
       "Threshold of color difference between cells")           //
      (string(prefix_ + thresholdColorVariation_opt).c_str(),   //
       value<double>(&_params->thresholdColorVariation_),       //
       "Threshold of color variation in cells")                 //
      (string(prefix_ + flagColorSmoothing_opt).c_str(),        //
       value<bool>(&_params->flagColorSmoothing_),              //
       "Enable color smoothing\n")                              //

      // color pre-smoothing
      (string(prefix_ + thresholdColorPreSmoothing_opt).c_str(),              //
       value<double>(&_params->thresholdColorPreSmoothing_),                  //
       "Threshold of color pre-smoothing")                                    //
      (string(prefix_ + thresholdColorPreSmoothingLocalEntropy_opt).c_str(),  //
       value<double>(&_params->thresholdColorPreSmoothingLocalEntropy_),      //
       "Threshold of color pre-smoothing local entropy")                      //
      (string(prefix_ + radius2ColorPreSmoothing_opt).c_str(),                //
       value<double>(&_params->radius2ColorPreSmoothing_),                    //
       "Redius of color pre-smoothing")                                       //
      (string(prefix_ + neighborCountColorPreSmoothing_opt).c_str(),          //
       value<size_t>(&_params->neighborCountColorPreSmoothing_),              //
       "Neighbor count for color pre-smoothing")                              //
      (string(prefix_ + flagColorPreSmoothing_opt).c_str(),                   //
       value<bool>(&_params->flagColorPreSmoothing_),                         //
       "Enable color pre-smoothing\n")                                        //

      // colouring
      (string(prefix_ + bestColorSearchRange_opt).c_str(),  //
       value<size_t>(&_params->bestColorSearchRange_),      //
       "Best color search range")                           //

      // Improved color transfer (m49367 CE2.17)
      (string(prefix_ + numNeighborsColorTransferFwd_opt).c_str(),             //
       value<int>(&_params->numNeighborsColorTransferFwd_),                    //
       "Number of neighbors creating Fwd list")                                //
      (string(prefix_ + numNeighborsColorTransferBwd_opt).c_str(),             //
       value<int>(&_params->numNeighborsColorTransferBwd_),                    //
       "Number of neighbors creating Bwd list")                                //
      (string(prefix_ + useDistWeightedAverageFwd_opt).c_str(),                //
       value<bool>(&_params->useDistWeightedAverageFwd_),                      //
       "Distance weighted average for Fwd list")                               //
      (string(prefix_ + useDistWeightedAverageBwd_opt).c_str(),                //
       value<bool>(&_params->useDistWeightedAverageBwd_),                      //
       "Distance weighted average for Bwd list")                               //
      (string(prefix_ + skipAvgIfIdenticalSourcePointPresentFwd_opt).c_str(),  //
       value<bool>(&_params->skipAvgIfIdenticalSourcePointPresentFwd_),        //
       "Skip avgeraging if target is identical to a Fwd point")                //
      (string(prefix_ + skipAvgIfIdenticalSourcePointPresentBwd_opt).c_str(),  //
       value<bool>(&_params->skipAvgIfIdenticalSourcePointPresentBwd_),        //
       "Skip avgeraging if target is identical to a Bwd point")                //
      (string(prefix_ + distOffsetFwd_opt).c_str(),                            //
       value<double>(&_params->distOffsetFwd_),                                //
       "Distance offset to avoid infinite weight")                             //
      (string(prefix_ + distOffsetBwd_opt).c_str(),                            //
       value<double>(&_params->distOffsetBwd_),                                //
       "Distance offset to avoid infinite weight")                             //
      (string(prefix_ + maxGeometryDist2Fwd_opt).c_str(),                      //
       value<double>(&_params->maxGeometryDist2Fwd_),                          //
       "Maximum allowed distance for a Fwd point")                             //
      (string(prefix_ + maxGeometryDist2Bwd_opt).c_str(),                      //
       value<double>(&_params->maxGeometryDist2Bwd_),                          //
       "Maximum allowed distance for a Bwd point")                             //
      (string(prefix_ + maxColorDist2Fwd_opt).c_str(),                         //
       value<double>(&_params->maxColorDist2Fwd_),                             //
       "Maximum allowed pari-wise color distance for Fwd list")                //
      (string(prefix_ + maxColorDist2Bwd_opt).c_str(),                         //
       value<double>(&_params->maxColorDist2Bwd_),                             //
       "Maximum allowed pari-wise color distance for Bwd list")                //
      (string(prefix_ + excludeColorOutlier_opt).c_str(),                      //
       value<bool>(&_params->excludeColorOutlier_),                            //
       "Exclude color outliers from the NN set")                               //
      (string(prefix_ + thresholdColorOutlierDist_opt).c_str(),                //
       value<double>(&_params->thresholdColorOutlierDist_),                    //
       "Threshold of color distance to exclude outliers from the NN set")      //

      // video encoding
      (string(prefix_ + videoEncoderOccupancyPath_opt).c_str(),        //
       value<string>(&_params->videoEncoderOccupancyPath_),            //
       "Occupancy video encoder executable path")                      //
      (string(prefix_ + videoEncoderGeometryPath_opt).c_str(),         //
       value<string>(&_params->videoEncoderGeometryPath_),             //
       "Geometry video encoder executable path")                       //
      (string(prefix_ + videoEncoderAttributePath_opt).c_str(),        //
       value<string>(&_params->videoEncoderAttributePath_),            //
       "Attribute video encoder executable path")                      //
      (string(prefix_ + videoEncoderOccupancyCodecId_opt).c_str(),     //
       value<PCCCodecId>(&_params->videoEncoderOccupancyCodecId_),     //
       "Occupancy video encoder codec id")                             //
      (string(prefix_ + videoEncoderGeometryCodecId_opt).c_str(),      //
       value<PCCCodecId>(&_params->videoEncoderGeometryCodecId_),      //
       "Geometry video encoder codec id")                              //
      (string(prefix_ + videoEncoderAttributeCodecId_opt).c_str(),     //
       value<PCCCodecId>(&_params->videoEncoderAttributeCodecId_),     //
       "Attribute video encoder codec id")                             //
      (string(prefix_ + byteStreamVideoEncoderOccupancy_opt).c_str(),  //
       value<bool>(&_params->byteStreamVideoCoderOccupancy_),          //
       "Attribute video encoder outputs byteStream")                   //
      (string(prefix_ + byteStreamVideoEncoderGeometry_opt).c_str(),   //
       value<bool>(&_params->byteStreamVideoCoderGeometry_),           //
       "Attribute video encoder outputs byteStream")                   //
      (string(prefix_ + byteStreamVideoEncoderAttribute_opt).c_str(),  //
       value<bool>(&_params->byteStreamVideoCoderAttribute_),          //
       "Attribute video encoder outputs byteStream")                   //

      (string(prefix_ + geometryQP_opt).c_str(),                              //
       value<int>(&_params->geometryQP_),                                     //
       "QP for compression of geometry video")                                //
      (string(prefix_ + attributeQP_opt).c_str(),                             //
       value<int>(&_params->attributeQP_),                                    //
       "QP for compression of attribute video")                               //
      (string(prefix_ + auxGeometryQP_opt).c_str(),                           //
       value<int>(&_params->auxGeometryQP_),                                  //
       "QP for compression of auxiliary geometry video : "                    //
       "default=4 for lossy raw points, geometryQP for lossless raw points")  //
      (string(prefix_ + auxAttributeQP_opt).c_str(),                          //
       value<int>(&_params->auxAttributeQP_),                                 //
       "QP for compression of auxiliary attribute video")                     //
      (string(prefix_ + geometryConfig_opt).c_str(),                          //
       value<string>(&_params->geometryConfig_),                              //
       "HM configuration file for geometry compression")                      //
      (string(prefix_ + geometry0Config_opt).c_str(),                         //
       value<string>(&_params->geometry0Config_),                             //
       "HM configuration file for geometry 0 compression")                    //
      (string(prefix_ + geometry1Config_opt).c_str(),                         //
       value<string>(&_params->geometry1Config_),                             //
       "HM configuration file for geometry 1 compression")                    //
      (string(prefix_ + attributeConfig_opt).c_str(),                         //
       value<string>(&_params->attributeConfig_),                             //
       "HM configuration file for attribute compression")                     //
      (string(prefix_ + attribute0Config_opt).c_str(),                        //
       value<string>(&_params->attribute0Config_),                            //
       "HM configuration file for attribute 0 compression")                   //
      (string(prefix_ + attribute1Config_opt).c_str(),                        //
       value<string>(&_params->attribute1Config_),                            //
       "HM configuration file for attribute 1 compression")                   //
      (string(prefix_ + rawPointsPatch_opt).c_str(),                          //
       value<bool>(&_params->rawPointsPatch_),                                //
       "Enable raw points patch\n")                                           //
      (string(prefix_ + noAttributes_opt).c_str(),                            //
       value<bool>(&_params->noAttributes_),                                  //
       "Disable encoding of attributes")                                      //
      (string(prefix_ + attributeVideo444_opt).c_str(),                       //
       value<bool>(&_params->attributeVideo444_),                             //
       "Use 444 format for attribute video")                                  //
      (string(prefix_ + useRawPointsSeparateVideo_opt).c_str(),               //
       value<bool>(&_params->useRawPointsSeparateVideo_),                     //
       "compress raw points with video codec")                                //
      (string(prefix_ + attributeRawSeparateVideoWidth_opt).c_str(),          //
       value<size_t>(&_params->attributeRawSeparateVideoWidth_),              //
       "Width of the MP's attribute in separate video")                       //
      (string(prefix_ + geometryMPConfig_opt).c_str(),                        //
       value<string>(&_params->geometryAuxVideoConfig_),                      //
       "HM configuration file for raw points geometry compression")           //
      (string(prefix_ + attributeMPConfig_opt).c_str(),                       //
       value<string>(&_params->attributeAuxVideoConfig_),                     //
       "HM configuration file for raw points attribute compression")          //

      // etc
      (string(prefix_ + nbThread_opt).c_str(),               //
       value<size_t>(&_params->nbThread_),                   //
       "Number of thread used for parallel processing")      //
      (string(prefix_ + keepIntermediateFiles_opt).c_str(),  //
       value<bool>(&_params->keepIntermediateFiles_),        //
       "Keep intermediate files: RGB, YUV and bin")          //
      (string(prefix_ + absoluteD1_opt).c_str(),             //
       value<bool>(&_params->absoluteD1_),                   //
       "Absolute D1")                                        //
      (string(prefix_ + absoluteT1_opt).c_str(),             //
       value<bool>(&_params->absoluteT1_),                   //
       "Absolute T1")                                        //
      (string(prefix_ + multipleStreams_opt).c_str(),        //
       value<bool>(&_params->multipleStreams_),              //

       // 0. absolute 1 delta
       "number of video(geometry and attribute) streams")   //
      (string(prefix_ + deltaQPD0_opt).c_str(),             //
       value<int>(&_params->deltaQPD0_),                    //
       "qp adjustment for geometry0 video: 0, +3, -3...")   //
      (string(prefix_ + deltaQPD1_opt).c_str(),             //
       value<int>(&_params->deltaQPD1_),                    //
       "qp adjustment for geometry1 video: 0, +3, -3...")   //
      (string(prefix_ + deltaQPT0_opt).c_str(),             //
       value<int>(&_params->deltaQPT0_),                    //
       "qp adjustment for attribute0 video: 0, +3, -3...")  //
      (string(prefix_ + deltaQPT1_opt).c_str(),             //
       value<int>(&_params->deltaQPT1_),                    //
       "qp adjustment for attribute1 video: 0, +3, -3...")  //

      (string(prefix_ + constrainedPack_opt).c_str(),                    //
       value<bool>(&_params->constrainedPack_),                          //
       "Temporally consistent patch packing")                            //
      (string(prefix_ + levelOfDetailX_opt).c_str(),                     //
       value<size_t>(&_params->levelOfDetailX_),                         //
       "levelOfDetail : X axis in 2D space (should be greater than 1)")  //
      (string(prefix_ + levelOfDetailY_opt).c_str(),                     //
       value<size_t>(&_params->levelOfDetailY_),                         //
       "levelOfDetail : Y axis in 2D space (should be greater than 1)")  //
      (string(prefix_ + groupDilation_opt).c_str(),                      //
       value<bool>(&_params->groupDilation_),                            //
       "Group Dilation")                                                 //

      // Lossy occupancy map coding
      (string(prefix_ + offsetLossyOM_opt).c_str(),                                                   //
       value<size_t>(&_params->offsetLossyOM_),                                                       //
       "Value to be assigned to non-zero occupancy map positions (default=0)\n")                      //
      (string(prefix_ + thresholdLossyOM_opt).c_str(),                                                //
       value<size_t>(&_params->thresholdLossyOM_),                                                    //
       "Threshold for converting non-binary occupancy map to binary (default=0)\n")                   //
      (string(prefix_ + prefilterLossyOM_opt).c_str(),                                                //
       value<bool>(&_params->prefilterLossyOM_),                                                      //
       "Selects whether the occupany map is prefiltered before lossy compression (default=false)\n")  //

      // SHVC
      (string(prefix_ + shvcLayerIndex_opt).c_str(),                                                //
       value<size_t>(&_params->shvcLayerIndex_),                                                    //
       "Decode Layer ID number using SHVC codec")                                                   //
      (string(prefix_ + shvcRateX_opt).c_str(),                                                     //
       value<size_t>(&_params->shvcRateX_),                                                         //
       "SHVCRateX : reduce rate of each SHVC layer X axis in 2D space (should be greater than 1)")  //
      (string(prefix_ + shvcRateY_opt).c_str(),                                                     //
       value<size_t>(&_params->shvcRateY_),                                                         //
       "SHVCRateY : reduce rate of each SHVC layer Y axis in 2D space (should be greater than 1)")  //

      // visual quality
      (string(prefix_ + patchColorSubsampling_opt).c_str(),             //
       value<bool>(&_params->patchColorSubsampling_),                   //
       "Enable per patch color sub-sampling\n")                         //
      (string(prefix_ + maxNumRefAtalsList_opt).c_str(),                //
       value<size_t>(&_params->maxNumRefAtlasList_),                    //
       "maximum Number of Reference Atlas Frame list, default: 1")      //
      (string(prefix_ + maxNumRefAtlasFrame_opt).c_str(),               //
       value<size_t>(&_params->maxNumRefAtlasFrame_),                   //
       "maximum Number of Reference Atlas Frame per list, default: 1")  //
      (string(prefix_ + pointLocalReconstruction_opt).c_str(),          //
       value<bool>(&_params->pointLocalReconstruction_),                //
       "Use point local reconstruction")                                //
      (string(prefix_ + mapCountMinus1_opt).c_str(),                    //
       value<size_t>(&_params->mapCountMinus1_),                        //
       "Numbers of layers (rename to maps?)")                           //
      (string(prefix_ + singleMapPixelInterleaving_opt).c_str(),        //
       value<bool>(&_params->singleMapPixelInterleaving_),              //
       "Use single layer pixel interleaving")                           //
      (string(prefix_ + removeDuplicatePoints_opt).c_str(),             //
       value<bool>(&_params->removeDuplicatePoints_),                   //
       "Remove duplicate points( ")                                     //
      (string(prefix_ + surfaceSeparation_opt).c_str(),                 //
       value<bool>(&_params->surfaceSeparation_),                       //
       "surface separation")                                            //

      // high gradient separation
      (string(prefix_ + highGradientSeparation_opt).c_str(),                             //
       value<bool>(&_params->highGradientSeparation_),                                   //
       "Separate high gradient points from a patch")                                     //
      (string(prefix_ + minGradient_opt).c_str(),                                        //
       value<double>(&_params->minGradient_),                                            //
       "Minimun gradient for a point to be separated")                                   //
      (string(prefix_ + minNumHighGradientPoints_opt).c_str(),                           //
       value<size_t>(&_params->minNumHighGradientPoints_),                               //
       "Minimum number of connected high gradient points to be separated from a patch")  //

      // flexible packing + safeguard + push-pull
      (string(prefix_ + packingStrategy_opt).c_str(),                                                   //
       value<size_t>(&_params->packingStrategy_),                                                       //
       "Patches packing strategy(0: anchor packing, 1(default): flexible packing, 2: tetris packing)")  //
      (string(prefix_ + useEightOrientations_opt).c_str(),                                              //
       value<bool>(&_params->useEightOrientations_),                                                    //
       "Allow either 2 orientations (0(default): NULL AND SWAP), or 8 orientation (1)\n")               //
      (string(prefix_ + safeGuardDistance_opt).c_str(),                                                 //
       value<size_t>(&_params->safeGuardDistance_),                                                     //
       "Number of empty blocks that must exist between the patches (default=1)\n")                      //
      (string(prefix_ + attributeBGFill_opt).c_str(),                                                   //
       value<size_t>(&_params->attributeBGFill_),                                                       //
       "Selects the background filling operation for attribute only (0: patch-edge extension, "         //
       "1(default): smoothed push-pull algorithm), 2: harmonic background filling ")                    //

      // lossy-raw-points patch
      (string(prefix_ + lossyRawPointsPatch_opt).c_str(),                                                    //
       value<bool>(&_params->lossyRawPointsPatch_),                                                          //
       "Lossy raw points patch(0: no lossy raw points patch, 1: enable lossy raw points patch (default=0)")  //
      (string(prefix_ + minNormSumOfInvDist4MPSelection_opt).c_str(),                                        //
       value<double>(&_params->minNormSumOfInvDist4MPSelection_),                                            //
       "Minimum normalized sum of inverse distance for raw points selection: "                               //
       "double value between 0.0 and 1.0 (default=0.35)")                                                    //
      (string(prefix_ + globalPatchAllocation_opt).c_str(),                                                  //
       value<int>(&_params->globalPatchAllocation_),                                                         //
       "Global temporally consistent patch allocation."                                                      //
       "(0: anchor's packing method(default), 1: gpa algorithm, 2: gtp algorithm)")                          //
      (string(prefix_ + globalPackingStrategyGOF_opt).c_str(),                                               //
       value<int>(&_params->globalPackingStrategyGOF_),                                                      //
       "Number of frames to pack globally (0:(entire GOF))")                                                 //
      (string(prefix_ + globalPackingStrategyReset_opt).c_str(),                                             //
       value<bool>(&_params->globalPackingStrategyReset_),                                                   //
       "Remove the reference to the previous frame (0(default), 1)")                                         //
      (string(prefix_ + globalPackingStrategyThreshold_opt).c_str(),                                         //
       value<double>(&_params->globalPackingStrategyThreshold_),                                             //
       "matched patches area ratio threshold (decides if connections are valid or not, 0(default))")         //
      (string(prefix_ + patchPrecedenceOrder_opt).c_str(),                                                   //
       value<bool>(&_params->patchPrecedenceOrderFlag_),                                                     //
       "Order of patches")                                                                                   //
      (string(prefix_ + lowDelayEncoding_opt).c_str(),                                                       //
       value<bool>(&_params->lowDelayEncoding_),                                                             //
       "Low Delay encoding (0(default): do nothing, 1: does not allow overlap of patches bounding boxes for low delay "
       "encoding)")                                                                                           //
      (string(prefix_ + geometryPadding_opt).c_str(),                                                         //
       value<size_t>(&_params->geometryPadding_),                                                             //
       "Selects the background filling operation for geometry (0: anchor, 1(default): 3D geometry padding)")  //
      (string(prefix_ + apply3dMotionCompensation_opt).c_str(),                                               //
       value<bool>(&_params->use3dmc_),                                                                       //
       "Use auxilliary information for 3d motion compensation.(0: conventional video coding, 1: 3D motion "
       "compensated)")                                                                                             //
      (string(prefix_ + usePccRDO_opt).c_str(),                                                                    //
       value<bool>(&_params->usePccRDO_),                                                                          //
       "Use HEVC PCC RDO optimization")                                                                            //
      (string(prefix_ + geometry3dCoordinatesBitdepth_opt).c_str(),                                                //
       value<size_t>(&_params->geometry3dCoordinatesBitdepth_),                                                    //
       "Bit depth of geomtery 3D coordinates")                                                                     //
      (string(prefix_ + geometryNominal2dBitdepth_opt).c_str(),                                                    //
       value<size_t>(&_params->geometryNominal2dBitdepth_),                                                        //
       "Bit depth of geometry 2D")                                                                                 //
      (string(prefix_ + nbPlrmMode_opt).c_str(),                                                                   //
       value<size_t>(&_params->plrlNumberOfModes_),                                                                //
       "Number of PLR mode")                                                                                       //
      (string(prefix_ + patchSize_opt).c_str(),                                                                    //
       value<size_t>(&_params->patchSize_),                                                                        //
       "Size of Patch for PLR")                                                                                    //
      (string(prefix_ + enhancedProjectionPlane_opt).c_str(),                                                      //
       value<bool>(&_params->enhancedPP_),                                                                         //
       "Use enhanced Projection Plane(0: OFF, 1: ON)")                                                             //
      (string(prefix_ + minWeightEPP_opt).c_str(),                                                                 //
       value<double>(&_params->minWeightEPP_),                                                                     //
       "Minimum value")                                                                                            //
      (string(prefix_ + additionalProjectionPlaneMode_opt).c_str(),                                                //
       value<int>(&_params->additionalProjectionPlaneMode_),                                                       //
       "additiona Projection Plane Mode 0:none 1:Y-Axis 2:X-Axis 3:Z-Axis 4:All-Axis 5:apply to portion ")         //
      (string(prefix_ + partialAdditionalProjectionPlane_opt).c_str(),                                             //
       value<double>(&_params->partialAdditionalProjectionPlane_),                                                 //
       "The value determines the partial point cloud. It's available with only additionalProjectionPlaneMode(5)")  //
      (string(prefix_ + numMaxTilePerFrame_opt).c_str(),                                                           //
       value<size_t>(&_params->numMaxTilePerFrame_),                                                               //
       "number of maximum tiles in a frame")                                                                       //
      (string(prefix_ + uniformPartitionSpacing_opt).c_str(),                                                      //
       value<bool>(&_params->uniformPartitionSpacing_),                                                            //
       "indictation of uniform partitioning")                                                                      //
      (string(prefix_ + tilePartitionWidth_opt).c_str(),                                                           //
       value<size_t>(&_params->tilePartitionWidth_),                                                               //
       "uniform partition width in the unit of 64 pixels")                                                         //
      (string(prefix_ + tilePartitionHeight_opt).c_str(),                                                          //
       value<size_t>(&_params->tilePartitionHeight_),                                                              //
       "uniform partition height in the unit of 64 pixels")                                                        //
      (string(prefix_ + tilePartitionWidthList_opt).c_str(),                                                       //
       value<vector<int32_t>>(&_params->tilePartitionWidthList_),                                                  //
       "non uniform partition width in the unit of 64 pixels")                                                     //
      (string(prefix_ + tilePartitionHeightList_opt).c_str(),                                                      //
       value<vector<int32_t>>(&_params->tilePartitionHeightList_),                                                 //
       "non uniform partition height in the unit of 64 pixels")                                                    //
      (string(prefix_ + tileSegmentationType_opt).c_str(),                                                         //
       value<size_t>(&_params->tileSegmentationType_),                                                             //
       "tile segmentaton method : 0.no tile partition 1. 3D ROI based 2.2D Patch size based ")                     //

      // Point cloud partitions (ROIs) and tiles (m47804 CE2.19)
      (string(prefix_ + enablePointCloudPartitioning_opt).c_str(),  //
       value<bool>(&_params->enablePointCloudPartitioning_),        //
       " ")                                                         //
      (string(prefix_ + roiBoundingBoxMinX_opt).c_str(),            //
       value<vector<int>>(&_params->roiBoundingBoxMinX_),           //
       " ")                                                         //
      (string(prefix_ + roiBoundingBoxMaxX_opt).c_str(),            //
       value<vector<int>>(&_params->roiBoundingBoxMaxX_),           //
       " ")                                                         //
      (string(prefix_ + roiBoundingBoxMinY_opt).c_str(),            //
       value<vector<int>>(&_params->roiBoundingBoxMinY_),           //
       " ")                                                         //
      (string(prefix_ + roiBoundingBoxMaxY_opt).c_str(),            //
       value<vector<int>>(&_params->roiBoundingBoxMaxY_),           //
       " ")                                                         //
      (string(prefix_ + roiBoundingBoxMinZ_opt).c_str(),            //
       value<vector<int>>(&_params->roiBoundingBoxMinZ_),           //
       " ")                                                         //
      (string(prefix_ + roiBoundingBoxMaxZ_opt).c_str(),            //
       value<vector<int>>(&_params->roiBoundingBoxMaxZ_),           //
       " ")                                                         //
      (string(prefix_ + numTilesHor_opt).c_str(),                   //
       value<int>(&_params->numTilesHor_),                          //
       " ")                                                         //
      (string(prefix_ + tileHeightToWidthRatio_opt).c_str(),        //
       value<double>(&_params->tileHeightToWidthRatio_),            //
       " ")                                                         //
      (string(prefix_ + numCutsAlong1stLongestAxis_opt).c_str(),    //
       value<int>(&_params->numCutsAlong1stLongestAxis_),           //
       " ")                                                         //
      (string(prefix_ + numCutsAlong2ndLongestAxis_opt).c_str(),    //
       value<int>(&_params->numCutsAlong2ndLongestAxis_),           //
       " ")                                                         //
      (string(prefix_ + numCutsAlong3rdLongestAxis_opt).c_str(),    //
       value<int>(&_params->numCutsAlong3rdLongestAxis_),           //
       " ")                                                         //

      // Sort raw points by Morton code (m49363 CE2.25)
      (string(prefix_ + mortonOrderSortRawPoints_opt).c_str(),  //
       value<bool>(&_params->mortonOrderSortRawPoints_),        //
       " ")                                                     //

      // Patch block filtering
      (string(prefix_ + pbfEnableFlag_opt).c_str(),             //
       value<bool>(&_params->pbfEnableFlag_),                   //
       " enable patch block filtering ")                        //
      (string(prefix_ + pbfFilterSize_opt).c_str(),             //
       value<int16_t>(&_params->pbfFilterSize_),                //
       "pbfFilterSize ")                                        //
      (string(prefix_ + pbfPassesCount_opt).c_str(),            //
       value<int16_t>(&_params->pbfPassesCount_),               //
       "pbfPassesCount ")                                       //
      (string(prefix_ + pbfLog2Threshold_opt).c_str(),          //
       value<int16_t>(&_params->pbfLog2Threshold_),             //
       "pbfLog2Threshold ")                                     //
      (string(prefix_ + tierFlag_opt).c_str(),                  //
       value<bool>(&_params->tierFlag_),                        //
       "Tier Flag")                                             //
      (string(prefix_ + profileCodecGroupIdc_opt).c_str(),      //
       value<size_t>(&_params->profileCodecGroupIdc_),          //
       "Profile Codec Group Idc")                               //
      (string(prefix_ + profileToolsetIdc_opt).c_str(),         //
       value<size_t>(&_params->profileToolsetIdc_),             //
       "Profile Toolset Idc")                                   //
      (string(prefix_ + profileReconstructionIdc_opt).c_str(),  //
       value<size_t>(&_params->profileReconstructionIdc_),      //
       "Profile Reconstruction Idc")                            //
      (string(prefix_ + levelIdc_opt).c_str(),                  //
       value<size_t>(&_params->levelIdc_),                      //
       "Level Idc")                                             //

      // ajt0526: avcCodecIdIndex/hevcCodecIdIndex/vvcCodecIdIndex when using CCM SEI and
      // profileCodecGroupIdc beeds to correspond to mp4RA?
      (string(prefix_ + avcCodecIdIndex_opt).c_str(),   //
       value<size_t>(&_params->avcCodecIdIndex_),       //
       "index for avc codec ")                          //
      (string(prefix_ + hevcCodecIdIndex_opt).c_str(),  //
       value<size_t>(&_params->hevcCodecIdIndex_),      //
       "index for hevc codec ")                         //
      (string(prefix_ + shvcCodecIdIndex_opt).c_str(),  //
       value<size_t>(&_params->shvcCodecIdIndex_),      //
       "index for shvc codec ")                         //
      (string(prefix_ + vvcCodecIdIndex_opt).c_str(),   //
       value<size_t>(&_params->vvcCodecIdIndex_),       //
       "index for vvc codec ")                          //

      // 8.3.4.6	Profile toolset constraints information syntax
      (string(prefix_ + oneV3CFrameOnlyFlag_opt).c_str(),  //
       value<bool>(&_params->oneV3CFrameOnlyFlag_),        //
       "One V3C Frame Only Flag")                          //
                                                           // TODO
      ;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC2Parameter::notify() {
  auto _params = std::static_pointer_cast<pcc::PCCEncoderParameters>(params_);
  _params->completePath();
  THROW_IF_NOT(_params->check());
}

//////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& out, const JPCCEncoderTMC2Parameter& obj) {
  auto _params = std::static_pointer_cast<pcc::PCCEncoderParameters>(obj.params_);
  obj.coutParameters(out)                                                                               //
      (configurationFolder_opt, _params->configurationFolder_)                                          //
      (uncompressedDataFolder_opt, _params->uncompressedDataFolder_)                                    //
      (uncompressedDataPath_opt, _params->uncompressedDataPath_)                                        //
      (compressedStreamPath_opt, _params->compressedStreamPath_)                                        //
      (reconstructedDataPath_opt, _params->reconstructedDataPath_)                                      //
      (forcedSsvhUnitSizePrecisionBytes_opt, _params->forcedSsvhUnitSizePrecisionBytes_)                //
      (startFrameNumber_opt, _params->startFrameNumber_)                                                //
      (frameCount_opt, _params->frameCount_)                                                            //
      (groupOfFramesSize_opt, _params->groupOfFramesSize_)                                              //
      (colorTransform_opt, _params->colorTransform_)                                                    //
      (colorSpaceConversionPath_opt, _params->colorSpaceConversionPath_)                                //
      (colorSpaceConversionConfig_opt, _params->colorSpaceConversionConfig_)                            //
      (inverseColorSpaceConversionConfig_opt, _params->inverseColorSpaceConversionConfig_)              //
      (gridBasedSegmentation_opt, _params->gridBasedSegmentation_)                                      //
      (voxelDimensionGridBasedSegmentation_opt, _params->voxelDimensionGridBasedSegmentation_)          //
      (nnNormalEstimation_opt, _params->nnNormalEstimation_)                                            //
      (normalOrientation_opt, _params->normalOrientation_)                                              //
      (gridBasedRefineSegmentation_opt, _params->gridBasedRefineSegmentation_)                          //
      (maxNNCountRefineSegmentation_opt, _params->maxNNCountRefineSegmentation_)                        //
      (iterationCountRefineSegmentation_opt, _params->iterationCountRefineSegmentation_)                //
      (voxelDimensionRefineSegmentation_opt, _params->voxelDimensionRefineSegmentation_)                //
      (searchRadiusRefineSegmentation_opt, _params->searchRadiusRefineSegmentation_)                    //
      (occupancyResolution_opt, _params->occupancyResolution_)                                          //
      (enablePatchSplitting_opt, _params->enablePatchSplitting_)                                        //
      (maxPatchSize_opt, _params->maxPatchSize_)                                                        //
      (log2QuantizerSizeX_opt, _params->log2QuantizerSizeX_)                                            //
      (log2QuantizerSizeY_opt, _params->log2QuantizerSizeY_)                                            //
      (minPointCountPerCCPatchSegmentation_opt, _params->minPointCountPerCCPatchSegmentation_)          //
      (maxNNCountPatchSegmentation_opt, _params->maxNNCountPatchSegmentation_)                          //
      (surfaceThickness_opt, _params->surfaceThickness_)                                                //
      (depthQuantizationStep_opt, _params->minLevel_)                                                   //
      (maxAllowedDist2RawPointsDetection_opt, _params->maxAllowedDist2RawPointsDetection_)              //
      (maxAllowedDist2RawPointsSelection_opt, _params->maxAllowedDist2RawPointsSelection_)              //
      (lambdaRefineSegmentation_opt, _params->lambdaRefineSegmentation_)                                //
      (minimumImageWidth_opt, _params->minimumImageWidth_)                                              //
      (minimumImageHeight_opt, _params->minimumImageHeight_)                                            //
      (maxCandidateCount_opt, _params->maxCandidateCount_)                                              //
      (occupancyPrecision_opt, _params->occupancyPrecision_)                                            //
      (occupancyMapConfig_opt, _params->occupancyMapConfig_)                                            //
      (occupancyMapQP_opt, _params->occupancyMapQP_)                                                    //
      (enhancedOccupancyMapCode_opt, _params->enhancedOccupancyMapCode_)                                //
      (EOMFixBitCount_opt, _params->EOMFixBitCount_)                                                    //
      (occupancyMapRefinement_opt, _params->occupancyMapRefinement_)                                    //
      (decodedAtlasInformationHash_opt, _params->decodedAtlasInformationHash_)                          //
      (attributeTransferFilterType_opt, _params->attrTransferFilterType_)                               //
      (flagGeometrySmoothing_opt, _params->flagGeometrySmoothing_)                                      //
      (neighborCountSmoothing_opt, _params->neighborCountSmoothing_)                                    //
      (radius2Smoothing_opt, _params->radius2Smoothing_)                                                //
      (radius2BoundaryDetection_opt, _params->radius2BoundaryDetection_)                                //
      (thresholdSmoothing_opt, _params->thresholdSmoothing_)                                            //
      (patchExpansion_opt, _params->patchExpansion_)                                                    //
      (gridSmoothing_opt, _params->gridSmoothing_)                                                      //
      (gridSize_opt, _params->gridSize_)                                                                //
      (thresholdColorSmoothing_opt, _params->thresholdColorSmoothing_)                                  //
      (cgridSize_opt, _params->cgridSize_)                                                              //
      (thresholdColorDifference_opt, _params->thresholdColorDifference_)                                //
      (thresholdColorVariation_opt, _params->thresholdColorVariation_)                                  //
      (flagColorSmoothing_opt, _params->flagColorSmoothing_)                                            //
      (thresholdColorPreSmoothing_opt, _params->thresholdColorPreSmoothing_)                            //
      (thresholdColorPreSmoothingLocalEntropy_opt, _params->thresholdColorPreSmoothingLocalEntropy_)    //
      (radius2ColorPreSmoothing_opt, _params->radius2ColorPreSmoothing_)                                //
      (neighborCountColorPreSmoothing_opt, _params->neighborCountColorPreSmoothing_)                    //
      (flagColorPreSmoothing_opt, _params->flagColorPreSmoothing_)                                      //
      (bestColorSearchRange_opt, _params->bestColorSearchRange_)                                        //
      (numNeighborsColorTransferFwd_opt, _params->numNeighborsColorTransferFwd_)                        //
      (numNeighborsColorTransferBwd_opt, _params->numNeighborsColorTransferBwd_)                        //
      (useDistWeightedAverageFwd_opt, _params->useDistWeightedAverageFwd_)                              //
      (useDistWeightedAverageBwd_opt, _params->useDistWeightedAverageBwd_)                              //
      (skipAvgIfIdenticalSourcePointPresentFwd_opt, _params->skipAvgIfIdenticalSourcePointPresentFwd_)  //
      (skipAvgIfIdenticalSourcePointPresentBwd_opt, _params->skipAvgIfIdenticalSourcePointPresentBwd_)  //
      (distOffsetFwd_opt, _params->distOffsetFwd_)                                                      //
      (distOffsetBwd_opt, _params->distOffsetBwd_)                                                      //
      (maxGeometryDist2Fwd_opt, _params->maxGeometryDist2Fwd_)                                          //
      (maxGeometryDist2Bwd_opt, _params->maxGeometryDist2Bwd_)                                          //
      (maxColorDist2Fwd_opt, _params->maxColorDist2Fwd_)                                                //
      (maxColorDist2Bwd_opt, _params->maxColorDist2Bwd_)                                                //
      (excludeColorOutlier_opt, _params->excludeColorOutlier_)                                          //
      (thresholdColorOutlierDist_opt, _params->thresholdColorOutlierDist_)                              //
      (videoEncoderOccupancyPath_opt, _params->videoEncoderOccupancyPath_)                              //
      (videoEncoderGeometryPath_opt, _params->videoEncoderGeometryPath_)                                //
      (videoEncoderAttributePath_opt, _params->videoEncoderAttributePath_)                              //
      (videoEncoderOccupancyCodecId_opt, _params->videoEncoderOccupancyCodecId_)                        //
      (videoEncoderGeometryCodecId_opt, _params->videoEncoderGeometryCodecId_)                          //
      (videoEncoderAttributeCodecId_opt, _params->videoEncoderAttributeCodecId_)                        //
      (byteStreamVideoEncoderOccupancy_opt, _params->byteStreamVideoCoderOccupancy_)                    //
      (byteStreamVideoEncoderGeometry_opt, _params->byteStreamVideoCoderGeometry_)                      //
      (byteStreamVideoEncoderAttribute_opt, _params->byteStreamVideoCoderAttribute_)                    //
      (geometryQP_opt, _params->geometryQP_)                                                            //
      (attributeQP_opt, _params->attributeQP_)                                                          //
      (auxGeometryQP_opt, _params->auxGeometryQP_)                                                      //
      (auxAttributeQP_opt, _params->auxAttributeQP_)                                                    //
      (geometryConfig_opt, _params->geometryConfig_)                                                    //
      (geometry0Config_opt, _params->geometry0Config_)                                                  //
      (geometry1Config_opt, _params->geometry1Config_)                                                  //
      (attributeConfig_opt, _params->attributeConfig_)                                                  //
      (attribute0Config_opt, _params->attribute0Config_)                                                //
      (attribute1Config_opt, _params->attribute1Config_)                                                //
      (rawPointsPatch_opt, _params->rawPointsPatch_)                                                    //
      (noAttributes_opt, _params->noAttributes_)                                                        //
      (attributeVideo444_opt, _params->attributeVideo444_)                                              //
      (useRawPointsSeparateVideo_opt, _params->useRawPointsSeparateVideo_)                              //
      (attributeRawSeparateVideoWidth_opt, _params->attributeRawSeparateVideoWidth_)                    //
      (geometryMPConfig_opt, _params->geometryAuxVideoConfig_)                                          //
      (attributeMPConfig_opt, _params->attributeAuxVideoConfig_)                                        //
      (nbThread_opt, _params->nbThread_)                                                                //
      (keepIntermediateFiles_opt, _params->keepIntermediateFiles_)                                      //
      (absoluteD1_opt, _params->absoluteD1_)                                                            //
      (absoluteT1_opt, _params->absoluteT1_)                                                            //
      (multipleStreams_opt, _params->multipleStreams_)                                                  //
      (deltaQPD0_opt, _params->deltaQPD0_)                                                              //
      (deltaQPD1_opt, _params->deltaQPD1_)                                                              //
      (deltaQPT0_opt, _params->deltaQPT0_)                                                              //
      (deltaQPT1_opt, _params->deltaQPT1_)                                                              //
      (constrainedPack_opt, _params->constrainedPack_)                                                  //
      (levelOfDetailX_opt, _params->levelOfDetailX_)                                                    //
      (levelOfDetailY_opt, _params->levelOfDetailY_)                                                    //
      (groupDilation_opt, _params->groupDilation_)                                                      //
      (offsetLossyOM_opt, _params->offsetLossyOM_)                                                      //
      (thresholdLossyOM_opt, _params->thresholdLossyOM_)                                                //
      (prefilterLossyOM_opt, _params->prefilterLossyOM_)                                                //
      (shvcLayerIndex_opt, _params->shvcLayerIndex_)                                                    //
      (shvcRateX_opt, _params->shvcRateX_)                                                              //
      (shvcRateY_opt, _params->shvcRateY_)                                                              //
      (patchColorSubsampling_opt, _params->patchColorSubsampling_)                                      //
      (maxNumRefAtalsList_opt, _params->maxNumRefAtlasList_)                                            //
      (maxNumRefAtlasFrame_opt, _params->maxNumRefAtlasFrame_)                                          //
      (pointLocalReconstruction_opt, _params->pointLocalReconstruction_)                                //
      (mapCountMinus1_opt, _params->mapCountMinus1_)                                                    //
      (singleMapPixelInterleaving_opt, _params->singleMapPixelInterleaving_)                            //
      (removeDuplicatePoints_opt, _params->removeDuplicatePoints_)                                      //
      (surfaceSeparation_opt, _params->surfaceSeparation_)                                              //
      (highGradientSeparation_opt, _params->highGradientSeparation_)                                    //
      (minGradient_opt, _params->minGradient_)                                                          //
      (minNumHighGradientPoints_opt, _params->minNumHighGradientPoints_)                                //
      (packingStrategy_opt, _params->packingStrategy_)                                                  //
      (useEightOrientations_opt, _params->useEightOrientations_)                                        //
      (safeGuardDistance_opt, _params->safeGuardDistance_)                                              //
      (attributeBGFill_opt, _params->attributeBGFill_)                                                  //
      (lossyRawPointsPatch_opt, _params->lossyRawPointsPatch_)                                          //
      (minNormSumOfInvDist4MPSelection_opt, _params->minNormSumOfInvDist4MPSelection_)                  //
      (globalPatchAllocation_opt, _params->globalPatchAllocation_)                                      //
      (globalPackingStrategyGOF_opt, _params->globalPackingStrategyGOF_)                                //
      (globalPackingStrategyReset_opt, _params->globalPackingStrategyReset_)                            //
      (globalPackingStrategyThreshold_opt, _params->globalPackingStrategyThreshold_)                    //
      (patchPrecedenceOrder_opt, _params->patchPrecedenceOrderFlag_)                                    //
      (lowDelayEncoding_opt, _params->lowDelayEncoding_)                                                //
      (geometryPadding_opt, _params->geometryPadding_)                                                  //
      (apply3dMotionCompensation_opt, _params->use3dmc_)                                                //
      (usePccRDO_opt, _params->usePccRDO_)                                                              //
      (geometry3dCoordinatesBitdepth_opt, _params->geometry3dCoordinatesBitdepth_)                      //
      (geometryNominal2dBitdepth_opt, _params->geometryNominal2dBitdepth_)                              //
      (nbPlrmMode_opt, _params->plrlNumberOfModes_)                                                     //
      (patchSize_opt, _params->patchSize_)                                                              //
      (enhancedProjectionPlane_opt, _params->enhancedPP_)                                               //
      (minWeightEPP_opt, _params->minWeightEPP_)                                                        //
      (additionalProjectionPlaneMode_opt, _params->additionalProjectionPlaneMode_)                      //
      (partialAdditionalProjectionPlane_opt, _params->partialAdditionalProjectionPlane_)                //
      (numMaxTilePerFrame_opt, _params->numMaxTilePerFrame_)                                            //
      (uniformPartitionSpacing_opt, _params->uniformPartitionSpacing_)                                  //
      (tilePartitionWidth_opt, _params->tilePartitionWidth_)                                            //
      (tilePartitionHeight_opt, _params->tilePartitionHeight_)                                          //
      (tilePartitionWidthList_opt, _params->tilePartitionWidthList_)                                    //
      (tilePartitionHeightList_opt, _params->tilePartitionHeightList_)                                  //
      (tileSegmentationType_opt, _params->tileSegmentationType_)                                        //
      (enablePointCloudPartitioning_opt, _params->enablePointCloudPartitioning_)                        //
      (roiBoundingBoxMinX_opt, _params->roiBoundingBoxMinX_)                                            //
      (roiBoundingBoxMaxX_opt, _params->roiBoundingBoxMaxX_)                                            //
      (roiBoundingBoxMinY_opt, _params->roiBoundingBoxMinY_)                                            //
      (roiBoundingBoxMaxY_opt, _params->roiBoundingBoxMaxY_)                                            //
      (roiBoundingBoxMinZ_opt, _params->roiBoundingBoxMinZ_)                                            //
      (roiBoundingBoxMaxZ_opt, _params->roiBoundingBoxMaxZ_)                                            //
      (numTilesHor_opt, _params->numTilesHor_)                                                          //
      (tileHeightToWidthRatio_opt, _params->tileHeightToWidthRatio_)                                    //
      (numCutsAlong1stLongestAxis_opt, _params->numCutsAlong1stLongestAxis_)                            //
      (numCutsAlong2ndLongestAxis_opt, _params->numCutsAlong2ndLongestAxis_)                            //
      (numCutsAlong3rdLongestAxis_opt, _params->numCutsAlong3rdLongestAxis_)                            //
      (mortonOrderSortRawPoints_opt, _params->mortonOrderSortRawPoints_)                                //
      (pbfEnableFlag_opt, _params->pbfEnableFlag_)                                                      //
      (pbfFilterSize_opt, _params->pbfFilterSize_)                                                      //
      (pbfPassesCount_opt, _params->pbfPassesCount_)                                                    //
      (pbfLog2Threshold_opt, _params->pbfLog2Threshold_)                                                //
      (tierFlag_opt, _params->tierFlag_)                                                                //
      (profileCodecGroupIdc_opt, _params->profileCodecGroupIdc_)                                        //
      (profileToolsetIdc_opt, _params->profileToolsetIdc_)                                              //
      (profileReconstructionIdc_opt, _params->profileReconstructionIdc_)                                //
      (levelIdc_opt, _params->levelIdc_)                                                                //
      (avcCodecIdIndex_opt, _params->avcCodecIdIndex_)                                                  //
      (hevcCodecIdIndex_opt, _params->hevcCodecIdIndex_)                                                //
      (shvcCodecIdIndex_opt, _params->shvcCodecIdIndex_)                                                //
      (vvcCodecIdIndex_opt, _params->vvcCodecIdIndex_)                                                  //
      (oneV3CFrameOnlyFlag_opt, _params->oneV3CFrameOnlyFlag_)                                          //
      ;
  return out;
}
}  // namespace jpcc::encoder

//////////////////////////////////////////////////////////////////////////////////////////////
namespace pcc {
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

#ifdef USE_JMAPP_VIDEO_CODEC
  if (tmp == "JMAPP" || tmp == "jmapp") { val = JMAPP; }
#endif
#ifdef USE_HMAPP_VIDEO_CODEC
  if (tmp == "HMAPP" || tmp == "hmapp") { val = HMAPP; }
#endif
#ifdef USE_SHMAPP_VIDEO_CODEC
  if (tmp == "SHMAPP" || tmp == "shmapp" || tmp == "shm" || tmp == "shm2" || tmp == "shm3") { val = SHMAPP; }
#endif
#ifdef USE_JMLIB_VIDEO_CODEC
  if (tmp == "JMLIB" || tmp == "jmlib" || tmp == "jm" || tmp == "avc") { val = JMLIB; }
#endif
#ifdef USE_HMLIB_VIDEO_CODEC
  if (tmp == "HMLIB" || tmp == "hmlib" || tmp == "hm" || tmp == "hevc") { val = HMLIB; }
#endif
#ifdef USE_VTMLIB_VIDEO_CODEC
  if (tmp == "VTMLIB" || tmp == "vtmlib" || tmp == "vtm" || tmp == "vvc") { val = VTMLIB; }
#endif
#ifdef USE_FFMPEG_VIDEO_CODEC
  if (tmp == "FFMPEG" || tmp == "ffmpeg") { val = FFMPEG; }
#endif
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
#ifdef USE_JMAPP_VIDEO_CODEC
    case PCCCodecId::JMAPP: out << int(val) << "(JMAPP)"; break;
#endif
#ifdef USE_HMAPP_VIDEO_CODEC
    case PCCCodecId::HMAPP: out << int(val) << "(HMAPP)"; break;
#endif
#ifdef USE_SHMAPP_VIDEO_CODEC
    case PCCCodecId::SHMAPP: out << int(val) << "(SHMAPP)"; break;
#endif
#ifdef USE_JMLIB_VIDEO_CODEC
    case PCCCodecId::JMLIB: out << int(val) << "(JMLIB)"; break;
#endif
#ifdef USE_HMLIB_VIDEO_CODEC
    case PCCCodecId::HMLIB: out << int(val) << "(HMLIB)"; break;
#endif
#ifdef USE_VTMLIB_VIDEO_CODEC
    case PCCCodecId::VTMLIB: out << int(val) << "(VTMLIB)"; break;
#endif
#ifdef USE_FFMPEG_VIDEO_CODEC
    case PCCCodecId::FFMPEG: out << int(val) << "(FFMPEG)"; break;
#endif
    default: out << int(val) << " (Unknown)"; break;
  }
  return out;
}

}  // namespace pcc
