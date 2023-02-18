#include <jpcc/encoder/JPCCEncoderTMC2Parameter.h>

#include <PCCEncoderParameters.h>

#define CONFIG_OPT ".configs"
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
  auto _params  = std::make_shared<pcc::PCCEncoderParameters>();
  this->params_ = _params;
  opts_.add_options()                               //
      (string(prefix_ + CONFIG_OPT).c_str(),        //
       value<vector<string>>(&this->tmc2Configs_),  //
       "tmc2Configs")                               //
      ;
  tmc2Opts_.add_options()                                                                 //
      (string(configurationFolder_opt).c_str(),                                           //
       value<string>(&_params->configurationFolder_),                                     //
       "Folder where the configuration files are stored,use for cfg relative paths.")     //
      (string(uncompressedDataFolder_opt).c_str(),                                        //
       value<string>(&_params->uncompressedDataFolder_),                                  //
       "Folder where the uncompress input data are stored, use for cfg relative paths.")  //

      // i/o
      (string(uncompressedDataPath_opt).c_str(),                                        //
       value<string>(&_params->uncompressedDataPath_),                                  //
       "Input pointcloud to encode. Multi-frame sequences may be represented by %04i")  //
      (string(compressedStreamPath_opt).c_str(),                                        //
       value<string>(&_params->compressedStreamPath_),                                  //
       "Output(encoder)/Input(decoder) compressed bitstream")                           //
      (string(reconstructedDataPath_opt).c_str(),                                       //
       value<string>(&_params->reconstructedDataPath_),                                 //
       "Output decoded pointcloud. Multi-frame sequences may be represented by %04i")   //
      (string(forcedSsvhUnitSizePrecisionBytes_opt).c_str(),                            //
       value<uint32_t>(&_params->forcedSsvhUnitSizePrecisionBytes_),                    //
       "forced SSVH unit size precision bytes")                                         //

      // sequence configuration
      (string(startFrameNumber_opt).c_str(),               //
       value<size_t>(&_params->startFrameNumber_),         //
       "First frame number in sequence to encode/decode")  //
      (string(frameCount_opt).c_str(),                     //
       value<size_t>(&_params->frameCount_),               //
       "Number of frames to encode")                       //
      (string(groupOfFramesSize_opt).c_str(),              //
       value<size_t>(&_params->groupOfFramesSize_),        //
       "Random access period")                             //

      // colour space conversion
      (string(colorTransform_opt).c_str(),                                              //
       value<PCCColorTransform>(&_params->colorTransform_),                             //
       "The colour transform to be applied:\n  0: none\n  1: RGB to YCbCr (Rec.709)")   //
      (string(colorSpaceConversionPath_opt).c_str(),                                    //
       value<string>(&_params->colorSpaceConversionPath_),                              //
       "Path to the HDRConvert. If unset, an internal color space conversion is used")  //
      (string(colorSpaceConversionConfig_opt).c_str(),                                  //
       value<string>(&_params->colorSpaceConversionConfig_),                            //
       "HDRConvert configuration file used for RGB444 to YUV420 conversion")            //
      (string(inverseColorSpaceConversionConfig_opt).c_str(),                           //
       value<string>(&_params->inverseColorSpaceConversionConfig_),                     //
       "HDRConvert configuration file used for YUV420 to RGB444 conversion")            //

      // segmentation
      // If enabled, change refineSegmentationGridBased() parameters according to m56857"
      (string(gridBasedSegmentation_opt).c_str(),                                           //
       value<bool>(&_params->gridBasedSegmentation_),                                       //
       "Voxel dimension for grid-based segmentation (GBS)")                                 //
      (string(voxelDimensionGridBasedSegmentation_opt).c_str(),                             //
       value<size_t>(&_params->voxelDimensionGridBasedSegmentation_),                       //
       "Voxel dimension for grid-based segmentation (GBS)")                                 //
      (string(nnNormalEstimation_opt).c_str(),                                              //
       value<size_t>(&_params->nnNormalEstimation_),                                        //
       "Number of points used for normal estimation")                                       //
      (string(normalOrientation_opt).c_str(),                                               //
       value<size_t>(&_params->normalOrientation_),                                         //
       "Normal orientation: 0: None 1: spanning tree, 2:view point, 3:cubemap projection")  //
      (string(gridBasedRefineSegmentation_opt).c_str(),                                     //
       value<bool>(&_params->gridBasedRefineSegmentation_),                                 //
       "Use grid-based approach for segmentation refinement")                               //
      (string(maxNNCountRefineSegmentation_opt).c_str(),                                    //
       value<size_t>(&_params->maxNNCountRefineSegmentation_),                              //
       "Number of nearest neighbors used during segmentation refinement")                   //
      (string(iterationCountRefineSegmentation_opt).c_str(),                                //
       value<size_t>(&_params->iterationCountRefineSegmentation_),                          //
       "Number of iterations performed during segmentation refinement")                     //
      (string(voxelDimensionRefineSegmentation_opt).c_str(),                                //
       value<size_t>(&_params->voxelDimensionRefineSegmentation_),                          //
       "Voxel dimension for segmentation refinement (must be a power of 2)")                //
      (string(searchRadiusRefineSegmentation_opt).c_str(),                                  //
       value<size_t>(&_params->searchRadiusRefineSegmentation_),                            //
       "Search radius for segmentation refinement")                                         //
      (string(occupancyResolution_opt).c_str(),                                             //
       value<size_t>(&_params->occupancyResolution_),                                       //
       "Resolution of packing block(a block contain only one patch)")                       //
      (string(enablePatchSplitting_opt).c_str(),                                            //
       value<bool>(&_params->enablePatchSplitting_),                                        //
       "Enable patch splitting")                                                            //
      (string(maxPatchSize_opt).c_str(),                                                    //
       value<size_t>(&_params->maxPatchSize_),                                              //
       "Maximum patch size for segmentation")                                               //
      (string(log2QuantizerSizeX_opt).c_str(),                                              //
       value<size_t>(&_params->log2QuantizerSizeX_),                                        //
       "log2 of Quantization step for patch size X: 0. pixel precision 4.16 as before")     //
      (string(log2QuantizerSizeY_opt).c_str(),                                              //
       value<size_t>(&_params->log2QuantizerSizeY_),                                        //
       "log2 of Quantization step for patch size Y: 0. pixel precision 4.16 as before")     //
      (string(minPointCountPerCCPatchSegmentation_opt).c_str(),                             //
       value<size_t>(&_params->minPointCountPerCCPatchSegmentation_),                       //
       "Minimum number of points for a connected component to be retained as a patch")      //
      (string(maxNNCountPatchSegmentation_opt).c_str(),                                     //
       value<size_t>(&_params->maxNNCountPatchSegmentation_),                               //
       "Number of nearest neighbors used during connected components extraction")           //
      (string(surfaceThickness_opt).c_str(),                                                //
       value<size_t>(&_params->surfaceThickness_),                                          //
       "Surface thickness")                                                                 //
      (string(depthQuantizationStep_opt).c_str(),                                           //
       value<size_t>(&_params->minLevel_),                                                  //
       "minimum level for patches")                                                         //
      (string(maxAllowedDist2RawPointsDetection_opt).c_str(),                               //
       value<double>(&_params->maxAllowedDist2RawPointsDetection_),                         //
       "Maximum distance for a point to be ignored during raw points detection")            //
      (string(maxAllowedDist2RawPointsSelection_opt).c_str(),                               //
       value<double>(&_params->maxAllowedDist2RawPointsSelection_),                         //
       "Maximum distance for a point to be ignored during  raw points  selection")          //
      (string(lambdaRefineSegmentation_opt).c_str(),                                        //
       value<double>(&_params->lambdaRefineSegmentation_),                                  //
       "Controls the smoothness of the patch boundaries  during segmentation  refinement")  //

      // packing
      (string(minimumImageWidth_opt).c_str(),         //
       value<size_t>(&_params->minimumImageWidth_),   //
       "Minimum width of packed patch frame")         //
      (string(minimumImageHeight_opt).c_str(),        //
       value<size_t>(&_params->minimumImageHeight_),  //
       "Minimum height of packed patch frame")        //

      // occupancy map
      (string(maxCandidateCount_opt).c_str(),         //
       value<size_t>(&_params->maxCandidateCount_),   //
       "Maximum nuber of candidates in list L")       //
      (string(occupancyPrecision_opt).c_str(),        //
       value<size_t>(&_params->occupancyPrecision_),  //
       "Occupancy map B0 precision")                  //
      (string(occupancyMapConfig_opt).c_str(),        //
       value<string>(&_params->occupancyMapConfig_),  //
       "Occupancy map encoder config file")           //
      (string(occupancyMapQP_opt).c_str(),            //
       value<size_t>(&_params->occupancyMapQP_),      //
       "QP for compression of occupancy map video")   //

      // EOM code
      (string(enhancedOccupancyMapCode_opt).c_str(),      //
       value<bool>(&_params->enhancedOccupancyMapCode_),  //
       "Use enhanced-delta-depth code")                   //
      (string(EOMFixBitCount_opt).c_str(),                //
       value<size_t>(&_params->EOMFixBitCount_),          //
       "enhanced occupancy map fixed bit count")          //
      (string(occupancyMapRefinement_opt).c_str(),        //
       value<bool>(&_params->occupancyMapRefinement_),    //
       "Use occupancy map refinement")                    //

      // hash
      (string(decodedAtlasInformationHash_opt).c_str(),                              //
       value<size_t>(&_params->decodedAtlasInformationHash_),                        //
       "Enable decoded atlas information hash 0. disable 1.MD5 2.CRC 3.Checksum\n")  //

      // smoothing
      (string(attributeTransferFilterType_opt).c_str(),         //
       value<size_t>(&_params->attrTransferFilterType_),        //
       "Exclude geometry smoothing from attribute transfer\n")  //
      (string(flagGeometrySmoothing_opt).c_str(),               //
       value<bool>(&_params->flagGeometrySmoothing_),           //
       "Enable geometry smoothing\n")                           //
      (string(neighborCountSmoothing_opt).c_str(),              //
       value<size_t>(&_params->neighborCountSmoothing_),        //
       "Neighbor count smoothing")                              //
      (string(radius2Smoothing_opt).c_str(),                    //
       value<double>(&_params->radius2Smoothing_),              //
       "Radius to smoothing")                                   //
      (string(radius2BoundaryDetection_opt).c_str(),            //
       value<double>(&_params->radius2BoundaryDetection_),      //
       "Radius to boundary detection")                          //
      (string(thresholdSmoothing_opt).c_str(),                  //
       value<double>(&_params->thresholdSmoothing_),            //
       "Threshold smoothing")                                   //

      // Patch Expansion (m47772 CE2.12)
      (string(patchExpansion_opt).c_str(),      //
       value<bool>(&_params->patchExpansion_),  //
       "Use occupancy map refinement")          //

      // grid smoothing (m44705 CE2.17)
      (string(gridSmoothing_opt).c_str(),      //
       value<bool>(&_params->gridSmoothing_),  //
       "Enable grid smoothing")                //
      (string(gridSize_opt).c_str(),           //
       value<size_t>(&_params->gridSize_),     //
       "grid size for the smoothing")          //

      // color smoothing
      (string(thresholdColorSmoothing_opt).c_str(),         //
       value<double>(&_params->thresholdColorSmoothing_),   //
       "Threshold of color smoothing")                      //
      (string(cgridSize_opt).c_str(),                       //
       value<size_t>(&_params->cgridSize_),                 //
       "grid size for the color smoothing")                 //
      (string(thresholdColorDifference_opt).c_str(),        //
       value<double>(&_params->thresholdColorDifference_),  //
       "Threshold of color difference between cells")       //
      (string(thresholdColorVariation_opt).c_str(),         //
       value<double>(&_params->thresholdColorVariation_),   //
       "Threshold of color variation in cells")             //
      (string(flagColorSmoothing_opt).c_str(),              //
       value<bool>(&_params->flagColorSmoothing_),          //
       "Enable color smoothing\n")                          //

      // color pre-smoothing
      (string(thresholdColorPreSmoothing_opt).c_str(),                    //
       value<double>(&_params->thresholdColorPreSmoothing_),              //
       "Threshold of color pre-smoothing")                                //
      (string(thresholdColorPreSmoothingLocalEntropy_opt).c_str(),        //
       value<double>(&_params->thresholdColorPreSmoothingLocalEntropy_),  //
       "Threshold of color pre-smoothing local entropy")                  //
      (string(radius2ColorPreSmoothing_opt).c_str(),                      //
       value<double>(&_params->radius2ColorPreSmoothing_),                //
       "Redius of color pre-smoothing")                                   //
      (string(neighborCountColorPreSmoothing_opt).c_str(),                //
       value<size_t>(&_params->neighborCountColorPreSmoothing_),          //
       "Neighbor count for color pre-smoothing")                          //
      (string(flagColorPreSmoothing_opt).c_str(),                         //
       value<bool>(&_params->flagColorPreSmoothing_),                     //
       "Enable color pre-smoothing\n")                                    //

      // colouring
      (string(bestColorSearchRange_opt).c_str(),        //
       value<size_t>(&_params->bestColorSearchRange_),  //
       "Best color search range")                       //

      // Improved color transfer (m49367 CE2.17)
      (string(numNeighborsColorTransferFwd_opt).c_str(),                   //
       value<int>(&_params->numNeighborsColorTransferFwd_),                //
       "Number of neighbors creating Fwd list")                            //
      (string(numNeighborsColorTransferBwd_opt).c_str(),                   //
       value<int>(&_params->numNeighborsColorTransferBwd_),                //
       "Number of neighbors creating Bwd list")                            //
      (string(useDistWeightedAverageFwd_opt).c_str(),                      //
       value<bool>(&_params->useDistWeightedAverageFwd_),                  //
       "Distance weighted average for Fwd list")                           //
      (string(useDistWeightedAverageBwd_opt).c_str(),                      //
       value<bool>(&_params->useDistWeightedAverageBwd_),                  //
       "Distance weighted average for Bwd list")                           //
      (string(skipAvgIfIdenticalSourcePointPresentFwd_opt).c_str(),        //
       value<bool>(&_params->skipAvgIfIdenticalSourcePointPresentFwd_),    //
       "Skip avgeraging if target is identical to a Fwd point")            //
      (string(skipAvgIfIdenticalSourcePointPresentBwd_opt).c_str(),        //
       value<bool>(&_params->skipAvgIfIdenticalSourcePointPresentBwd_),    //
       "Skip avgeraging if target is identical to a Bwd point")            //
      (string(distOffsetFwd_opt).c_str(),                                  //
       value<double>(&_params->distOffsetFwd_),                            //
       "Distance offset to avoid infinite weight")                         //
      (string(distOffsetBwd_opt).c_str(),                                  //
       value<double>(&_params->distOffsetBwd_),                            //
       "Distance offset to avoid infinite weight")                         //
      (string(maxGeometryDist2Fwd_opt).c_str(),                            //
       value<double>(&_params->maxGeometryDist2Fwd_),                      //
       "Maximum allowed distance for a Fwd point")                         //
      (string(maxGeometryDist2Bwd_opt).c_str(),                            //
       value<double>(&_params->maxGeometryDist2Bwd_),                      //
       "Maximum allowed distance for a Bwd point")                         //
      (string(maxColorDist2Fwd_opt).c_str(),                               //
       value<double>(&_params->maxColorDist2Fwd_),                         //
       "Maximum allowed pari-wise color distance for Fwd list")            //
      (string(maxColorDist2Bwd_opt).c_str(),                               //
       value<double>(&_params->maxColorDist2Bwd_),                         //
       "Maximum allowed pari-wise color distance for Bwd list")            //
      (string(excludeColorOutlier_opt).c_str(),                            //
       value<bool>(&_params->excludeColorOutlier_),                        //
       "Exclude color outliers from the NN set")                           //
      (string(thresholdColorOutlierDist_opt).c_str(),                      //
       value<double>(&_params->thresholdColorOutlierDist_),                //
       "Threshold of color distance to exclude outliers from the NN set")  //

      // video encoding
      (string(videoEncoderOccupancyPath_opt).c_str(),               //
       value<string>(&_params->videoEncoderOccupancyPath_),         //
       "Occupancy video encoder executable path")                   //
      (string(videoEncoderGeometryPath_opt).c_str(),                //
       value<string>(&_params->videoEncoderGeometryPath_),          //
       "Geometry video encoder executable path")                    //
      (string(videoEncoderAttributePath_opt).c_str(),               //
       value<string>(&_params->videoEncoderAttributePath_),         //
       "Attribute video encoder executable path")                   //
      (string(videoEncoderOccupancyCodecId_opt).c_str(),            //
       value<PCCCodecId>(&_params->videoEncoderOccupancyCodecId_),  //
       "Occupancy video encoder codec id")                          //
      (string(videoEncoderGeometryCodecId_opt).c_str(),             //
       value<PCCCodecId>(&_params->videoEncoderGeometryCodecId_),   //
       "Geometry video encoder codec id")                           //
      (string(videoEncoderAttributeCodecId_opt).c_str(),            //
       value<PCCCodecId>(&_params->videoEncoderAttributeCodecId_),  //
       "Attribute video encoder codec id")                          //
      (string(byteStreamVideoEncoderOccupancy_opt).c_str(),         //
       value<bool>(&_params->byteStreamVideoCoderOccupancy_),       //
       "Attribute video encoder outputs byteStream")                //
      (string(byteStreamVideoEncoderGeometry_opt).c_str(),          //
       value<bool>(&_params->byteStreamVideoCoderGeometry_),        //
       "Attribute video encoder outputs byteStream")                //
      (string(byteStreamVideoEncoderAttribute_opt).c_str(),         //
       value<bool>(&_params->byteStreamVideoCoderAttribute_),       //
       "Attribute video encoder outputs byteStream")                //

      (string(geometryQP_opt).c_str(),                                        //
       value<int>(&_params->geometryQP_),                                     //
       "QP for compression of geometry video")                                //
      (string(attributeQP_opt).c_str(),                                       //
       value<int>(&_params->attributeQP_),                                    //
       "QP for compression of attribute video")                               //
      (string(auxGeometryQP_opt).c_str(),                                     //
       value<int>(&_params->auxGeometryQP_),                                  //
       "QP for compression of auxiliary geometry video : "                    //
       "default=4 for lossy raw points, geometryQP for lossless raw points")  //
      (string(auxAttributeQP_opt).c_str(),                                    //
       value<int>(&_params->auxAttributeQP_),                                 //
       "QP for compression of auxiliary attribute video")                     //
      (string(geometryConfig_opt).c_str(),                                    //
       value<string>(&_params->geometryConfig_),                              //
       "HM configuration file for geometry compression")                      //
      (string(geometry0Config_opt).c_str(),                                   //
       value<string>(&_params->geometry0Config_),                             //
       "HM configuration file for geometry 0 compression")                    //
      (string(geometry1Config_opt).c_str(),                                   //
       value<string>(&_params->geometry1Config_),                             //
       "HM configuration file for geometry 1 compression")                    //
      (string(attributeConfig_opt).c_str(),                                   //
       value<string>(&_params->attributeConfig_),                             //
       "HM configuration file for attribute compression")                     //
      (string(attribute0Config_opt).c_str(),                                  //
       value<string>(&_params->attribute0Config_),                            //
       "HM configuration file for attribute 0 compression")                   //
      (string(attribute1Config_opt).c_str(),                                  //
       value<string>(&_params->attribute1Config_),                            //
       "HM configuration file for attribute 1 compression")                   //
      (string(rawPointsPatch_opt).c_str(),                                    //
       value<bool>(&_params->rawPointsPatch_),                                //
       "Enable raw points patch\n")                                           //
      (string(noAttributes_opt).c_str(),                                      //
       value<bool>(&_params->noAttributes_),                                  //
       "Disable encoding of attributes")                                      //
      (string(attributeVideo444_opt).c_str(),                                 //
       value<bool>(&_params->attributeVideo444_),                             //
       "Use 444 format for attribute video")                                  //
      (string(useRawPointsSeparateVideo_opt).c_str(),                         //
       value<bool>(&_params->useRawPointsSeparateVideo_),                     //
       "compress raw points with video codec")                                //
      (string(attributeRawSeparateVideoWidth_opt).c_str(),                    //
       value<size_t>(&_params->attributeRawSeparateVideoWidth_),              //
       "Width of the MP's attribute in separate video")                       //
      (string(geometryMPConfig_opt).c_str(),                                  //
       value<string>(&_params->geometryAuxVideoConfig_),                      //
       "HM configuration file for raw points geometry compression")           //
      (string(attributeMPConfig_opt).c_str(),                                 //
       value<string>(&_params->attributeAuxVideoConfig_),                     //
       "HM configuration file for raw points attribute compression")          //

      // etc
      (string(nbThread_opt).c_str(),                     //
       value<size_t>(&_params->nbThread_),               //
       "Number of thread used for parallel processing")  //
      (string(keepIntermediateFiles_opt).c_str(),        //
       value<bool>(&_params->keepIntermediateFiles_),    //
       "Keep intermediate files: RGB, YUV and bin")      //
      (string(absoluteD1_opt).c_str(),                   //
       value<bool>(&_params->absoluteD1_),               //
       "Absolute D1")                                    //
      (string(absoluteT1_opt).c_str(),                   //
       value<bool>(&_params->absoluteT1_),               //
       "Absolute T1")                                    //
      (string(multipleStreams_opt).c_str(),              //
       value<bool>(&_params->multipleStreams_),          //

       // 0. absolute 1 delta
       "number of video(geometry and attribute) streams")   //
      (string(deltaQPD0_opt).c_str(),                       //
       value<int>(&_params->deltaQPD0_),                    //
       "qp adjustment for geometry0 video: 0, +3, -3...")   //
      (string(deltaQPD1_opt).c_str(),                       //
       value<int>(&_params->deltaQPD1_),                    //
       "qp adjustment for geometry1 video: 0, +3, -3...")   //
      (string(deltaQPT0_opt).c_str(),                       //
       value<int>(&_params->deltaQPT0_),                    //
       "qp adjustment for attribute0 video: 0, +3, -3...")  //
      (string(deltaQPT1_opt).c_str(),                       //
       value<int>(&_params->deltaQPT1_),                    //
       "qp adjustment for attribute1 video: 0, +3, -3...")  //

      (string(constrainedPack_opt).c_str(),                              //
       value<bool>(&_params->constrainedPack_),                          //
       "Temporally consistent patch packing")                            //
      (string(levelOfDetailX_opt).c_str(),                               //
       value<size_t>(&_params->levelOfDetailX_),                         //
       "levelOfDetail : X axis in 2D space (should be greater than 1)")  //
      (string(levelOfDetailY_opt).c_str(),                               //
       value<size_t>(&_params->levelOfDetailY_),                         //
       "levelOfDetail : Y axis in 2D space (should be greater than 1)")  //
      (string(groupDilation_opt).c_str(),                                //
       value<bool>(&_params->groupDilation_),                            //
       "Group Dilation")                                                 //

      // Lossy occupancy map coding
      (string(offsetLossyOM_opt).c_str(),                                                             //
       value<size_t>(&_params->offsetLossyOM_),                                                       //
       "Value to be assigned to non-zero occupancy map positions (default=0)\n")                      //
      (string(thresholdLossyOM_opt).c_str(),                                                          //
       value<size_t>(&_params->thresholdLossyOM_),                                                    //
       "Threshold for converting non-binary occupancy map to binary (default=0)\n")                   //
      (string(prefilterLossyOM_opt).c_str(),                                                          //
       value<bool>(&_params->prefilterLossyOM_),                                                      //
       "Selects whether the occupany map is prefiltered before lossy compression (default=false)\n")  //

      // SHVC
      (string(shvcLayerIndex_opt).c_str(),                                                          //
       value<size_t>(&_params->shvcLayerIndex_),                                                    //
       "Decode Layer ID number using SHVC codec")                                                   //
      (string(shvcRateX_opt).c_str(),                                                               //
       value<size_t>(&_params->shvcRateX_),                                                         //
       "SHVCRateX : reduce rate of each SHVC layer X axis in 2D space (should be greater than 1)")  //
      (string(shvcRateY_opt).c_str(),                                                               //
       value<size_t>(&_params->shvcRateY_),                                                         //
       "SHVCRateY : reduce rate of each SHVC layer Y axis in 2D space (should be greater than 1)")  //

      // visual quality
      (string(patchColorSubsampling_opt).c_str(),                       //
       value<bool>(&_params->patchColorSubsampling_),                   //
       "Enable per patch color sub-sampling\n")                         //
      (string(maxNumRefAtalsList_opt).c_str(),                          //
       value<size_t>(&_params->maxNumRefAtlasList_),                    //
       "maximum Number of Reference Atlas Frame list, default: 1")      //
      (string(maxNumRefAtlasFrame_opt).c_str(),                         //
       value<size_t>(&_params->maxNumRefAtlasFrame_),                   //
       "maximum Number of Reference Atlas Frame per list, default: 1")  //
      (string(pointLocalReconstruction_opt).c_str(),                    //
       value<bool>(&_params->pointLocalReconstruction_),                //
       "Use point local reconstruction")                                //
      (string(mapCountMinus1_opt).c_str(),                              //
       value<size_t>(&_params->mapCountMinus1_),                        //
       "Numbers of layers (rename to maps?)")                           //
      (string(singleMapPixelInterleaving_opt).c_str(),                  //
       value<bool>(&_params->singleMapPixelInterleaving_),              //
       "Use single layer pixel interleaving")                           //
      (string(removeDuplicatePoints_opt).c_str(),                       //
       value<bool>(&_params->removeDuplicatePoints_),                   //
       "Remove duplicate points( ")                                     //
      (string(surfaceSeparation_opt).c_str(),                           //
       value<bool>(&_params->surfaceSeparation_),                       //
       "surface separation")                                            //

      // high gradient separation
      (string(highGradientSeparation_opt).c_str(),                                       //
       value<bool>(&_params->highGradientSeparation_),                                   //
       "Separate high gradient points from a patch")                                     //
      (string(minGradient_opt).c_str(),                                                  //
       value<double>(&_params->minGradient_),                                            //
       "Minimun gradient for a point to be separated")                                   //
      (string(minNumHighGradientPoints_opt).c_str(),                                     //
       value<size_t>(&_params->minNumHighGradientPoints_),                               //
       "Minimum number of connected high gradient points to be separated from a patch")  //

      // flexible packing + safeguard + push-pull
      (string(packingStrategy_opt).c_str(),                                                             //
       value<size_t>(&_params->packingStrategy_),                                                       //
       "Patches packing strategy(0: anchor packing, 1(default): flexible packing, 2: tetris packing)")  //
      (string(useEightOrientations_opt).c_str(),                                                        //
       value<bool>(&_params->useEightOrientations_),                                                    //
       "Allow either 2 orientations (0(default): NULL AND SWAP), or 8 orientation (1)\n")               //
      (string(safeGuardDistance_opt).c_str(),                                                           //
       value<size_t>(&_params->safeGuardDistance_),                                                     //
       "Number of empty blocks that must exist between the patches (default=1)\n")                      //
      (string(attributeBGFill_opt).c_str(),                                                             //
       value<size_t>(&_params->attributeBGFill_),                                                       //
       "Selects the background filling operation for attribute only (0: patch-edge extension, "         //
       "1(default): smoothed push-pull algorithm), 2: harmonic background filling ")                    //

      // lossy-raw-points patch
      (string(lossyRawPointsPatch_opt).c_str(),                                                              //
       value<bool>(&_params->lossyRawPointsPatch_),                                                          //
       "Lossy raw points patch(0: no lossy raw points patch, 1: enable lossy raw points patch (default=0)")  //
      (string(minNormSumOfInvDist4MPSelection_opt).c_str(),                                                  //
       value<double>(&_params->minNormSumOfInvDist4MPSelection_),                                            //
       "Minimum normalized sum of inverse distance for raw points selection: "                               //
       "double value between 0.0 and 1.0 (default=0.35)")                                                    //
      (string(globalPatchAllocation_opt).c_str(),                                                            //
       value<int>(&_params->globalPatchAllocation_),                                                         //
       "Global temporally consistent patch allocation."                                                      //
       "(0: anchor's packing method(default), 1: gpa algorithm, 2: gtp algorithm)")                          //
      (string(globalPackingStrategyGOF_opt).c_str(),                                                         //
       value<int>(&_params->globalPackingStrategyGOF_),                                                      //
       "Number of frames to pack globally (0:(entire GOF))")                                                 //
      (string(globalPackingStrategyReset_opt).c_str(),                                                       //
       value<bool>(&_params->globalPackingStrategyReset_),                                                   //
       "Remove the reference to the previous frame (0(default), 1)")                                         //
      (string(globalPackingStrategyThreshold_opt).c_str(),                                                   //
       value<double>(&_params->globalPackingStrategyThreshold_),                                             //
       "matched patches area ratio threshold (decides if connections are valid or not, 0(default))")         //
      (string(patchPrecedenceOrder_opt).c_str(),                                                             //
       value<bool>(&_params->patchPrecedenceOrderFlag_),                                                     //
       "Order of patches")                                                                                   //
      (string(lowDelayEncoding_opt).c_str(),                                                                 //
       value<bool>(&_params->lowDelayEncoding_),                                                             //
       "Low Delay encoding (0(default): do nothing, 1: does not allow overlap of patches bounding boxes for low delay "
       "encoding)")                                                                                           //
      (string(geometryPadding_opt).c_str(),                                                                   //
       value<size_t>(&_params->geometryPadding_),                                                             //
       "Selects the background filling operation for geometry (0: anchor, 1(default): 3D geometry padding)")  //
      (string(apply3dMotionCompensation_opt).c_str(),                                                         //
       value<bool>(&_params->use3dmc_),                                                                       //
       "Use auxilliary information for 3d motion compensation.(0: conventional video coding, 1: 3D motion "
       "compensated)")                                                                                             //
      (string(usePccRDO_opt).c_str(),                                                                              //
       value<bool>(&_params->usePccRDO_),                                                                          //
       "Use HEVC PCC RDO optimization")                                                                            //
      (string(geometry3dCoordinatesBitdepth_opt).c_str(),                                                          //
       value<size_t>(&_params->geometry3dCoordinatesBitdepth_),                                                    //
       "Bit depth of geomtery 3D coordinates")                                                                     //
      (string(geometryNominal2dBitdepth_opt).c_str(),                                                              //
       value<size_t>(&_params->geometryNominal2dBitdepth_),                                                        //
       "Bit depth of geometry 2D")                                                                                 //
      (string(nbPlrmMode_opt).c_str(),                                                                             //
       value<size_t>(&_params->plrlNumberOfModes_),                                                                //
       "Number of PLR mode")                                                                                       //
      (string(patchSize_opt).c_str(),                                                                              //
       value<size_t>(&_params->patchSize_),                                                                        //
       "Size of Patch for PLR")                                                                                    //
      (string(enhancedProjectionPlane_opt).c_str(),                                                                //
       value<bool>(&_params->enhancedPP_),                                                                         //
       "Use enhanced Projection Plane(0: OFF, 1: ON)")                                                             //
      (string(minWeightEPP_opt).c_str(),                                                                           //
       value<double>(&_params->minWeightEPP_),                                                                     //
       "Minimum value")                                                                                            //
      (string(additionalProjectionPlaneMode_opt).c_str(),                                                          //
       value<int>(&_params->additionalProjectionPlaneMode_),                                                       //
       "additiona Projection Plane Mode 0:none 1:Y-Axis 2:X-Axis 3:Z-Axis 4:All-Axis 5:apply to portion ")         //
      (string(partialAdditionalProjectionPlane_opt).c_str(),                                                       //
       value<double>(&_params->partialAdditionalProjectionPlane_),                                                 //
       "The value determines the partial point cloud. It's available with only additionalProjectionPlaneMode(5)")  //
      (string(numMaxTilePerFrame_opt).c_str(),                                                                     //
       value<size_t>(&_params->numMaxTilePerFrame_),                                                               //
       "number of maximum tiles in a frame")                                                                       //
      (string(uniformPartitionSpacing_opt).c_str(),                                                                //
       value<bool>(&_params->uniformPartitionSpacing_),                                                            //
       "indictation of uniform partitioning")                                                                      //
      (string(tilePartitionWidth_opt).c_str(),                                                                     //
       value<size_t>(&_params->tilePartitionWidth_),                                                               //
       "uniform partition width in the unit of 64 pixels")                                                         //
      (string(tilePartitionHeight_opt).c_str(),                                                                    //
       value<size_t>(&_params->tilePartitionHeight_),                                                              //
       "uniform partition height in the unit of 64 pixels")                                                        //
      (string(tilePartitionWidthList_opt).c_str(),                                                                 //
       value<vector<int32_t>>(&_params->tilePartitionWidthList_),                                                  //
       "non uniform partition width in the unit of 64 pixels")                                                     //
      (string(tilePartitionHeightList_opt).c_str(),                                                                //
       value<vector<int32_t>>(&_params->tilePartitionHeightList_),                                                 //
       "non uniform partition height in the unit of 64 pixels")                                                    //
      (string(tileSegmentationType_opt).c_str(),                                                                   //
       value<size_t>(&_params->tileSegmentationType_),                                                             //
       "tile segmentaton method : 0.no tile partition 1. 3D ROI based 2.2D Patch size based ")                     //

      // Point cloud partitions (ROIs) and tiles (m47804 CE2.19)
      (string(enablePointCloudPartitioning_opt).c_str(),      //
       value<bool>(&_params->enablePointCloudPartitioning_),  //
       " ")                                                   //
      (string(roiBoundingBoxMinX_opt).c_str(),                //
       value<vector<int>>(&_params->roiBoundingBoxMinX_),     //
       " ")                                                   //
      (string(roiBoundingBoxMaxX_opt).c_str(),                //
       value<vector<int>>(&_params->roiBoundingBoxMaxX_),     //
       " ")                                                   //
      (string(roiBoundingBoxMinY_opt).c_str(),                //
       value<vector<int>>(&_params->roiBoundingBoxMinY_),     //
       " ")                                                   //
      (string(roiBoundingBoxMaxY_opt).c_str(),                //
       value<vector<int>>(&_params->roiBoundingBoxMaxY_),     //
       " ")                                                   //
      (string(roiBoundingBoxMinZ_opt).c_str(),                //
       value<vector<int>>(&_params->roiBoundingBoxMinZ_),     //
       " ")                                                   //
      (string(roiBoundingBoxMaxZ_opt).c_str(),                //
       value<vector<int>>(&_params->roiBoundingBoxMaxZ_),     //
       " ")                                                   //
      (string(numTilesHor_opt).c_str(),                       //
       value<int>(&_params->numTilesHor_),                    //
       " ")                                                   //
      (string(tileHeightToWidthRatio_opt).c_str(),            //
       value<double>(&_params->tileHeightToWidthRatio_),      //
       " ")                                                   //
      (string(numCutsAlong1stLongestAxis_opt).c_str(),        //
       value<int>(&_params->numCutsAlong1stLongestAxis_),     //
       " ")                                                   //
      (string(numCutsAlong2ndLongestAxis_opt).c_str(),        //
       value<int>(&_params->numCutsAlong2ndLongestAxis_),     //
       " ")                                                   //
      (string(numCutsAlong3rdLongestAxis_opt).c_str(),        //
       value<int>(&_params->numCutsAlong3rdLongestAxis_),     //
       " ")                                                   //

      // Sort raw points by Morton code (m49363 CE2.25)
      (string(mortonOrderSortRawPoints_opt).c_str(),      //
       value<bool>(&_params->mortonOrderSortRawPoints_),  //
       " ")                                               //

      // Patch block filtering
      (string(pbfEnableFlag_opt).c_str(),                   //
       value<bool>(&_params->pbfEnableFlag_),               //
       " enable patch block filtering ")                    //
      (string(pbfFilterSize_opt).c_str(),                   //
       value<int16_t>(&_params->pbfFilterSize_),            //
       "pbfFilterSize ")                                    //
      (string(pbfPassesCount_opt).c_str(),                  //
       value<int16_t>(&_params->pbfPassesCount_),           //
       "pbfPassesCount ")                                   //
      (string(pbfLog2Threshold_opt).c_str(),                //
       value<int16_t>(&_params->pbfLog2Threshold_),         //
       "pbfLog2Threshold ")                                 //
      (string(tierFlag_opt).c_str(),                        //
       value<bool>(&_params->tierFlag_),                    //
       "Tier Flag")                                         //
      (string(profileCodecGroupIdc_opt).c_str(),            //
       value<size_t>(&_params->profileCodecGroupIdc_),      //
       "Profile Codec Group Idc")                           //
      (string(profileToolsetIdc_opt).c_str(),               //
       value<size_t>(&_params->profileToolsetIdc_),         //
       "Profile Toolset Idc")                               //
      (string(profileReconstructionIdc_opt).c_str(),        //
       value<size_t>(&_params->profileReconstructionIdc_),  //
       "Profile Reconstruction Idc")                        //
      (string(levelIdc_opt).c_str(),                        //
       value<size_t>(&_params->levelIdc_),                  //
       "Level Idc")                                         //

      // ajt0526: avcCodecIdIndex/hevcCodecIdIndex/vvcCodecIdIndex when using CCM SEI and
      // profileCodecGroupIdc beeds to correspond to mp4RA?
      (string(avcCodecIdIndex_opt).c_str(),         //
       value<size_t>(&_params->avcCodecIdIndex_),   //
       "index for avc codec ")                      //
      (string(hevcCodecIdIndex_opt).c_str(),        //
       value<size_t>(&_params->hevcCodecIdIndex_),  //
       "index for hevc codec ")                     //
      (string(shvcCodecIdIndex_opt).c_str(),        //
       value<size_t>(&_params->shvcCodecIdIndex_),  //
       "index for shvc codec ")                     //
      (string(vvcCodecIdIndex_opt).c_str(),         //
       value<size_t>(&_params->vvcCodecIdIndex_),   //
       "index for vvc codec ")                      //

      // 8.3.4.6	Profile toolset constraints information syntax
      (string(oneV3CFrameOnlyFlag_opt).c_str(),      //
       value<bool>(&_params->oneV3CFrameOnlyFlag_),  //
       "One V3C Frame Only Flag")                    //
                                                     // TODO
      ;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC2Parameter::notify() {
  for (auto it = tmc2Configs_.begin(); it != tmc2Configs_.end(); ++it) {
    const string& config = *it;
    // check if config parsed then return
    ifstream ifs(config.c_str());
    if (!ifs.good()) { BOOST_THROW_EXCEPTION(runtime_error("'" + config + "' not found")); }
    const parsed_options cfgOpts = parse_config_file(ifs, tmc2Opts_, true);
    variables_map        vm;
    store(cfgOpts, vm);
    boost::program_options::notify(vm);
  }
  auto _params = std::static_pointer_cast<pcc::PCCEncoderParameters>(params_);
  _params->completePath();
  THROW_IF_NOT(_params->check());
}

//////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& out, const JPCCEncoderTMC2Parameter& obj) {
  auto _params = std::static_pointer_cast<pcc::PCCEncoderParameters>(obj.params_);
  obj.coutParameters(out)                                                                                             //
      (CONFIG_OPT, obj.tmc2Configs_)                                                                                  //
      (string(".") + configurationFolder_opt, _params->configurationFolder_)                                          //
      (string(".") + configurationFolder_opt, _params->configurationFolder_)                                          //
      (string(".") + uncompressedDataFolder_opt, _params->uncompressedDataFolder_)                                    //
      (string(".") + uncompressedDataPath_opt, _params->uncompressedDataPath_)                                        //
      (string(".") + compressedStreamPath_opt, _params->compressedStreamPath_)                                        //
      (string(".") + reconstructedDataPath_opt, _params->reconstructedDataPath_)                                      //
      (string(".") + forcedSsvhUnitSizePrecisionBytes_opt, _params->forcedSsvhUnitSizePrecisionBytes_)                //
      (string(".") + startFrameNumber_opt, _params->startFrameNumber_)                                                //
      (string(".") + frameCount_opt, _params->frameCount_)                                                            //
      (string(".") + groupOfFramesSize_opt, _params->groupOfFramesSize_)                                              //
      (string(".") + colorTransform_opt, _params->colorTransform_)                                                    //
      (string(".") + colorSpaceConversionPath_opt, _params->colorSpaceConversionPath_)                                //
      (string(".") + colorSpaceConversionConfig_opt, _params->colorSpaceConversionConfig_)                            //
      (string(".") + inverseColorSpaceConversionConfig_opt, _params->inverseColorSpaceConversionConfig_)              //
      (string(".") + gridBasedSegmentation_opt, _params->gridBasedSegmentation_)                                      //
      (string(".") + voxelDimensionGridBasedSegmentation_opt, _params->voxelDimensionGridBasedSegmentation_)          //
      (string(".") + nnNormalEstimation_opt, _params->nnNormalEstimation_)                                            //
      (string(".") + normalOrientation_opt, _params->normalOrientation_)                                              //
      (string(".") + gridBasedRefineSegmentation_opt, _params->gridBasedRefineSegmentation_)                          //
      (string(".") + maxNNCountRefineSegmentation_opt, _params->maxNNCountRefineSegmentation_)                        //
      (string(".") + iterationCountRefineSegmentation_opt, _params->iterationCountRefineSegmentation_)                //
      (string(".") + voxelDimensionRefineSegmentation_opt, _params->voxelDimensionRefineSegmentation_)                //
      (string(".") + searchRadiusRefineSegmentation_opt, _params->searchRadiusRefineSegmentation_)                    //
      (string(".") + occupancyResolution_opt, _params->occupancyResolution_)                                          //
      (string(".") + enablePatchSplitting_opt, _params->enablePatchSplitting_)                                        //
      (string(".") + maxPatchSize_opt, _params->maxPatchSize_)                                                        //
      (string(".") + log2QuantizerSizeX_opt, _params->log2QuantizerSizeX_)                                            //
      (string(".") + log2QuantizerSizeY_opt, _params->log2QuantizerSizeY_)                                            //
      (string(".") + minPointCountPerCCPatchSegmentation_opt, _params->minPointCountPerCCPatchSegmentation_)          //
      (string(".") + maxNNCountPatchSegmentation_opt, _params->maxNNCountPatchSegmentation_)                          //
      (string(".") + surfaceThickness_opt, _params->surfaceThickness_)                                                //
      (string(".") + depthQuantizationStep_opt, _params->minLevel_)                                                   //
      (string(".") + maxAllowedDist2RawPointsDetection_opt, _params->maxAllowedDist2RawPointsDetection_)              //
      (string(".") + maxAllowedDist2RawPointsSelection_opt, _params->maxAllowedDist2RawPointsSelection_)              //
      (string(".") + lambdaRefineSegmentation_opt, _params->lambdaRefineSegmentation_)                                //
      (string(".") + minimumImageWidth_opt, _params->minimumImageWidth_)                                              //
      (string(".") + minimumImageHeight_opt, _params->minimumImageHeight_)                                            //
      (string(".") + maxCandidateCount_opt, _params->maxCandidateCount_)                                              //
      (string(".") + occupancyPrecision_opt, _params->occupancyPrecision_)                                            //
      (string(".") + occupancyMapConfig_opt, _params->occupancyMapConfig_)                                            //
      (string(".") + occupancyMapQP_opt, _params->occupancyMapQP_)                                                    //
      (string(".") + enhancedOccupancyMapCode_opt, _params->enhancedOccupancyMapCode_)                                //
      (string(".") + EOMFixBitCount_opt, _params->EOMFixBitCount_)                                                    //
      (string(".") + occupancyMapRefinement_opt, _params->occupancyMapRefinement_)                                    //
      (string(".") + decodedAtlasInformationHash_opt, _params->decodedAtlasInformationHash_)                          //
      (string(".") + attributeTransferFilterType_opt, _params->attrTransferFilterType_)                               //
      (string(".") + flagGeometrySmoothing_opt, _params->flagGeometrySmoothing_)                                      //
      (string(".") + neighborCountSmoothing_opt, _params->neighborCountSmoothing_)                                    //
      (string(".") + radius2Smoothing_opt, _params->radius2Smoothing_)                                                //
      (string(".") + radius2BoundaryDetection_opt, _params->radius2BoundaryDetection_)                                //
      (string(".") + thresholdSmoothing_opt, _params->thresholdSmoothing_)                                            //
      (string(".") + patchExpansion_opt, _params->patchExpansion_)                                                    //
      (string(".") + gridSmoothing_opt, _params->gridSmoothing_)                                                      //
      (string(".") + gridSize_opt, _params->gridSize_)                                                                //
      (string(".") + thresholdColorSmoothing_opt, _params->thresholdColorSmoothing_)                                  //
      (string(".") + cgridSize_opt, _params->cgridSize_)                                                              //
      (string(".") + thresholdColorDifference_opt, _params->thresholdColorDifference_)                                //
      (string(".") + thresholdColorVariation_opt, _params->thresholdColorVariation_)                                  //
      (string(".") + flagColorSmoothing_opt, _params->flagColorSmoothing_)                                            //
      (string(".") + thresholdColorPreSmoothing_opt, _params->thresholdColorPreSmoothing_)                            //
      (string(".") + thresholdColorPreSmoothingLocalEntropy_opt, _params->thresholdColorPreSmoothingLocalEntropy_)    //
      (string(".") + radius2ColorPreSmoothing_opt, _params->radius2ColorPreSmoothing_)                                //
      (string(".") + neighborCountColorPreSmoothing_opt, _params->neighborCountColorPreSmoothing_)                    //
      (string(".") + flagColorPreSmoothing_opt, _params->flagColorPreSmoothing_)                                      //
      (string(".") + bestColorSearchRange_opt, _params->bestColorSearchRange_)                                        //
      (string(".") + numNeighborsColorTransferFwd_opt, _params->numNeighborsColorTransferFwd_)                        //
      (string(".") + numNeighborsColorTransferBwd_opt, _params->numNeighborsColorTransferBwd_)                        //
      (string(".") + useDistWeightedAverageFwd_opt, _params->useDistWeightedAverageFwd_)                              //
      (string(".") + useDistWeightedAverageBwd_opt, _params->useDistWeightedAverageBwd_)                              //
      (string(".") + skipAvgIfIdenticalSourcePointPresentFwd_opt, _params->skipAvgIfIdenticalSourcePointPresentFwd_)  //
      (string(".") + skipAvgIfIdenticalSourcePointPresentBwd_opt, _params->skipAvgIfIdenticalSourcePointPresentBwd_)  //
      (string(".") + distOffsetFwd_opt, _params->distOffsetFwd_)                                                      //
      (string(".") + distOffsetBwd_opt, _params->distOffsetBwd_)                                                      //
      (string(".") + maxGeometryDist2Fwd_opt, _params->maxGeometryDist2Fwd_)                                          //
      (string(".") + maxGeometryDist2Bwd_opt, _params->maxGeometryDist2Bwd_)                                          //
      (string(".") + maxColorDist2Fwd_opt, _params->maxColorDist2Fwd_)                                                //
      (string(".") + maxColorDist2Bwd_opt, _params->maxColorDist2Bwd_)                                                //
      (string(".") + excludeColorOutlier_opt, _params->excludeColorOutlier_)                                          //
      (string(".") + thresholdColorOutlierDist_opt, _params->thresholdColorOutlierDist_)                              //
      (string(".") + videoEncoderOccupancyPath_opt, _params->videoEncoderOccupancyPath_)                              //
      (string(".") + videoEncoderGeometryPath_opt, _params->videoEncoderGeometryPath_)                                //
      (string(".") + videoEncoderAttributePath_opt, _params->videoEncoderAttributePath_)                              //
      (string(".") + videoEncoderOccupancyCodecId_opt, _params->videoEncoderOccupancyCodecId_)                        //
      (string(".") + videoEncoderGeometryCodecId_opt, _params->videoEncoderGeometryCodecId_)                          //
      (string(".") + videoEncoderAttributeCodecId_opt, _params->videoEncoderAttributeCodecId_)                        //
      (string(".") + byteStreamVideoEncoderOccupancy_opt, _params->byteStreamVideoCoderOccupancy_)                    //
      (string(".") + byteStreamVideoEncoderGeometry_opt, _params->byteStreamVideoCoderGeometry_)                      //
      (string(".") + byteStreamVideoEncoderAttribute_opt, _params->byteStreamVideoCoderAttribute_)                    //
      (string(".") + geometryQP_opt, _params->geometryQP_)                                                            //
      (string(".") + attributeQP_opt, _params->attributeQP_)                                                          //
      (string(".") + auxGeometryQP_opt, _params->auxGeometryQP_)                                                      //
      (string(".") + auxAttributeQP_opt, _params->auxAttributeQP_)                                                    //
      (string(".") + geometryConfig_opt, _params->geometryConfig_)                                                    //
      (string(".") + geometry0Config_opt, _params->geometry0Config_)                                                  //
      (string(".") + geometry1Config_opt, _params->geometry1Config_)                                                  //
      (string(".") + attributeConfig_opt, _params->attributeConfig_)                                                  //
      (string(".") + attribute0Config_opt, _params->attribute0Config_)                                                //
      (string(".") + attribute1Config_opt, _params->attribute1Config_)                                                //
      (string(".") + rawPointsPatch_opt, _params->rawPointsPatch_)                                                    //
      (string(".") + noAttributes_opt, _params->noAttributes_)                                                        //
      (string(".") + attributeVideo444_opt, _params->attributeVideo444_)                                              //
      (string(".") + useRawPointsSeparateVideo_opt, _params->useRawPointsSeparateVideo_)                              //
      (string(".") + attributeRawSeparateVideoWidth_opt, _params->attributeRawSeparateVideoWidth_)                    //
      (string(".") + geometryMPConfig_opt, _params->geometryAuxVideoConfig_)                                          //
      (string(".") + attributeMPConfig_opt, _params->attributeAuxVideoConfig_)                                        //
      (string(".") + nbThread_opt, _params->nbThread_)                                                                //
      (string(".") + keepIntermediateFiles_opt, _params->keepIntermediateFiles_)                                      //
      (string(".") + absoluteD1_opt, _params->absoluteD1_)                                                            //
      (string(".") + absoluteT1_opt, _params->absoluteT1_)                                                            //
      (string(".") + multipleStreams_opt, _params->multipleStreams_)                                                  //
      (string(".") + deltaQPD0_opt, _params->deltaQPD0_)                                                              //
      (string(".") + deltaQPD1_opt, _params->deltaQPD1_)                                                              //
      (string(".") + deltaQPT0_opt, _params->deltaQPT0_)                                                              //
      (string(".") + deltaQPT1_opt, _params->deltaQPT1_)                                                              //
      (string(".") + constrainedPack_opt, _params->constrainedPack_)                                                  //
      (string(".") + levelOfDetailX_opt, _params->levelOfDetailX_)                                                    //
      (string(".") + levelOfDetailY_opt, _params->levelOfDetailY_)                                                    //
      (string(".") + groupDilation_opt, _params->groupDilation_)                                                      //
      (string(".") + offsetLossyOM_opt, _params->offsetLossyOM_)                                                      //
      (string(".") + thresholdLossyOM_opt, _params->thresholdLossyOM_)                                                //
      (string(".") + prefilterLossyOM_opt, _params->prefilterLossyOM_)                                                //
      (string(".") + shvcLayerIndex_opt, _params->shvcLayerIndex_)                                                    //
      (string(".") + shvcRateX_opt, _params->shvcRateX_)                                                              //
      (string(".") + shvcRateY_opt, _params->shvcRateY_)                                                              //
      (string(".") + patchColorSubsampling_opt, _params->patchColorSubsampling_)                                      //
      (string(".") + maxNumRefAtalsList_opt, _params->maxNumRefAtlasList_)                                            //
      (string(".") + maxNumRefAtlasFrame_opt, _params->maxNumRefAtlasFrame_)                                          //
      (string(".") + pointLocalReconstruction_opt, _params->pointLocalReconstruction_)                                //
      (string(".") + mapCountMinus1_opt, _params->mapCountMinus1_)                                                    //
      (string(".") + singleMapPixelInterleaving_opt, _params->singleMapPixelInterleaving_)                            //
      (string(".") + removeDuplicatePoints_opt, _params->removeDuplicatePoints_)                                      //
      (string(".") + surfaceSeparation_opt, _params->surfaceSeparation_)                                              //
      (string(".") + highGradientSeparation_opt, _params->highGradientSeparation_)                                    //
      (string(".") + minGradient_opt, _params->minGradient_)                                                          //
      (string(".") + minNumHighGradientPoints_opt, _params->minNumHighGradientPoints_)                                //
      (string(".") + packingStrategy_opt, _params->packingStrategy_)                                                  //
      (string(".") + useEightOrientations_opt, _params->useEightOrientations_)                                        //
      (string(".") + safeGuardDistance_opt, _params->safeGuardDistance_)                                              //
      (string(".") + attributeBGFill_opt, _params->attributeBGFill_)                                                  //
      (string(".") + lossyRawPointsPatch_opt, _params->lossyRawPointsPatch_)                                          //
      (string(".") + minNormSumOfInvDist4MPSelection_opt, _params->minNormSumOfInvDist4MPSelection_)                  //
      (string(".") + globalPatchAllocation_opt, _params->globalPatchAllocation_)                                      //
      (string(".") + globalPackingStrategyGOF_opt, _params->globalPackingStrategyGOF_)                                //
      (string(".") + globalPackingStrategyReset_opt, _params->globalPackingStrategyReset_)                            //
      (string(".") + globalPackingStrategyThreshold_opt, _params->globalPackingStrategyThreshold_)                    //
      (string(".") + patchPrecedenceOrder_opt, _params->patchPrecedenceOrderFlag_)                                    //
      (string(".") + lowDelayEncoding_opt, _params->lowDelayEncoding_)                                                //
      (string(".") + geometryPadding_opt, _params->geometryPadding_)                                                  //
      (string(".") + apply3dMotionCompensation_opt, _params->use3dmc_)                                                //
      (string(".") + usePccRDO_opt, _params->usePccRDO_)                                                              //
      (string(".") + geometry3dCoordinatesBitdepth_opt, _params->geometry3dCoordinatesBitdepth_)                      //
      (string(".") + geometryNominal2dBitdepth_opt, _params->geometryNominal2dBitdepth_)                              //
      (string(".") + nbPlrmMode_opt, _params->plrlNumberOfModes_)                                                     //
      (string(".") + patchSize_opt, _params->patchSize_)                                                              //
      (string(".") + enhancedProjectionPlane_opt, _params->enhancedPP_)                                               //
      (string(".") + minWeightEPP_opt, _params->minWeightEPP_)                                                        //
      (string(".") + additionalProjectionPlaneMode_opt, _params->additionalProjectionPlaneMode_)                      //
      (string(".") + partialAdditionalProjectionPlane_opt, _params->partialAdditionalProjectionPlane_)                //
      (string(".") + numMaxTilePerFrame_opt, _params->numMaxTilePerFrame_)                                            //
      (string(".") + uniformPartitionSpacing_opt, _params->uniformPartitionSpacing_)                                  //
      (string(".") + tilePartitionWidth_opt, _params->tilePartitionWidth_)                                            //
      (string(".") + tilePartitionHeight_opt, _params->tilePartitionHeight_)                                          //
      (string(".") + tilePartitionWidthList_opt, _params->tilePartitionWidthList_)                                    //
      (string(".") + tilePartitionHeightList_opt, _params->tilePartitionHeightList_)                                  //
      (string(".") + tileSegmentationType_opt, _params->tileSegmentationType_)                                        //
      (string(".") + enablePointCloudPartitioning_opt, _params->enablePointCloudPartitioning_)                        //
      (string(".") + roiBoundingBoxMinX_opt, _params->roiBoundingBoxMinX_)                                            //
      (string(".") + roiBoundingBoxMaxX_opt, _params->roiBoundingBoxMaxX_)                                            //
      (string(".") + roiBoundingBoxMinY_opt, _params->roiBoundingBoxMinY_)                                            //
      (string(".") + roiBoundingBoxMaxY_opt, _params->roiBoundingBoxMaxY_)                                            //
      (string(".") + roiBoundingBoxMinZ_opt, _params->roiBoundingBoxMinZ_)                                            //
      (string(".") + roiBoundingBoxMaxZ_opt, _params->roiBoundingBoxMaxZ_)                                            //
      (string(".") + numTilesHor_opt, _params->numTilesHor_)                                                          //
      (string(".") + tileHeightToWidthRatio_opt, _params->tileHeightToWidthRatio_)                                    //
      (string(".") + numCutsAlong1stLongestAxis_opt, _params->numCutsAlong1stLongestAxis_)                            //
      (string(".") + numCutsAlong2ndLongestAxis_opt, _params->numCutsAlong2ndLongestAxis_)                            //
      (string(".") + numCutsAlong3rdLongestAxis_opt, _params->numCutsAlong3rdLongestAxis_)                            //
      (string(".") + mortonOrderSortRawPoints_opt, _params->mortonOrderSortRawPoints_)                                //
      (string(".") + pbfEnableFlag_opt, _params->pbfEnableFlag_)                                                      //
      (string(".") + pbfFilterSize_opt, _params->pbfFilterSize_)                                                      //
      (string(".") + pbfPassesCount_opt, _params->pbfPassesCount_)                                                    //
      (string(".") + pbfLog2Threshold_opt, _params->pbfLog2Threshold_)                                                //
      (string(".") + tierFlag_opt, _params->tierFlag_)                                                                //
      (string(".") + profileCodecGroupIdc_opt, _params->profileCodecGroupIdc_)                                        //
      (string(".") + profileToolsetIdc_opt, _params->profileToolsetIdc_)                                              //
      (string(".") + profileReconstructionIdc_opt, _params->profileReconstructionIdc_)                                //
      (string(".") + levelIdc_opt, _params->levelIdc_)                                                                //
      (string(".") + avcCodecIdIndex_opt, _params->avcCodecIdIndex_)                                                  //
      (string(".") + hevcCodecIdIndex_opt, _params->hevcCodecIdIndex_)                                                //
      (string(".") + shvcCodecIdIndex_opt, _params->shvcCodecIdIndex_)                                                //
      (string(".") + vvcCodecIdIndex_opt, _params->vvcCodecIdIndex_)                                                  //
      (string(".") + oneV3CFrameOnlyFlag_opt, _params->oneV3CFrameOnlyFlag_)                                          //
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
    case PCCColorTransform::COLOR_TRANSFORM_NONE: out << static_cast<int>(val) << " (none)"; break;
    case PCCColorTransform::COLOR_TRANSFORM_RGB_TO_YCBCR:
      out << static_cast<int>(val) << " (RGB to YCbCr (Rec.709))";
      break;
    default: out << static_cast<int>(val) << " (Unknown)"; break;
  }
  return out;
}
std::ostream& operator<<(std::ostream& out, const PCCCodecId& val) {
  switch (val) {
#ifdef USE_JMAPP_VIDEO_CODEC
    case PCCCodecId::JMAPP: out << static_cast<int>(val) << "(JMAPP)"; break;
#endif
#ifdef USE_HMAPP_VIDEO_CODEC
    case PCCCodecId::HMAPP: out << static_cast<int>(val) << "(HMAPP)"; break;
#endif
#ifdef USE_SHMAPP_VIDEO_CODEC
    case PCCCodecId::SHMAPP: out << static_cast<int>(val) << "(SHMAPP)"; break;
#endif
#ifdef USE_JMLIB_VIDEO_CODEC
    case PCCCodecId::JMLIB: out << static_cast<int>(val) << "(JMLIB)"; break;
#endif
#ifdef USE_HMLIB_VIDEO_CODEC
    case PCCCodecId::HMLIB: out << static_cast<int>(val) << "(HMLIB)"; break;
#endif
#ifdef USE_VTMLIB_VIDEO_CODEC
    case PCCCodecId::VTMLIB: out << static_cast<int>(val) << "(VTMLIB)"; break;
#endif
#ifdef USE_FFMPEG_VIDEO_CODEC
    case PCCCodecId::FFMPEG: out << static_cast<int>(val) << "(FFMPEG)"; break;
#endif
    default: out << static_cast<int>(val) << " (Unknown)"; break;
  }
  return out;
}

}  // namespace pcc
