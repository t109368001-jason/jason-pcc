#include <jpcc/encoder/JPCCEncoderTMC3Parameter.h>

namespace jpcc::encoder {

using namespace std;
using namespace po;
using namespace pcc;

#define BACKEND_TYPE_OPT ".backendType"
#define SRC_UNIT_LENGTH_OPT ".srcUnitLength"
#define SRC_UNIT_OPT ".srcUnit"
#define CODING_SCALE_OPT ".codingScale"
#define SEQUENCE_SCALE_OPT ".sequenceScale"
#define POSITION_QUANTIZATION_SCALE_OPT ".positionQuantizationScale"
#define EXTERNAL_SCALE_OPT ".externalScale"
#define GEOMETRY_AXIS_ORDER_OPT ".geometry_axis_order"
#define AUTO_SEQ_BBOX_OPT ".autoSeqBbox"
#define SEQ_ORIGIN_OPT ".seqOrigin"
#define SEQ_SIZE_WHD_OPT ".seqSizeWhd"
#define MERGE_DUPLICATED_POINTS_OPT ".mergeDuplicatedPoints"
#define PARTITION_METHOD_OPT ".partitionMethod"
#define PARTITION_OCTREE_DEPTH_OPT ".partitionOctreeDepth"
#define SLICE_MAX_POINTS_OPT ".sliceMaxPoints"
#define SLICE_MIN_POINTS_OPT ".sliceMinPoints"
#define TILE_SIZE_OPT ".tileSize"
#define CABAC_BYPASS_STREAM_ENABLED_FLAG_OPT ".cabac_bypass_stream_enabled_flag"
#define ENTROPY_CONTINUATION_ENABLED_OPT ".entropyContinuationEnabled"
#define ENFORCE_LEVEL_LIMITS_OPT ".enforceLevelLimits"
#define GEOM_TREE_TYPE_OPT ".geomTreeType"
#define QTBT_ENABLED_OPT ".qtbtEnabled"
#define MAX_NUM_QTBT_BEFORE_OT_OPT ".maxNumQtBtBeforeOt"
#define MIN_QTBT_SIZE_LOG2_OPT ".minQtbtSizeLog2"
#define NUM_OCTREE_ENTROPY_STREAMS_OPT ".numOctreeEntropyStreams"
#define BITWISE_OCCUPANCY_CODING_OPT ".bitwiseOccupancyCoding"
#define NEIGHBOUR_AVAIL_BOUNDARY_LOG2_OPT ".neighbourAvailBoundaryLog2"
#define INFERRED_DIRECT_CODING_MODE_OPT ".inferredDirectCodingMode"
#define JOINT_TWO_POINT_IDCM_OPT ".jointTwoPointIdcm"
#define ADJACENT_CHILD_CONTEXTUALIZATION_OPT ".adjacentChildContextualization"
#define INTRA_PRED_MAX_NODE_SIZE_LOG2_OPT ".intra_pred_max_node_size_log2"
#define PLANAR_ENABLED_OPT ".planarEnabled"
#define PLANAR_MODE_THRESHOLD0_OPT ".planarModeThreshold0"
#define PLANAR_MODE_THRESHOLD1_OPT ".planarModeThreshold1"
#define PLANAR_MODE_THRESHOLD2_OPT ".planarModeThreshold2"
#define PLANAR_MODE_IDCM_USE_OPT ".planarModeIdcmUse"
#define TRISOUP_NODE_SIZE_LOG2_OPT ".trisoupNodeSizeLog2"
#define TRISOUP_SAMPLING_VALUE_OPT ".trisoup_sampling_value"
#define POSITION_QUANTISATION_ENABLED_OPT ".positionQuantisationEnabled"
#define POSITION_QUANTISATION_METHOD_OPT ".positionQuantisationMethod"
#define POSITION_QP_MULTIPLIER_LOG2 ".positionQpMultiplierLog2"
#define POSITION_BASE_QP_OPT ".positionBaseQp"
#define POSITION_IDCM_QP_OPT ".positionIdcmQp"
#define POSITION_SLICE_QP_OFFSET_OPT ".positionSliceQpOffset"
#define POSITION_QUANTISATION_OCTREE_SIZE_LOG2_OPT ".positionQuantisationOctreeSizeLog2"
#define POSITION_QUANTISATION_OCTREE_DEPTH_OPT ".positionQuantisationOctreeDepth"
#define POSITION_BASE_QP_FREQ_LOG2_OPT ".positionBaseQpFreqLog2"
#define POSITION_SLICE_QP_FREQ_LOG2_OPT ".positionSliceQpFreqLog2"
#define ANGULAR_ENABLED_OPT ".angularEnabled"
#define LIDAR_HEAD_POSITION_OPT ".lidarHeadPosition"
#define NUM_LASERS_OPT ".numLasers"
#define LASERS_THETA_OPT ".lasersTheta"
#define LASERS_Z_OPT ".lasersZ"
#define LASERS_NUM_PHI_PER_TURN_OPT ".lasersNumPhiPerTurn"
#define PLANAR_BUFFER_DISABLED_OPT ".planarBufferDisabled"
#define PRED_GEOM_AZIMUTH_QUANTIZATION_OPT ".predGeomAzimuthQuantization"
#define POSITION_AZIMUTH_SCALE_LOG2_OPT ".positionAzimuthScaleLog2"
#define POSITION_AZIMUTH_SPEED_OPT ".positionAzimuthSpeed"
#define POSITION_RADIUS_INV_SCALE_LOG2_OPT ".positionRadiusInvScaleLog2"
#define PRED_GEOM_SORT_OPT ".predGeomSort"
#define PRED_GEOM_AZIMUTH_SORT_PRECISION_OPT ".predGeomAzimuthSortPrecision"
#define PRED_GEOM_TREE_PTS_MAX_OPT ".predGeomTreePtsMax"
#define POINT_COUNT_METADATA_OPT ".pointCountMetadata"
#define ATTR_SPHERICAL_MAX_LOG2_OPT ".attrSphericalMaxLog2"
#define RECOLOUR_SEARCH_RANGE_OPT ".recolourSearchRange"
#define RECOLOUR_NUM_NEIGHBOURS_FWD_OPT ".recolourNumNeighboursFwd"
#define RECOLOUR_NUM_NEIGHBOURS_BWD_OPT ".recolourNumNeighboursBwd"
#define RECOLOUR_USE_DIST_WEIGHTED_AVG_FWD_OPT ".recolourUseDistWeightedAvgFwd"
#define RECOLOUR_USE_DIST_WEIGHTED_AVG_BWD_OPT ".recolourUseDistWeightedAvgBwd"
#define RECOLOUR_SKIP_AVG_IF_IDENTICAL_SOURCE_POINT_PRESENT_FWD_OPT ".recolourSkipAvgIfIdenticalSourcePointPresentFwd"
#define RECOLOUR_SKIP_AVG_IF_IDENTICAL_SOURCE_POINT_PRESENT_BWD_OPT ".recolourSkipAvgIfIdenticalSourcePointPresentBwd"
#define RECOLOUR_DIST_OFFSET_FWD_OPT ".recolourDistOffsetFwd"
#define RECOLOUR_DIST_OFFSET_BWD_OPT ".recolourDistOffsetBwd"
#define RECOLOUR_MAX_GEOMETRY_DIST2_FWD_OPT ".recolourMaxGeometryDist2Fwd"
#define RECOLOUR_MAX_GEOMETRY_DIST2_BWD_OPT ".recolourMaxGeometryDist2Bwd"
#define RECOLOUR_MAX_ATTRIBUTE_DIST2_FWD_OPT ".recolourMaxAttributeDist2Fwd"
#define RECOLOUR_MAX_ATTRIBUTE_DIST2_BWD_OPT ".recolourMaxAttributeDist2Bwd"

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC3Parameter::JPCCEncoderTMC3Parameter() :
    JPCCEncoderTMC3Parameter(JPCC_ENCODER_TMC3_OPT_PREFIX, __FUNCTION__) {}

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC3Parameter::JPCCEncoderTMC3Parameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption), EncoderParams() {
  setDefault();
  opts_.add_options()                                                                                  //
      (string(prefix_ + SRC_UNIT_LENGTH_OPT).c_str(), value<double>(&this->srcUnitLength),             //
       "Length of source point cloud x,y,z unit vectors in srcUnits")                                  //
      (string(prefix_ + SRC_UNIT_OPT).c_str(), value<ScaleUnit>(&this->sps.seq_geom_scale_unit_flag),  //
       " 0: dimensionless\n"
       " 1: metres")                                                                      //
      (string(prefix_ + CODING_SCALE_OPT).c_str(), value<double>(&this->codedGeomScale),  //
       "Scale used to represent coded geometry. Relative to inputScale")                  //
      (string(prefix_ + SEQUENCE_SCALE_OPT).c_str(), value<double>(&this->seqGeomScale),  //
       "Scale used to obtain sequence coordinate system. Relative to inputScale")         //
      // Alias for compatibility with old name.
      (string(prefix_ + POSITION_QUANTIZATION_SCALE_OPT).c_str(),                     //
       value<double>(&this->seqGeomScale),                                            //
       "(deprecated)")                                                                //
      (string(prefix_ + EXTERNAL_SCALE_OPT).c_str(),                                  //
       value<double>(&this->extGeomScale),                                            //
       "Scale used to define external coordinate system.\n"                           //
       "Meaningless when srcUnit = metres\n"                                          //
       "  0: Use srcUnitLength\n"                                                     //
       " >0: Relative to inputScale")                                                 //
      (string(prefix_ + GEOMETRY_AXIS_ORDER_OPT).c_str(),                             //
       value<AxisOrder>(&this->sps.geometry_axis_order),                              //
       "Sets the geometry axis coding order:\n"                                       //
       "  0: (zyx)\n  1: (xyz)\n  2: (xzy)\n"                                         //
       "  3: (yzx)\n  4: (zyx)\n  5: (zxy)\n"                                         //
       "  6: (yxz)\n  7: (xyz)")                                                      //
      (string(prefix_ + AUTO_SEQ_BBOX_OPT).c_str(), value<bool>(&this->autoSeqBbox),  //
       "Calculate seqOrigin and seqSizeWhd automatically.")                           //
      // NB: the underlying variable is in STV order.
      //     Conversion happens during argument sanitization.
      (string(prefix_ + SEQ_ORIGIN_OPT).c_str(), value<Vec3<int>>(&this->sps.seqBoundingBoxOrigin),         //
       "Origin (x,y,z) of the sequence bounding box (in input coordinate system). Requires autoSeqBbox=0")  //
      // NB: the underlying variable is in STV order.
      //     Conversion happens during argument sanitization.
      (string(prefix_ + SEQ_SIZE_WHD_OPT).c_str(), value<Vec3<int>>(&this->sps.seqBoundingBoxSize),  //
       "Size of the sequence bounding box (in input coordinate system). Requires autoSeqBbox=0")     //
      (string(prefix_ + MERGE_DUPLICATED_POINTS_OPT).c_str(),                                        //
       value<bool>(&this->gps.geom_unique_points_flag),                                              //
       "Enables removal of duplicated points")                                                       //
      (string(prefix_ + PARTITION_METHOD_OPT).c_str(),                                               //
       value<PartitionMethod>(&this->partition.method),                                              //
       "Method used to partition input point cloud into slices/tiles:\n"                             //
       "  0: none\n"                                                                                 //
       "  2: n Uniform-geometry partition bins along the longest edge\n"                             //
       "  3: Uniform geometry partition at n octree depth\n"                                         //
       "  4: Uniform square partition\n"                                                             //
       "  5: n-point spans of input")                                                                //
      (string(prefix_ + PARTITION_OCTREE_DEPTH_OPT).c_str(),                                         //
       value<int>(&this->partition.octreeDepth),                                                     //
       "Depth of octree partition for partitionMethod=4")                                            //
      (string(prefix_ + SLICE_MAX_POINTS_OPT).c_str(), value<int>(&this->partition.sliceMaxPoints),  //
       "Maximum number of points per slice")                                                         //
      (string(prefix_ + SLICE_MIN_POINTS_OPT).c_str(), value<int>(&this->partition.sliceMinPoints),  //
       "Minimum number of points per slice (soft limit)")                                            //
      (string(prefix_ + TILE_SIZE_OPT).c_str(), value<int>(&this->partition.tileSize),               //
       "Partition input into cubic tiles of given size")                                             //
      (string(prefix_ + CABAC_BYPASS_STREAM_ENABLED_FLAG_OPT).c_str(),                               //
       value<bool>(&this->sps.cabac_bypass_stream_enabled_flag),                                     //
       "Controls coding method for ep(bypass) bins")                                                 //
      (string(prefix_ + ENTROPY_CONTINUATION_ENABLED_OPT).c_str(),                                   //
       value<bool>(&this->sps.entropy_continuation_enabled_flag),                                    //
       "Propagate context state between slices")                                                     //
      (string(prefix_ + ENFORCE_LEVEL_LIMITS_OPT).c_str(), value<bool>(&this->enforceLevelLimits),   //
       "Abort if level limits exceeded")                                                             //
      (string(prefix_ + GEOM_TREE_TYPE_OPT).c_str(), value<bool>(&this->gps.predgeom_enabled_flag),  //
       "Selects the tree coding method:\n"                                                           //
       "  0: octree\n"                                                                               //
       "  1: predictive")                                                                            //
      (string(prefix_ + QTBT_ENABLED_OPT).c_str(), value<bool>(&this->gps.qtbt_enabled_flag),        //
       "Enables non-cubic geometry bounding box")                                                    //
      (string(prefix_ + MAX_NUM_QTBT_BEFORE_OT_OPT).c_str(),                                         //
       value<int>(&this->geom.qtbt.maxNumQtBtBeforeOt),                                              //
       "Max number of qtbt partitions before ot")                                                    //
      (string(prefix_ + MIN_QTBT_SIZE_LOG2_OPT).c_str(),                                             //
       value<int>(&this->geom.qtbt.minQtbtSizeLog2),                                                 //
       "Minimum size of qtbt partitions")                                                            //
      (string(prefix_ + NUM_OCTREE_ENTROPY_STREAMS_OPT).c_str(),                                     //
                                                                  // NB: this is adjusted by minus 1 after the arguments
                                                                  // are parsed
       value<int>(&this->gbh.geom_stream_cnt_minus1),              //
       "Number of entropy streams for octree coding")              //
      (string(prefix_ + BITWISE_OCCUPANCY_CODING_OPT).c_str(),     //
       value<bool>(&this->gps.bitwise_occupancy_coding_flag),      //
       "Selects between bitwise and bytewise occupancy coding:\n"  //
       "  0: bytewise\n"                                           //
       "  1: bitwise")                                             //
      (string(prefix_ + NEIGHBOUR_AVAIL_BOUNDARY_LOG2_OPT).c_str(),
       // NB: this is adjusted by minus 1 after the
       // arguments are parsed
       value<int>(&this->gps.neighbour_avail_boundary_log2_minus1),          //
       "Defines the avaliability volume for neighbour occupancy lookups:\n"  //
       "<2: Limited to sibling nodes only")                                  //
      (string(prefix_ + INFERRED_DIRECT_CODING_MODE_OPT).c_str(),            //
       value<int>(&this->gps.inferred_direct_coding_mode),                   //
       "Early termination of the geometry octree for isolated points:"       //
       " 0: disabled\n"                                                      //
       " 1: fully constrained\n"                                             //
       " 2: partially constrained\n"                                         //
       " 3: unconstrained (fastest)")                                        //
      (string(prefix_ + JOINT_TWO_POINT_IDCM_OPT).c_str(),                   //
       value<bool>(&this->gps.joint_2pt_idcm_enabled_flag),                  //
       "Jointly code common prefix of two IDCM points")                      //
      (string(prefix_ + ADJACENT_CHILD_CONTEXTUALIZATION_OPT).c_str(),
       value<bool>(&this->gps.adjacent_child_contextualization_enabled_flag),  //
       "Occupancy contextualization using neighbouring adjacent children")     //
      (string(prefix_ + INTRA_PRED_MAX_NODE_SIZE_LOG2_OPT).c_str(),
       value<int>(&this->gps.intra_pred_max_node_size_log2),                                                        //
       "octree nodesizes eligible for occupancy intra prediction")                                                  //
      (string(prefix_ + PLANAR_ENABLED_OPT).c_str(), value<bool>(&this->gps.geom_planar_mode_enabled_flag),         //
       "Use planar mode for geometry coding")                                                                       //
      (string(prefix_ + PLANAR_MODE_THRESHOLD0_OPT).c_str(), value<int>(&this->gps.geom_planar_threshold0),         //
       "Activation threshold (0-127) of first planar mode. Lower values imply more use of the first planar mode")   //
      (string(prefix_ + PLANAR_MODE_THRESHOLD1_OPT).c_str(),                                                        //
       value<int>(&this->gps.geom_planar_threshold1),                                                               //
       "Activation threshold (0-127) of second planar mode. Lower values imply more use of the first planar mode")  //
      (string(prefix_ + PLANAR_MODE_THRESHOLD2_OPT).c_str(),                                                        //
       value<int>(&this->gps.geom_planar_threshold2),                                                               //
       "Activation threshold (0-127) of third planar mode. Lower values imply more use of the third planar mode")   //
      (string(prefix_ + PLANAR_MODE_IDCM_USE_OPT)
           .c_str(),                                  //
                                                      // NB: this is adjusted by minus1 after the arguments are parsed
       value<int>(&this->gps.geom_idcm_rate_minus1),  //
       "Degree (1/32%) of IDCM activation when planar mode is enabled\n"             //
       "  0 => never, 32 => always")                                                 //
      (string(prefix_ + TRISOUP_NODE_SIZE_LOG2_OPT).c_str(),                         //
       value<vector<int>>(&this->trisoupNodeSizesLog2),                              //
       "Node size for surface triangulation\n"                                       //
       " <2: disabled")                                                              //
      (string(prefix_ + TRISOUP_SAMPLING_VALUE_OPT).c_str(),                         //
       value<int>(&this->gps.trisoup_sampling_value),                                //
       "Trisoup voxelisation sampling rate\n"                                        //
       "  0: automatic")                                                             //
      (string(prefix_ + POSITION_QUANTISATION_ENABLED_OPT).c_str(),                  //
       value<bool>(&this->gps.geom_scaling_enabled_flag),                            //
       "Enable in-loop quantisation of positions")                                   //
      (string(prefix_ + POSITION_QUANTISATION_METHOD_OPT).c_str(),                   //
       value<OctreeEncOpts::QpMethod>(&this->geom.qpMethod),                         //
       "Method used to determine per-node QP:\n"                                     //
       "  0: uniform\n"                                                              //
       "  1: random\n"                                                               //
       "  2: by node point density")                                                 //
      (string(prefix_ + POSITION_QP_MULTIPLIER_LOG2).c_str(),                        //
       value<int>(&this->gps.geom_qp_multiplier_log2),                               //
       "Granularity of QP to step size mapping:\n"                                   //
       "  n: 2^n QPs per doubling interval, n in 0..3")                              //
      (string(prefix_ + POSITION_BASE_QP_OPT).c_str(),                               //
       value<int>(&this->gps.geom_base_qp),                                          //
       "Base QP used in position quantisation (0 = lossless)")                       //
      (string(prefix_ + POSITION_IDCM_QP_OPT).c_str(), value<int>(&this->idcmQp),    //
       "QP used in position quantisation of IDCM nodes")                             //
      (string(prefix_ + POSITION_SLICE_QP_OFFSET_OPT).c_str(),                       //
       value<int>(&this->gbh.geom_slice_qp_offset),                                  //
       "Per-slice QP offset used in position quantisation")                          //
      (string(prefix_ + POSITION_QUANTISATION_OCTREE_SIZE_LOG2_OPT).c_str(),         //
       value<int>(&this->geom.qpOffsetNodeSizeLog2),                                 //
       "Octree node size used for signalling position QP offsets (-1 => disabled)")  //
      (string(prefix_ + POSITION_QUANTISATION_OCTREE_DEPTH_OPT).c_str(),             //
       value<int>(&this->geom.qpOffsetDepth),                                        //
       "Octree depth used for signalling position QP offsets (-1 => disabled)")      //
      (string(prefix_ + POSITION_BASE_QP_FREQ_LOG2_OPT).c_str(),                     //
       value<int>(&this->gps.geom_qp_offset_intvl_log2),                             //
       "Frequency of sending QP offsets in predictive geometry coding")              //
      // NB: this will be corrected to be relative to base value later
      (string(prefix_ + POSITION_SLICE_QP_FREQ_LOG2_OPT).c_str(),
       value<int>(&this->gbh.geom_qp_offset_intvl_log2_delta),                                                 //
       "Frequency of sending QP offsets in predictive geometry coding")                                        //
      (string(prefix_ + ANGULAR_ENABLED_OPT).c_str(), value<bool>(&this->gps.geom_angular_mode_enabled_flag),  //
       "Controls angular contextualisation of occupancy")                                                      //
      // NB: the underlying variable is in STV order.
      //     Conversion happens during argument sanitization.
      (string(prefix_ + LIDAR_HEAD_POSITION_OPT).c_str(), value<Vec3<int>>(&this->gps.gpsAngularOrigin),            //
       "laser head position (x,y,z) in angular mode")                                                               //
      (string(prefix_ + NUM_LASERS_OPT).c_str(), value<int>(&this->numLasers),                                      //
       "Number of lasers in angular mode")                                                                          //
      (string(prefix_ + LASERS_THETA_OPT).c_str(), value<vector<double>>(&this->lasersTheta),                       //
       "Vertical laser angle in angular mode")                                                                      //
      (string(prefix_ + LASERS_Z_OPT).c_str(), value<vector<double>>(&this->lasersZ),                               //
       "Vertical laser offset in angular mode")                                                                     //
      (string(prefix_ + LASERS_NUM_PHI_PER_TURN_OPT).c_str(), value<vector<int>>(&this->gps.angularNumPhiPerTurn),  //
       "Number of sampling poisitions in a complete laser turn in angular mode")                                    //
      (string(prefix_ + PLANAR_BUFFER_DISABLED_OPT).c_str(),                                                        //
       value<bool>(&this->gps.planar_buffer_disabled_flag),                                                         //
       "Disable planar buffer (when angular mode is enabled)")                                                      //
      (string(prefix_ + PRED_GEOM_AZIMUTH_QUANTIZATION_OPT).c_str(),                                                //
       value<bool>(&this->gps.azimuth_scaling_enabled_flag),                                                        //
       "Quantize azimuth according to radius in predictive geometry coding")                                        //
      (string(prefix_ + POSITION_AZIMUTH_SCALE_LOG2_OPT).c_str(),                                                   //
       value<int>(&this->gps.geom_angular_azimuth_scale_log2_minus11),                                              //
       "Additional bits to represent azimuth angle in predictive geometry coding")                                  //
      // NB: this will be corrected to be minus 1 later
      (string(prefix_ + POSITION_AZIMUTH_SPEED_OPT).c_str(),                                                         //
       value<int>(&this->gps.geom_angular_azimuth_speed_minus1),                                                     //
       "Scale factor applied to azimuth angle in predictive geometry coding")                                        //
      (string(prefix_ + POSITION_RADIUS_INV_SCALE_LOG2_OPT).c_str(),                                                 //
       value<int>(&this->gps.geom_angular_radius_inv_scale_log2),                                                    //
       "Inverse scale factor applied to radius in predictive geometry coding")                                       //
      (string(prefix_ + PRED_GEOM_SORT_OPT).c_str(), value<PredGeomEncOpts::SortMode>(&this->predGeom.sortMode),     //
       "Predictive geometry tree construction order")                                                                //
      (string(prefix_ + PRED_GEOM_AZIMUTH_SORT_PRECISION_OPT).c_str(),                                               //
       value<float>(&this->predGeom.azimuthSortRecipBinWidth),                                                       //
       "Reciprocal precision used in azimuthal sorting for tree construction")                                       //
      (string(prefix_ + PRED_GEOM_TREE_PTS_MAX_OPT).c_str(), value<int>(&this->predGeom.maxPtsPerTree),              //
       "Maximum number of points per predictive geometry tree")                                                      //
      (string(prefix_ + POINT_COUNT_METADATA_OPT).c_str(),                                                           //
       value<bool>(&this->gps.octree_point_count_list_present_flag),                                                 //
       "Add octree layer point count metadata")                                                                      //
      (string(prefix_ + ATTR_SPHERICAL_MAX_LOG2_OPT).c_str(), value<int>(&this->attrSphericalMaxLog2),               //
       "Override spherical coordinate normalisation factor")                                                         //
      (string(prefix_ + RECOLOUR_SEARCH_RANGE_OPT).c_str(), value<int>(&this->recolour.searchRange), "")             //
      (string(prefix_ + RECOLOUR_NUM_NEIGHBOURS_FWD_OPT).c_str(), value<int>(&this->recolour.numNeighboursFwd), "")  //
      (string(prefix_ + RECOLOUR_NUM_NEIGHBOURS_BWD_OPT).c_str(), value<int>(&this->recolour.numNeighboursBwd), "")  //
      (string(prefix_ + RECOLOUR_USE_DIST_WEIGHTED_AVG_FWD_OPT).c_str(),                                             //
       value<bool>(&this->recolour.useDistWeightedAvgFwd),                                                           //
       "")                                                                                                           //
      (string(prefix_ + RECOLOUR_USE_DIST_WEIGHTED_AVG_BWD_OPT).c_str(),                                             //
       value<bool>(&this->recolour.useDistWeightedAvgBwd),                                                           //
       "")                                                                                                           //
      (string(prefix_ + RECOLOUR_SKIP_AVG_IF_IDENTICAL_SOURCE_POINT_PRESENT_FWD_OPT).c_str(),                        //
       value<bool>(&this->recolour.skipAvgIfIdenticalSourcePointPresentFwd), "")                                     //
      (string(prefix_ + RECOLOUR_SKIP_AVG_IF_IDENTICAL_SOURCE_POINT_PRESENT_BWD_OPT).c_str(),                        //
       value<bool>(&this->recolour.skipAvgIfIdenticalSourcePointPresentBwd), "")                                     //
      (string(prefix_ + RECOLOUR_DIST_OFFSET_FWD_OPT).c_str(), value<double>(&this->recolour.distOffsetFwd), "")     //
      (string(prefix_ + RECOLOUR_DIST_OFFSET_BWD_OPT).c_str(), value<double>(&this->recolour.distOffsetBwd), "")     //
      (string(prefix_ + RECOLOUR_MAX_GEOMETRY_DIST2_FWD_OPT).c_str(),                                                //
       value<double>(&this->recolour.maxGeometryDist2Fwd),                                                           //
       "")                                                                                                           //
      (string(prefix_ + RECOLOUR_MAX_GEOMETRY_DIST2_BWD_OPT).c_str(),                                                //
       value<double>(&this->recolour.maxGeometryDist2Bwd),                                                           //
       "")                                                                                                           //
      (string(prefix_ + RECOLOUR_MAX_ATTRIBUTE_DIST2_FWD_OPT).c_str(),                                               //
       value<double>(&this->recolour.maxAttributeDist2Fwd),                                                          //
       "")                                                                                                           //
      (string(prefix_ + RECOLOUR_MAX_ATTRIBUTE_DIST2_BWD_OPT).c_str(),                                               //
       value<double>(&this->recolour.maxAttributeDist2Bwd),                                                          //
       "")                                                                                                           //
      ;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC3Parameter::setDefault() {
  srcUnitLength                                     = 1.;
  sps.seq_geom_scale_unit_flag                      = ScaleUnit::kDimensionless;
  codedGeomScale                                    = 1.;
  seqGeomScale                                      = 1.;
  extGeomScale                                      = 1.;
  sps.geometry_axis_order                           = AxisOrder::kXYZ;
  autoSeqBbox                                       = true;
  sps.seqBoundingBoxOrigin                          = {0};
  sps.seqBoundingBoxSize                            = {0};
  gps.geom_unique_points_flag                       = true;
  partition.method                                  = PartitionMethod::kUniformSquare;
  partition.octreeDepth                             = 1;
  partition.sliceMaxPoints                          = 1100000;
  partition.sliceMinPoints                          = 550000;
  partition.tileSize                                = 0;
  sps.cabac_bypass_stream_enabled_flag              = false;
  sps.entropy_continuation_enabled_flag             = false;
  enforceLevelLimits                                = true;
  gps.predgeom_enabled_flag                         = false;
  gps.qtbt_enabled_flag                             = true;
  geom.qtbt.maxNumQtBtBeforeOt                      = 4;
  geom.qtbt.minQtbtSizeLog2                         = 0;
  gbh.geom_stream_cnt_minus1                        = 1;
  gps.bitwise_occupancy_coding_flag                 = true;
  gps.neighbour_avail_boundary_log2_minus1          = 0;
  gps.inferred_direct_coding_mode                   = 1;
  gps.joint_2pt_idcm_enabled_flag                   = true;
  gps.adjacent_child_contextualization_enabled_flag = true;
  gps.intra_pred_max_node_size_log2                 = 0;
  gps.geom_planar_mode_enabled_flag                 = true;
  gps.geom_planar_threshold0                        = 77;
  gps.geom_planar_threshold1                        = 99;
  gps.geom_planar_threshold2                        = 113;
  gps.geom_idcm_rate_minus1                         = 0;
  trisoupNodeSizesLog2                              = {0};
  gps.trisoup_sampling_value                        = 0;
  gps.geom_scaling_enabled_flag                     = false;
  geom.qpMethod                                     = OctreeEncOpts::QpMethod::kUniform;
  gps.geom_qp_multiplier_log2                       = 0;
  gps.geom_base_qp                                  = 0;
  idcmQp                                            = 0;
  gbh.geom_slice_qp_offset                          = 0;
  geom.qpOffsetNodeSizeLog2                         = -1;
  geom.qpOffsetDepth                                = -1;
  gps.geom_qp_offset_intvl_log2                     = 8;
  gbh.geom_qp_offset_intvl_log2_delta               = 0;
  gps.geom_angular_mode_enabled_flag                = false;
  gps.gpsAngularOrigin                              = {0, 0, 0};
  numLasers                                         = 0;
  lasersTheta                                       = {};
  lasersZ                                           = {};
  gps.angularNumPhiPerTurn                          = {};
  gps.planar_buffer_disabled_flag                   = false;
  gps.azimuth_scaling_enabled_flag                  = true;
  gps.geom_angular_azimuth_scale_log2_minus11       = 5;
  gps.geom_angular_azimuth_speed_minus1             = 363;
  gps.geom_angular_radius_inv_scale_log2            = 0;
  predGeom.sortMode                                 = PredGeomEncOpts::kSortMorton;
  predGeom.azimuthSortRecipBinWidth                 = 0.0f;
  predGeom.maxPtsPerTree                            = 1100000;
  gps.octree_point_count_list_present_flag          = false;
  attrSphericalMaxLog2                              = 0;
  recolour.searchRange                              = 1;
  recolour.numNeighboursFwd                         = 8;
  recolour.numNeighboursBwd                         = 1;
  recolour.useDistWeightedAvgFwd                    = true;
  recolour.useDistWeightedAvgBwd                    = true;
  recolour.skipAvgIfIdenticalSourcePointPresentFwd  = true;
  recolour.skipAvgIfIdenticalSourcePointPresentBwd  = false;
  recolour.distOffsetFwd                            = 4.;
  recolour.distOffsetBwd                            = 4.;
  recolour.maxGeometryDist2Fwd                      = 1000.;
  recolour.maxGeometryDist2Bwd                      = 1000.;
  recolour.maxAttributeDist2Fwd                     = 1000.;
  recolour.maxAttributeDist2Bwd                     = 1000.;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC3Parameter::notify() {}

//////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& out, const JPCCEncoderTMC3Parameter& obj) {
  obj.coutParameters(out)                                                                            //
      (SRC_UNIT_LENGTH_OPT, obj.srcUnitLength)                                                       //
      (SRC_UNIT_OPT, obj.sps.seq_geom_scale_unit_flag)                                               //
      (CODING_SCALE_OPT, obj.codedGeomScale)                                                         //
      (SEQUENCE_SCALE_OPT, obj.seqGeomScale)                                                         //
      (POSITION_QUANTIZATION_SCALE_OPT, obj.seqGeomScale)                                            //
      (EXTERNAL_SCALE_OPT, obj.extGeomScale)                                                         //
      (GEOMETRY_AXIS_ORDER_OPT, obj.sps.geometry_axis_order)                                         //
      (AUTO_SEQ_BBOX_OPT, obj.autoSeqBbox)                                                           //
      (SEQ_ORIGIN_OPT, obj.sps.seqBoundingBoxOrigin)                                                 //
      (SEQ_SIZE_WHD_OPT, obj.sps.seqBoundingBoxSize)                                                 //
      (MERGE_DUPLICATED_POINTS_OPT, obj.gps.geom_unique_points_flag)                                 //
      (PARTITION_METHOD_OPT, obj.partition.method)                                                   //
      (PARTITION_OCTREE_DEPTH_OPT, obj.partition.octreeDepth)                                        //
      (SLICE_MAX_POINTS_OPT, obj.partition.sliceMaxPoints)                                           //
      (SLICE_MIN_POINTS_OPT, obj.partition.sliceMinPoints)                                           //
      (TILE_SIZE_OPT, obj.partition.tileSize)                                                        //
      (CABAC_BYPASS_STREAM_ENABLED_FLAG_OPT, obj.sps.cabac_bypass_stream_enabled_flag)               //
      (ENTROPY_CONTINUATION_ENABLED_OPT, obj.sps.entropy_continuation_enabled_flag)                  //
      (ENFORCE_LEVEL_LIMITS_OPT, obj.enforceLevelLimits)                                             //
      (GEOM_TREE_TYPE_OPT, obj.gps.predgeom_enabled_flag)                                            //
      (QTBT_ENABLED_OPT, obj.gps.qtbt_enabled_flag)                                                  //
      (MAX_NUM_QTBT_BEFORE_OT_OPT, obj.geom.qtbt.maxNumQtBtBeforeOt)                                 //
      (MIN_QTBT_SIZE_LOG2_OPT, obj.geom.qtbt.minQtbtSizeLog2)                                        //
      (NUM_OCTREE_ENTROPY_STREAMS_OPT, obj.gbh.geom_stream_cnt_minus1)                               //
      (BITWISE_OCCUPANCY_CODING_OPT, obj.gps.bitwise_occupancy_coding_flag)                          //
      (NEIGHBOUR_AVAIL_BOUNDARY_LOG2_OPT, obj.gps.neighbour_avail_boundary_log2_minus1)              //
      (INFERRED_DIRECT_CODING_MODE_OPT, obj.gps.inferred_direct_coding_mode)                         //
      (JOINT_TWO_POINT_IDCM_OPT, obj.gps.joint_2pt_idcm_enabled_flag)                                //
      (ADJACENT_CHILD_CONTEXTUALIZATION_OPT, obj.gps.adjacent_child_contextualization_enabled_flag)  //
      (INTRA_PRED_MAX_NODE_SIZE_LOG2_OPT, obj.gps.intra_pred_max_node_size_log2)                     //
      (PLANAR_ENABLED_OPT, obj.gps.geom_planar_mode_enabled_flag)                                    //
      (PLANAR_MODE_THRESHOLD0_OPT, obj.gps.geom_planar_threshold0)                                   //
      (PLANAR_MODE_THRESHOLD1_OPT, obj.gps.geom_planar_threshold1)                                   //
      (PLANAR_MODE_THRESHOLD2_OPT, obj.gps.geom_planar_threshold2)                                   //
      (PLANAR_MODE_IDCM_USE_OPT, obj.gps.geom_idcm_rate_minus1)                                      //
      (TRISOUP_NODE_SIZE_LOG2_OPT, obj.trisoupNodeSizesLog2)                                         //
      (TRISOUP_SAMPLING_VALUE_OPT, obj.gps.trisoup_sampling_value)                                   //
      (POSITION_QUANTISATION_ENABLED_OPT, obj.gps.geom_scaling_enabled_flag)                         //
      (POSITION_QUANTISATION_METHOD_OPT, obj.geom.qpMethod)                                          //
      (POSITION_QP_MULTIPLIER_LOG2, obj.gps.geom_qp_multiplier_log2)                                 //
      (POSITION_BASE_QP_OPT, obj.gps.geom_base_qp)                                                   //
      (POSITION_IDCM_QP_OPT, obj.idcmQp)                                                             //
      (POSITION_SLICE_QP_OFFSET_OPT, obj.gbh.geom_slice_qp_offset)                                   //
      (POSITION_QUANTISATION_OCTREE_SIZE_LOG2_OPT, obj.geom.qpOffsetNodeSizeLog2)                    //
      (POSITION_QUANTISATION_OCTREE_DEPTH_OPT, obj.geom.qpOffsetDepth)                               //
      (POSITION_BASE_QP_FREQ_LOG2_OPT, obj.gps.geom_qp_offset_intvl_log2)                            //
      (POSITION_SLICE_QP_FREQ_LOG2_OPT, obj.gbh.geom_qp_offset_intvl_log2_delta)                     //
      (ANGULAR_ENABLED_OPT, obj.gps.geom_angular_mode_enabled_flag)                                  //
      (LIDAR_HEAD_POSITION_OPT, obj.gps.gpsAngularOrigin)                                            //
      (NUM_LASERS_OPT, obj.numLasers)                                                                //
      (LASERS_THETA_OPT, obj.lasersTheta)                                                            //
      (LASERS_Z_OPT, obj.lasersZ)                                                                    //
      (LASERS_NUM_PHI_PER_TURN_OPT, obj.gps.angularNumPhiPerTurn)                                    //
      (PLANAR_BUFFER_DISABLED_OPT, obj.gps.planar_buffer_disabled_flag)                              //
      (PRED_GEOM_AZIMUTH_QUANTIZATION_OPT, obj.gps.azimuth_scaling_enabled_flag)                     //
      (POSITION_AZIMUTH_SCALE_LOG2_OPT, obj.gps.geom_angular_azimuth_scale_log2_minus11)             //
      (POSITION_AZIMUTH_SPEED_OPT, obj.gps.geom_angular_azimuth_speed_minus1)                        //
      (POSITION_RADIUS_INV_SCALE_LOG2_OPT, obj.gps.geom_angular_radius_inv_scale_log2)               //
      (PRED_GEOM_SORT_OPT, obj.predGeom.sortMode)                                                    //
      (PRED_GEOM_AZIMUTH_SORT_PRECISION_OPT, obj.predGeom.azimuthSortRecipBinWidth)                  //
      (PRED_GEOM_TREE_PTS_MAX_OPT, obj.predGeom.maxPtsPerTree)                                       //
      (POINT_COUNT_METADATA_OPT, obj.gps.octree_point_count_list_present_flag)                       //
      (ATTR_SPHERICAL_MAX_LOG2_OPT, obj.attrSphericalMaxLog2)                                        //
      (RECOLOUR_SEARCH_RANGE_OPT, obj.recolour.searchRange)                                          //
      (RECOLOUR_NUM_NEIGHBOURS_FWD_OPT, obj.recolour.numNeighboursFwd)                               //
      (RECOLOUR_NUM_NEIGHBOURS_BWD_OPT, obj.recolour.numNeighboursBwd)                               //
      (RECOLOUR_USE_DIST_WEIGHTED_AVG_FWD_OPT, obj.recolour.useDistWeightedAvgFwd)                   //
      (RECOLOUR_USE_DIST_WEIGHTED_AVG_BWD_OPT, obj.recolour.useDistWeightedAvgBwd)                   //
      (RECOLOUR_SKIP_AVG_IF_IDENTICAL_SOURCE_POINT_PRESENT_FWD_OPT,
       obj.recolour.skipAvgIfIdenticalSourcePointPresentFwd)  //
      (RECOLOUR_SKIP_AVG_IF_IDENTICAL_SOURCE_POINT_PRESENT_BWD_OPT,
       obj.recolour.skipAvgIfIdenticalSourcePointPresentBwd)                     //
      (RECOLOUR_DIST_OFFSET_FWD_OPT, obj.recolour.distOffsetFwd)                 //
      (RECOLOUR_DIST_OFFSET_BWD_OPT, obj.recolour.distOffsetBwd)                 //
      (RECOLOUR_MAX_GEOMETRY_DIST2_FWD_OPT, obj.recolour.maxGeometryDist2Fwd)    //
      (RECOLOUR_MAX_GEOMETRY_DIST2_BWD_OPT, obj.recolour.maxGeometryDist2Bwd)    //
      (RECOLOUR_MAX_ATTRIBUTE_DIST2_FWD_OPT, obj.recolour.maxAttributeDist2Fwd)  //
      (RECOLOUR_MAX_ATTRIBUTE_DIST2_BWD_OPT, obj.recolour.maxAttributeDist2Bwd)  //
      ;
  return out;
}

}  // namespace jpcc::encoder

//////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------------------------------------------
// :: Command line / config parsing helpers
template <typename T>
static std::istream& readUInt(std::istream& in, T& val) {
  unsigned int tmp;
  in >> tmp;
  val = T(tmp);
  return in;
}
namespace pcc {
static std::istream& operator>>(std::istream& in, ScaleUnit& val) {
  try {
    readUInt(in, val);
  } catch (...) {
    in.clear();
    std::string str;
    in >> str;
    val = ScaleUnit::kDimensionless;
    if (str == "metre")
      val = ScaleUnit::kMetre;
    else if (!str.empty())
      BOOST_THROW_EXCEPTION(std::runtime_error("Cannot parse unit"));
  }
  return in;
}
}  // namespace pcc
// static std::istream& operator>>(std::istream& in, OutputSystem& val) { return readUInt(in, val); }
namespace pcc {
static std::istream& operator>>(std::istream& in, ColourMatrix& val) { return readUInt(in, val); }
}  // namespace pcc
namespace pcc {
static std::istream& operator>>(std::istream& in, AxisOrder& val) { return readUInt(in, val); }
}  // namespace pcc
namespace pcc {
static std::istream& operator>>(std::istream& in, AttributeEncoding& val) { return readUInt(in, val); }
}  // namespace pcc
namespace pcc {
static std::istream& operator>>(std::istream& in, LodDecimationMethod& val) { return readUInt(in, val); }
}  // namespace pcc
namespace pcc {
static std::istream& operator>>(std::istream& in, PartitionMethod& val) { return readUInt(in, val); }
}  // namespace pcc
namespace pcc {
static std::istream& operator>>(std::istream& in, PredGeomEncOpts::SortMode& val) { return readUInt(in, val); }
}  // namespace pcc
namespace pcc {
static std::istream& operator>>(std::istream& in, OctreeEncOpts::QpMethod& val) { return readUInt(in, val); }
}  // namespace pcc
// static std::ostream& operator<<(std::ostream& out, const OutputSystem& val) {
//   switch (val) {
//     case OutputSystem::kConformance: out << "0 (Conformance)"; break;
//     case OutputSystem::kExternal: out << "1 (External)"; break;
//   }
//   return out;
// }
namespace pcc {
static std::ostream& operator<<(std::ostream& out, const ScaleUnit& val) {
  switch (val) {
    case ScaleUnit::kDimensionless: out << "0 (Dimensionless)"; break;
    case ScaleUnit::kMetre: out << "1 (Metre)"; break;
  }
  return out;
}
}  // namespace pcc
namespace pcc {
static std::ostream& operator<<(std::ostream& out, const ColourMatrix& val) {
  switch (val) {
    case ColourMatrix::kIdentity: out << "0 (Identity)"; break;
    case ColourMatrix::kBt709: out << "1 (Bt709)"; break;
    case ColourMatrix::kUnspecified: out << "2 (Unspecified)"; break;
    case ColourMatrix::kReserved_3: out << "3 (Reserved)"; break;
    case ColourMatrix::kUsa47Cfr73dot682a20: out << "4 (Usa47Cfr73dot682a20)"; break;
    case ColourMatrix::kBt601: out << "5 (Bt601)"; break;
    case ColourMatrix::kSmpte170M: out << "6 (Smpte170M)"; break;
    case ColourMatrix::kSmpte240M: out << "7 (Smpte240M)"; break;
    case ColourMatrix::kYCgCo: out << "8 (kYCgCo)"; break;
    case ColourMatrix::kBt2020Ncl: out << "9 (Bt2020Ncl)"; break;
    case ColourMatrix::kBt2020Cl: out << "10 (Bt2020Cl)"; break;
    case ColourMatrix::kSmpte2085: out << "11 (Smpte2085)"; break;
    default: out << "Unknown"; break;
  }
  return out;
}
}  // namespace pcc
namespace pcc {
static std::ostream& operator<<(std::ostream& out, const AxisOrder& val) {
  switch (val) {
    case AxisOrder::kZYX: out << "0 (zyx)"; break;
    case AxisOrder::kXYZ: out << "1 (xyz)"; break;
    case AxisOrder::kXZY: out << "2 (xzy)"; break;
    case AxisOrder::kYZX: out << "3 (yzx)"; break;
    case AxisOrder::kZYX_4: out << "4 (zyx)"; break;
    case AxisOrder::kZXY: out << "5 (zxy)"; break;
    case AxisOrder::kYXZ: out << "6 (yxz)"; break;
    case AxisOrder::kXYZ_7: out << "7 (xyz)"; break;
  }
  return out;
}
}  // namespace pcc
namespace pcc {
static std::ostream& operator<<(std::ostream& out, const AttributeEncoding& val) {
  switch (val) {
    case AttributeEncoding::kRAHTransform: out << "0 (RAHT)"; break;
    case AttributeEncoding::kPredictingTransform: out << "1 (Pred)"; break;
    case AttributeEncoding::kLiftingTransform: out << "2 (Lift)"; break;
    case AttributeEncoding::kRaw: out << "3 (Raw)"; break;
  }
  return out;
}
}  // namespace pcc
namespace pcc {
static std::ostream& operator<<(std::ostream& out, const LodDecimationMethod& val) {
  switch (val) {
    case LodDecimationMethod::kNone: out << "0 (None)"; break;
    case LodDecimationMethod::kPeriodic: out << "1 (Periodic)"; break;
    case LodDecimationMethod::kCentroid: out << "2 (Centroid)"; break;
  }
  return out;
}
}  // namespace pcc
namespace pcc {
static std::ostream& operator<<(std::ostream& out, const PartitionMethod& val) {
  switch (val) {
    case PartitionMethod::kNone: out << "0 (None)"; break;
    case PartitionMethod::kUniformGeom: out << "2 (UniformGeom)"; break;
    case PartitionMethod::kOctreeUniform: out << "3 (UniformOctree)"; break;
    case PartitionMethod::kUniformSquare: out << "4 (UniformSquare)"; break;
    case PartitionMethod::kNpoints: out << "5 (NPointSpans)"; break;
    default: out << int(val) << " (Unknown)"; break;
  }
  return out;
}
}  // namespace pcc
namespace pcc {
static std::ostream& operator<<(std::ostream& out, const PredGeomEncOpts::SortMode& val) {
  switch (val) {
    using SortMode = PredGeomEncOpts::SortMode;
    case SortMode::kNoSort: out << int(val) << " (None)"; break;
    case SortMode::kSortMorton: out << int(val) << " (Morton)"; break;
    case SortMode::kSortAzimuth: out << int(val) << " (Azimuth)"; break;
    case SortMode::kSortRadius: out << int(val) << " (Radius)"; break;
    case SortMode::kSortLaserAngle: out << int(val) << " (LaserAngle)"; break;
    default: out << int(val) << " (Unknown)"; break;
  }
  return out;
}
}  // namespace pcc
namespace pcc {
static std::ostream& operator<<(std::ostream& out, const OctreeEncOpts::QpMethod& val) {
  switch (val) {
    using Method = OctreeEncOpts::QpMethod;
    case Method::kUniform: out << int(val) << " (Uniform)"; break;
    case Method::kRandom: out << int(val) << " (Random)"; break;
    case Method::kByDensity: out << int(val) << " (ByDensity)"; break;
    default: out << int(val) << " (Unknown)"; break;
  }
  return out;
}
}  // namespace pcc
// namespace df {
// namespace program_options_lite {
// template <typename T>
// struct option_detail<pcc::Vec3<T>> {
//   static constexpr bool is_container  = true;
//   static constexpr bool is_fixed_size = true;
//   typedef T*            output_iterator;
//
//   static void            clear(pcc::Vec3<T>& container){};
//   static output_iterator make_output_iterator(Vec3<T>& container) { return &container[0]; }
// };
// }  // namespace program_options_lite
// }  // namespace df
//---------------------------------------------------------------------------
