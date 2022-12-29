#include <jpcc/encoder/JPCCEncoderTMC3Parameter.h>

#include <PCCMath.h>
#include <PCCTMC3Encoder.h>

using namespace std;
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
// :: Command line / config parsing helpers
namespace pcc {
template <typename T>
static std::istream& readUInt(std::istream& in, T& val);
static std::istream& operator>>(std::istream& in, ScaleUnit& val);
static std::istream& operator>>(std::istream& in, AxisOrder& val);
static std::istream& operator>>(std::istream& in, PartitionMethod& val);
static std::istream& operator>>(std::istream& in, PredGeomEncOpts::SortMode& val);
static std::istream& operator>>(std::istream& in, OctreeEncOpts::QpMethod& val);
static std::ostream& operator<<(std::ostream& out, const ScaleUnit& val);
static std::ostream& operator<<(std::ostream& out, const AxisOrder& val);
static std::ostream& operator<<(std::ostream& out, const PartitionMethod& val);
static std::ostream& operator<<(std::ostream& out, const PredGeomEncOpts::SortMode& val);
static std::ostream& operator<<(std::ostream& out, const OctreeEncOpts::QpMethod& val);
}  // namespace pcc

namespace jpcc::encoder {

using namespace po;

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC3Parameter::JPCCEncoderTMC3Parameter() :
    JPCCEncoderTMC3Parameter(JPCC_ENCODER_TMC3_OPT_PREFIX, __FUNCTION__) {}

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC3Parameter::JPCCEncoderTMC3Parameter(  // NOLINT(cppcoreguidelines-pro-type-member-init)
    const string& prefix,
    const string& caption) :
    Parameter(prefix, caption) {
  params_      = std::make_shared<pcc::EncoderParams>(pcc::EncoderParams{0});
  auto _params = std::static_pointer_cast<pcc::EncoderParams>(params_);
  setDefault();
  opts_.add_options()                                                                                     //
      (string(prefix_ + SRC_UNIT_LENGTH_OPT).c_str(), value<double>(&_params->srcUnitLength),             //
       "Length of source point cloud x,y,z unit vectors in srcUnits")                                     //
      (string(prefix_ + SRC_UNIT_OPT).c_str(), value<ScaleUnit>(&_params->sps.seq_geom_scale_unit_flag),  //
       " 0: dimensionless\n"
       " 1: metres")                                                                         //
      (string(prefix_ + CODING_SCALE_OPT).c_str(), value<double>(&_params->codedGeomScale),  //
       "Scale used to represent coded geometry. Relative to inputScale")                     //
      (string(prefix_ + SEQUENCE_SCALE_OPT).c_str(), value<double>(&_params->seqGeomScale),  //
       "Scale used to obtain sequence coordinate system. Relative to inputScale")            //
      // Alias for compatibility with old name.
      (string(prefix_ + POSITION_QUANTIZATION_SCALE_OPT).c_str(),                        //
       value<double>(&_params->seqGeomScale),                                            //
       "(deprecated)")                                                                   //
      (string(prefix_ + EXTERNAL_SCALE_OPT).c_str(),                                     //
       value<double>(&_params->extGeomScale),                                            //
       "Scale used to define external coordinate system.\n"                              //
       "Meaningless when srcUnit = metres\n"                                             //
       "  0: Use srcUnitLength\n"                                                        //
       " >0: Relative to inputScale")                                                    //
      (string(prefix_ + GEOMETRY_AXIS_ORDER_OPT).c_str(),                                //
       value<AxisOrder>(&_params->sps.geometry_axis_order),                              //
       "Sets the geometry axis coding order:\n"                                          //
       "  0: (zyx)\n  1: (xyz)\n  2: (xzy)\n"                                            //
       "  3: (yzx)\n  4: (zyx)\n  5: (zxy)\n"                                            //
       "  6: (yxz)\n  7: (xyz)")                                                         //
      (string(prefix_ + AUTO_SEQ_BBOX_OPT).c_str(), value<bool>(&_params->autoSeqBbox),  //
       "Calculate seqOrigin and seqSizeWhd automatically.")                              //
      // NB: the underlying variable is in STV order.
      //     Conversion happens during argument sanitization.
      (string(prefix_ + SEQ_ORIGIN_OPT).c_str(), value<pcc::Vec3<int>>(&_params->sps.seqBoundingBoxOrigin),  //
       "Origin (x,y,z) of the sequence bounding box (in input coordinate system). Requires autoSeqBbox=0")   //
      // NB: the underlying variable is in STV order.
      //     Conversion happens during argument sanitization.
      (string(prefix_ + SEQ_SIZE_WHD_OPT).c_str(), value<pcc::Vec3<int>>(&_params->sps.seqBoundingBoxSize),  //
       "Size of the sequence bounding box (in input coordinate system). Requires autoSeqBbox=0")             //
      (string(prefix_ + MERGE_DUPLICATED_POINTS_OPT).c_str(),                                                //
       value<bool>(&_params->gps.geom_unique_points_flag),                                                   //
       "Enables removal of duplicated points")                                                               //
      (string(prefix_ + PARTITION_METHOD_OPT).c_str(),                                                       //
       value<PartitionMethod>(&_params->partition.method),                                                   //
       "Method used to partition input point cloud into slices/tiles:\n"                                     //
       "  0: none\n"                                                                                         //
       "  2: n Uniform-geometry partition bins along the longest edge\n"                                     //
       "  3: Uniform geometry partition at n octree depth\n"                                                 //
       "  4: Uniform square partition\n"                                                                     //
       "  5: n-point spans of input")                                                                        //
      (string(prefix_ + PARTITION_OCTREE_DEPTH_OPT).c_str(),                                                 //
       value<int>(&_params->partition.octreeDepth),                                                          //
       "Depth of octree partition for partitionMethod=4")                                                    //
      (string(prefix_ + SLICE_MAX_POINTS_OPT).c_str(), value<int>(&_params->partition.sliceMaxPoints),       //
       "Maximum number of points per slice")                                                                 //
      (string(prefix_ + SLICE_MIN_POINTS_OPT).c_str(), value<int>(&_params->partition.sliceMinPoints),       //
       "Minimum number of points per slice (soft limit)")                                                    //
      (string(prefix_ + TILE_SIZE_OPT).c_str(), value<int>(&_params->partition.tileSize),                    //
       "Partition input into cubic tiles of given size")                                                     //
      (string(prefix_ + CABAC_BYPASS_STREAM_ENABLED_FLAG_OPT).c_str(),                                       //
       value<bool>(&_params->sps.cabac_bypass_stream_enabled_flag),                                          //
       "Controls coding method for ep(bypass) bins")                                                         //
      (string(prefix_ + ENTROPY_CONTINUATION_ENABLED_OPT).c_str(),                                           //
       value<bool>(&_params->sps.entropy_continuation_enabled_flag),                                         //
       "Propagate context state between slices")                                                             //
      (string(prefix_ + ENFORCE_LEVEL_LIMITS_OPT).c_str(), value<bool>(&_params->enforceLevelLimits),        //
       "Abort if level limits exceeded")                                                                     //
      (string(prefix_ + GEOM_TREE_TYPE_OPT).c_str(), value<bool>(&_params->gps.predgeom_enabled_flag),       //
       "Selects the tree coding method:\n"                                                                   //
       "  0: octree\n"                                                                                       //
       "  1: predictive")                                                                                    //
      (string(prefix_ + QTBT_ENABLED_OPT).c_str(), value<bool>(&_params->gps.qtbt_enabled_flag),             //
       "Enables non-cubic geometry bounding box")                                                            //
      (string(prefix_ + MAX_NUM_QTBT_BEFORE_OT_OPT).c_str(),                                                 //
       value<int>(&_params->geom.qtbt.maxNumQtBtBeforeOt),                                                   //
       "Max number of qtbt partitions before ot")                                                            //
      (string(prefix_ + MIN_QTBT_SIZE_LOG2_OPT).c_str(),                                                     //
       value<int>(&_params->geom.qtbt.minQtbtSizeLog2),                                                      //
       "Minimum size of qtbt partitions")                                                                    //
      (string(prefix_ + NUM_OCTREE_ENTROPY_STREAMS_OPT).c_str(),                                             //
                                                                  // NB: this is adjusted by minus 1 after the arguments
                                                                  // are parsed
       value<int>(&_params->gbh.geom_stream_cnt_minus1),           //
       "Number of entropy streams for octree coding")              //
      (string(prefix_ + BITWISE_OCCUPANCY_CODING_OPT).c_str(),     //
       value<bool>(&_params->gps.bitwise_occupancy_coding_flag),   //
       "Selects between bitwise and bytewise occupancy coding:\n"  //
       "  0: bytewise\n"                                           //
       "  1: bitwise")                                             //
      (string(prefix_ + NEIGHBOUR_AVAIL_BOUNDARY_LOG2_OPT).c_str(),
       // NB: this is adjusted by minus 1 after the
       // arguments are parsed
       value<int>(&_params->gps.neighbour_avail_boundary_log2_minus1),       //
       "Defines the avaliability volume for neighbour occupancy lookups:\n"  //
       "<2: Limited to sibling nodes only")                                  //
      (string(prefix_ + INFERRED_DIRECT_CODING_MODE_OPT).c_str(),            //
       value<int>(&_params->gps.inferred_direct_coding_mode),                //
       "Early termination of the geometry octree for isolated points:"       //
       " 0: disabled\n"                                                      //
       " 1: fully constrained\n"                                             //
       " 2: partially constrained\n"                                         //
       " 3: unconstrained (fastest)")                                        //
      (string(prefix_ + JOINT_TWO_POINT_IDCM_OPT).c_str(),                   //
       value<bool>(&_params->gps.joint_2pt_idcm_enabled_flag),               //
       "Jointly code common prefix of two IDCM points")                      //
      (string(prefix_ + ADJACENT_CHILD_CONTEXTUALIZATION_OPT).c_str(),
       value<bool>(&_params->gps.adjacent_child_contextualization_enabled_flag),  //
       "Occupancy contextualization using neighbouring adjacent children")        //
      (string(prefix_ + INTRA_PRED_MAX_NODE_SIZE_LOG2_OPT).c_str(),
       value<int>(&_params->gps.intra_pred_max_node_size_log2),                                                     //
       "octree nodesizes eligible for occupancy intra prediction")                                                  //
      (string(prefix_ + PLANAR_ENABLED_OPT).c_str(), value<bool>(&_params->gps.geom_planar_mode_enabled_flag),      //
       "Use planar mode for geometry coding")                                                                       //
      (string(prefix_ + PLANAR_MODE_THRESHOLD0_OPT).c_str(), value<int>(&_params->gps.geom_planar_threshold0),      //
       "Activation threshold (0-127) of first planar mode. Lower values imply more use of the first planar mode")   //
      (string(prefix_ + PLANAR_MODE_THRESHOLD1_OPT).c_str(),                                                        //
       value<int>(&_params->gps.geom_planar_threshold1),                                                            //
       "Activation threshold (0-127) of second planar mode. Lower values imply more use of the first planar mode")  //
      (string(prefix_ + PLANAR_MODE_THRESHOLD2_OPT).c_str(),                                                        //
       value<int>(&_params->gps.geom_planar_threshold2),                                                            //
       "Activation threshold (0-127) of third planar mode. Lower values imply more use of the third planar mode")   //
      (string(prefix_ + PLANAR_MODE_IDCM_USE_OPT)
           .c_str(),  //
                      // NB: this is adjusted by minus1 after the arguments are parsed
       value<int>(&_params->gps.geom_idcm_rate_minus1),                               //
       "Degree (1/32%) of IDCM activation when planar mode is enabled\n"              //
       "  0 => never, 32 => always")                                                  //
      (string(prefix_ + TRISOUP_NODE_SIZE_LOG2_OPT).c_str(),                          //
       value<vector<int>>(&_params->trisoupNodeSizesLog2),                            //
       "Node size for surface triangulation\n"                                        //
       " <2: disabled")                                                               //
      (string(prefix_ + TRISOUP_SAMPLING_VALUE_OPT).c_str(),                          //
       value<int>(&_params->gps.trisoup_sampling_value),                              //
       "Trisoup voxelisation sampling rate\n"                                         //
       "  0: automatic")                                                              //
      (string(prefix_ + POSITION_QUANTISATION_ENABLED_OPT).c_str(),                   //
       value<bool>(&_params->gps.geom_scaling_enabled_flag),                          //
       "Enable in-loop quantisation of positions")                                    //
      (string(prefix_ + POSITION_QUANTISATION_METHOD_OPT).c_str(),                    //
       value<OctreeEncOpts::QpMethod>(&_params->geom.qpMethod),                       //
       "Method used to determine per-node QP:\n"                                      //
       "  0: uniform\n"                                                               //
       "  1: random\n"                                                                //
       "  2: by node point density")                                                  //
      (string(prefix_ + POSITION_QP_MULTIPLIER_LOG2).c_str(),                         //
       value<int>(&_params->gps.geom_qp_multiplier_log2),                             //
       "Granularity of QP to step size mapping:\n"                                    //
       "  n: 2^n QPs per doubling interval, n in 0..3")                               //
      (string(prefix_ + POSITION_BASE_QP_OPT).c_str(),                                //
       value<int>(&_params->gps.geom_base_qp),                                        //
       "Base QP used in position quantisation (0 = lossless)")                        //
      (string(prefix_ + POSITION_IDCM_QP_OPT).c_str(), value<int>(&_params->idcmQp),  //
       "QP used in position quantisation of IDCM nodes")                              //
      (string(prefix_ + POSITION_SLICE_QP_OFFSET_OPT).c_str(),                        //
       value<int>(&_params->gbh.geom_slice_qp_offset),                                //
       "Per-slice QP offset used in position quantisation")                           //
      (string(prefix_ + POSITION_QUANTISATION_OCTREE_SIZE_LOG2_OPT).c_str(),          //
       value<int>(&_params->geom.qpOffsetNodeSizeLog2),                               //
       "Octree node size used for signalling position QP offsets (-1 => disabled)")   //
      (string(prefix_ + POSITION_QUANTISATION_OCTREE_DEPTH_OPT).c_str(),              //
       value<int>(&_params->geom.qpOffsetDepth),                                      //
       "Octree depth used for signalling position QP offsets (-1 => disabled)")       //
      (string(prefix_ + POSITION_BASE_QP_FREQ_LOG2_OPT).c_str(),                      //
       value<int>(&_params->gps.geom_qp_offset_intvl_log2),                           //
       "Frequency of sending QP offsets in predictive geometry coding")               //
      // NB: this will be corrected to be relative to base value later
      (string(prefix_ + POSITION_SLICE_QP_FREQ_LOG2_OPT).c_str(),
       value<int>(&_params->gbh.geom_qp_offset_intvl_log2_delta),                                                 //
       "Frequency of sending QP offsets in predictive geometry coding")                                           //
      (string(prefix_ + ANGULAR_ENABLED_OPT).c_str(), value<bool>(&_params->gps.geom_angular_mode_enabled_flag),  //
       "Controls angular contextualisation of occupancy")                                                         //
      // NB: the underlying variable is in STV order.
      //     Conversion happens during argument sanitization.
      (string(prefix_ + LIDAR_HEAD_POSITION_OPT).c_str(), value<pcc::Vec3<int>>(&_params->gps.gpsAngularOrigin),  //
       "laser head position (x,y,z) in angular mode")                                                             //
      (string(prefix_ + NUM_LASERS_OPT).c_str(), value<int>(&_params->numLasers),                                 //
       "Number of lasers in angular mode")                                                                        //
      (string(prefix_ + LASERS_THETA_OPT).c_str(), value<vector<double>>(&_params->lasersTheta),                  //
       "Vertical laser angle in angular mode")                                                                    //
      (string(prefix_ + LASERS_Z_OPT).c_str(), value<vector<double>>(&_params->lasersZ),                          //
       "Vertical laser offset in angular mode")                                                                   //
      (string(prefix_ + LASERS_NUM_PHI_PER_TURN_OPT).c_str(),
       value<vector<int>>(&_params->gps.angularNumPhiPerTurn),                      //
       "Number of sampling poisitions in a complete laser turn in angular mode")    //
      (string(prefix_ + PLANAR_BUFFER_DISABLED_OPT).c_str(),                        //
       value<bool>(&_params->gps.planar_buffer_disabled_flag),                      //
       "Disable planar buffer (when angular mode is enabled)")                      //
      (string(prefix_ + PRED_GEOM_AZIMUTH_QUANTIZATION_OPT).c_str(),                //
       value<bool>(&_params->gps.azimuth_scaling_enabled_flag),                     //
       "Quantize azimuth according to radius in predictive geometry coding")        //
      (string(prefix_ + POSITION_AZIMUTH_SCALE_LOG2_OPT).c_str(),                   //
       value<int>(&_params->gps.geom_angular_azimuth_scale_log2_minus11),           //
       "Additional bits to represent azimuth angle in predictive geometry coding")  //
      // NB: this will be corrected to be minus 1 later
      (string(prefix_ + POSITION_AZIMUTH_SPEED_OPT).c_str(),                                                         //
       value<int>(&_params->gps.geom_angular_azimuth_speed_minus1),                                                  //
       "Scale factor applied to azimuth angle in predictive geometry coding")                                        //
      (string(prefix_ + POSITION_RADIUS_INV_SCALE_LOG2_OPT).c_str(),                                                 //
       value<int>(&_params->gps.geom_angular_radius_inv_scale_log2),                                                 //
       "Inverse scale factor applied to radius in predictive geometry coding")                                       //
      (string(prefix_ + PRED_GEOM_SORT_OPT).c_str(), value<PredGeomEncOpts::SortMode>(&_params->predGeom.sortMode),  //
       "Predictive geometry tree construction order")                                                                //
      (string(prefix_ + PRED_GEOM_AZIMUTH_SORT_PRECISION_OPT).c_str(),                                               //
       value<float>(&_params->predGeom.azimuthSortRecipBinWidth),                                                    //
       "Reciprocal precision used in azimuthal sorting for tree construction")                                       //
      (string(prefix_ + PRED_GEOM_TREE_PTS_MAX_OPT).c_str(), value<int>(&_params->predGeom.maxPtsPerTree),           //
       "Maximum number of points per predictive geometry tree")                                                      //
      (string(prefix_ + POINT_COUNT_METADATA_OPT).c_str(),                                                           //
       value<bool>(&_params->gps.octree_point_count_list_present_flag),                                              //
       "Add octree layer point count metadata")                                                                      //
      (string(prefix_ + ATTR_SPHERICAL_MAX_LOG2_OPT).c_str(), value<int>(&_params->attrSphericalMaxLog2),            //
       "Override spherical coordinate normalisation factor")                                                         //
      (string(prefix_ + RECOLOUR_SEARCH_RANGE_OPT).c_str(), value<int>(&_params->recolour.searchRange), "")          //
      (string(prefix_ + RECOLOUR_NUM_NEIGHBOURS_FWD_OPT).c_str(), value<int>(&_params->recolour.numNeighboursFwd),
       "")  //
      (string(prefix_ + RECOLOUR_NUM_NEIGHBOURS_BWD_OPT).c_str(), value<int>(&_params->recolour.numNeighboursBwd),
       "")                                                                                                           //
      (string(prefix_ + RECOLOUR_USE_DIST_WEIGHTED_AVG_FWD_OPT).c_str(),                                             //
       value<bool>(&_params->recolour.useDistWeightedAvgFwd),                                                        //
       "")                                                                                                           //
      (string(prefix_ + RECOLOUR_USE_DIST_WEIGHTED_AVG_BWD_OPT).c_str(),                                             //
       value<bool>(&_params->recolour.useDistWeightedAvgBwd),                                                        //
       "")                                                                                                           //
      (string(prefix_ + RECOLOUR_SKIP_AVG_IF_IDENTICAL_SOURCE_POINT_PRESENT_FWD_OPT).c_str(),                        //
       value<bool>(&_params->recolour.skipAvgIfIdenticalSourcePointPresentFwd), "")                                  //
      (string(prefix_ + RECOLOUR_SKIP_AVG_IF_IDENTICAL_SOURCE_POINT_PRESENT_BWD_OPT).c_str(),                        //
       value<bool>(&_params->recolour.skipAvgIfIdenticalSourcePointPresentBwd), "")                                  //
      (string(prefix_ + RECOLOUR_DIST_OFFSET_FWD_OPT).c_str(), value<double>(&_params->recolour.distOffsetFwd), "")  //
      (string(prefix_ + RECOLOUR_DIST_OFFSET_BWD_OPT).c_str(), value<double>(&_params->recolour.distOffsetBwd), "")  //
      (string(prefix_ + RECOLOUR_MAX_GEOMETRY_DIST2_FWD_OPT).c_str(),                                                //
       value<double>(&_params->recolour.maxGeometryDist2Fwd),                                                        //
       "")                                                                                                           //
      (string(prefix_ + RECOLOUR_MAX_GEOMETRY_DIST2_BWD_OPT).c_str(),                                                //
       value<double>(&_params->recolour.maxGeometryDist2Bwd),                                                        //
       "")                                                                                                           //
      (string(prefix_ + RECOLOUR_MAX_ATTRIBUTE_DIST2_FWD_OPT).c_str(),                                               //
       value<double>(&_params->recolour.maxAttributeDist2Fwd),                                                       //
       "")                                                                                                           //
      (string(prefix_ + RECOLOUR_MAX_ATTRIBUTE_DIST2_BWD_OPT).c_str(),                                               //
       value<double>(&_params->recolour.maxAttributeDist2Bwd),                                                       //
       "")                                                                                                           //
      ;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC3Parameter::setDefault() {
  auto _params                                               = std::static_pointer_cast<pcc::EncoderParams>(params_);
  _params->srcUnitLength                                     = 1.;
  _params->sps.seq_geom_scale_unit_flag                      = ScaleUnit::kDimensionless;
  _params->codedGeomScale                                    = 1.;
  _params->seqGeomScale                                      = 1.;
  _params->extGeomScale                                      = 1.;
  _params->sps.geometry_axis_order                           = AxisOrder::kXYZ;
  _params->autoSeqBbox                                       = true;
  _params->sps.seqBoundingBoxOrigin                          = {0, 0, 0};
  _params->sps.seqBoundingBoxSize                            = {0, 0, 0};
  _params->gps.geom_unique_points_flag                       = true;
  _params->partition.method                                  = PartitionMethod::kUniformSquare;
  _params->partition.octreeDepth                             = 1;
  _params->partition.sliceMaxPoints                          = 1100000;
  _params->partition.sliceMinPoints                          = 550000;
  _params->partition.tileSize                                = 0;
  _params->sps.cabac_bypass_stream_enabled_flag              = false;
  _params->sps.entropy_continuation_enabled_flag             = false;
  _params->enforceLevelLimits                                = true;
  _params->gps.predgeom_enabled_flag                         = false;
  _params->gps.qtbt_enabled_flag                             = true;
  _params->geom.qtbt.maxNumQtBtBeforeOt                      = 4;
  _params->geom.qtbt.minQtbtSizeLog2                         = 0;
  _params->gbh.geom_stream_cnt_minus1                        = 1;
  _params->gps.bitwise_occupancy_coding_flag                 = true;
  _params->gps.neighbour_avail_boundary_log2_minus1          = 0;
  _params->gps.inferred_direct_coding_mode                   = 1;
  _params->gps.joint_2pt_idcm_enabled_flag                   = true;
  _params->gps.adjacent_child_contextualization_enabled_flag = true;
  _params->gps.intra_pred_max_node_size_log2                 = 0;
  _params->gps.geom_planar_mode_enabled_flag                 = true;
  _params->gps.geom_planar_threshold0                        = 77;
  _params->gps.geom_planar_threshold1                        = 99;
  _params->gps.geom_planar_threshold2                        = 113;
  _params->gps.geom_idcm_rate_minus1                         = 0;
  _params->trisoupNodeSizesLog2                              = {0, 0, 0};
  _params->gps.trisoup_sampling_value                        = 0;
  _params->gps.geom_scaling_enabled_flag                     = false;
  _params->geom.qpMethod                                     = OctreeEncOpts::QpMethod::kUniform;
  _params->gps.geom_qp_multiplier_log2                       = 0;
  _params->gps.geom_base_qp                                  = 0;
  _params->idcmQp                                            = 0;
  _params->gbh.geom_slice_qp_offset                          = 0;
  _params->geom.qpOffsetNodeSizeLog2                         = -1;
  _params->geom.qpOffsetDepth                                = -1;
  _params->gps.geom_qp_offset_intvl_log2                     = 8;
  _params->gbh.geom_qp_offset_intvl_log2_delta               = 0;
  _params->gps.geom_angular_mode_enabled_flag                = false;
  _params->gps.gpsAngularOrigin                              = {0, 0, 0};
  _params->numLasers                                         = 0;
  _params->lasersTheta                                       = {};
  _params->lasersZ                                           = {};
  _params->gps.angularNumPhiPerTurn                          = {};
  _params->gps.planar_buffer_disabled_flag                   = false;
  _params->gps.azimuth_scaling_enabled_flag                  = true;
  _params->gps.geom_angular_azimuth_scale_log2_minus11       = 5;
  _params->gps.geom_angular_azimuth_speed_minus1             = 363;
  _params->gps.geom_angular_radius_inv_scale_log2            = 0;
  _params->predGeom.sortMode                                 = PredGeomEncOpts::kSortMorton;
  _params->predGeom.azimuthSortRecipBinWidth                 = 0.0f;
  _params->predGeom.maxPtsPerTree                            = 1100000;
  _params->gps.octree_point_count_list_present_flag          = false;
  _params->attrSphericalMaxLog2                              = 0;
  _params->recolour.searchRange                              = 1;
  _params->recolour.numNeighboursFwd                         = 8;
  _params->recolour.numNeighboursBwd                         = 1;
  _params->recolour.useDistWeightedAvgFwd                    = true;
  _params->recolour.useDistWeightedAvgBwd                    = true;
  _params->recolour.skipAvgIfIdenticalSourcePointPresentFwd  = true;
  _params->recolour.skipAvgIfIdenticalSourcePointPresentBwd  = false;
  _params->recolour.distOffsetFwd                            = 4.;
  _params->recolour.distOffsetBwd                            = 4.;
  _params->recolour.maxGeometryDist2Fwd                      = 1000.;
  _params->recolour.maxGeometryDist2Bwd                      = 1000.;
  _params->recolour.maxAttributeDist2Fwd                     = 1000.;
  _params->recolour.maxAttributeDist2Bwd                     = 1000.;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC3Parameter::notify() {}

//////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& out, const JPCCEncoderTMC3Parameter& obj) {
  auto _params = std::static_pointer_cast<pcc::EncoderParams>(obj.params_);
  obj.coutParameters(out)                                                                                 //
      (SRC_UNIT_LENGTH_OPT, _params->srcUnitLength)                                                       //
      (SRC_UNIT_OPT, _params->sps.seq_geom_scale_unit_flag)                                               //
      (CODING_SCALE_OPT, _params->codedGeomScale)                                                         //
      (SEQUENCE_SCALE_OPT, _params->seqGeomScale)                                                         //
      (POSITION_QUANTIZATION_SCALE_OPT, _params->seqGeomScale)                                            //
      (EXTERNAL_SCALE_OPT, _params->extGeomScale)                                                         //
      (GEOMETRY_AXIS_ORDER_OPT, _params->sps.geometry_axis_order)                                         //
      (AUTO_SEQ_BBOX_OPT, _params->autoSeqBbox)                                                           //
      (SEQ_ORIGIN_OPT, _params->sps.seqBoundingBoxOrigin)                                                 //
      (SEQ_SIZE_WHD_OPT, _params->sps.seqBoundingBoxSize)                                                 //
      (MERGE_DUPLICATED_POINTS_OPT, _params->gps.geom_unique_points_flag)                                 //
      (PARTITION_METHOD_OPT, _params->partition.method)                                                   //
      (PARTITION_OCTREE_DEPTH_OPT, _params->partition.octreeDepth)                                        //
      (SLICE_MAX_POINTS_OPT, _params->partition.sliceMaxPoints)                                           //
      (SLICE_MIN_POINTS_OPT, _params->partition.sliceMinPoints)                                           //
      (TILE_SIZE_OPT, _params->partition.tileSize)                                                        //
      (CABAC_BYPASS_STREAM_ENABLED_FLAG_OPT, _params->sps.cabac_bypass_stream_enabled_flag)               //
      (ENTROPY_CONTINUATION_ENABLED_OPT, _params->sps.entropy_continuation_enabled_flag)                  //
      (ENFORCE_LEVEL_LIMITS_OPT, _params->enforceLevelLimits)                                             //
      (GEOM_TREE_TYPE_OPT, _params->gps.predgeom_enabled_flag)                                            //
      (QTBT_ENABLED_OPT, _params->gps.qtbt_enabled_flag)                                                  //
      (MAX_NUM_QTBT_BEFORE_OT_OPT, _params->geom.qtbt.maxNumQtBtBeforeOt)                                 //
      (MIN_QTBT_SIZE_LOG2_OPT, _params->geom.qtbt.minQtbtSizeLog2)                                        //
      (NUM_OCTREE_ENTROPY_STREAMS_OPT, _params->gbh.geom_stream_cnt_minus1)                               //
      (BITWISE_OCCUPANCY_CODING_OPT, _params->gps.bitwise_occupancy_coding_flag)                          //
      (NEIGHBOUR_AVAIL_BOUNDARY_LOG2_OPT, _params->gps.neighbour_avail_boundary_log2_minus1)              //
      (INFERRED_DIRECT_CODING_MODE_OPT, _params->gps.inferred_direct_coding_mode)                         //
      (JOINT_TWO_POINT_IDCM_OPT, _params->gps.joint_2pt_idcm_enabled_flag)                                //
      (ADJACENT_CHILD_CONTEXTUALIZATION_OPT, _params->gps.adjacent_child_contextualization_enabled_flag)  //
      (INTRA_PRED_MAX_NODE_SIZE_LOG2_OPT, _params->gps.intra_pred_max_node_size_log2)                     //
      (PLANAR_ENABLED_OPT, _params->gps.geom_planar_mode_enabled_flag)                                    //
      (PLANAR_MODE_THRESHOLD0_OPT, _params->gps.geom_planar_threshold0)                                   //
      (PLANAR_MODE_THRESHOLD1_OPT, _params->gps.geom_planar_threshold1)                                   //
      (PLANAR_MODE_THRESHOLD2_OPT, _params->gps.geom_planar_threshold2)                                   //
      (PLANAR_MODE_IDCM_USE_OPT, _params->gps.geom_idcm_rate_minus1)                                      //
      (TRISOUP_NODE_SIZE_LOG2_OPT, _params->trisoupNodeSizesLog2)                                         //
      (TRISOUP_SAMPLING_VALUE_OPT, _params->gps.trisoup_sampling_value)                                   //
      (POSITION_QUANTISATION_ENABLED_OPT, _params->gps.geom_scaling_enabled_flag)                         //
      (POSITION_QUANTISATION_METHOD_OPT, _params->geom.qpMethod)                                          //
      (POSITION_QP_MULTIPLIER_LOG2, _params->gps.geom_qp_multiplier_log2)                                 //
      (POSITION_BASE_QP_OPT, _params->gps.geom_base_qp)                                                   //
      (POSITION_IDCM_QP_OPT, _params->idcmQp)                                                             //
      (POSITION_SLICE_QP_OFFSET_OPT, _params->gbh.geom_slice_qp_offset)                                   //
      (POSITION_QUANTISATION_OCTREE_SIZE_LOG2_OPT, _params->geom.qpOffsetNodeSizeLog2)                    //
      (POSITION_QUANTISATION_OCTREE_DEPTH_OPT, _params->geom.qpOffsetDepth)                               //
      (POSITION_BASE_QP_FREQ_LOG2_OPT, _params->gps.geom_qp_offset_intvl_log2)                            //
      (POSITION_SLICE_QP_FREQ_LOG2_OPT, _params->gbh.geom_qp_offset_intvl_log2_delta)                     //
      (ANGULAR_ENABLED_OPT, _params->gps.geom_angular_mode_enabled_flag)                                  //
      (LIDAR_HEAD_POSITION_OPT, _params->gps.gpsAngularOrigin)                                            //
      (NUM_LASERS_OPT, _params->numLasers)                                                                //
      (LASERS_THETA_OPT, _params->lasersTheta)                                                            //
      (LASERS_Z_OPT, _params->lasersZ)                                                                    //
      (LASERS_NUM_PHI_PER_TURN_OPT, _params->gps.angularNumPhiPerTurn)                                    //
      (PLANAR_BUFFER_DISABLED_OPT, _params->gps.planar_buffer_disabled_flag)                              //
      (PRED_GEOM_AZIMUTH_QUANTIZATION_OPT, _params->gps.azimuth_scaling_enabled_flag)                     //
      (POSITION_AZIMUTH_SCALE_LOG2_OPT, _params->gps.geom_angular_azimuth_scale_log2_minus11)             //
      (POSITION_AZIMUTH_SPEED_OPT, _params->gps.geom_angular_azimuth_speed_minus1)                        //
      (POSITION_RADIUS_INV_SCALE_LOG2_OPT, _params->gps.geom_angular_radius_inv_scale_log2)               //
      (PRED_GEOM_SORT_OPT, _params->predGeom.sortMode)                                                    //
      (PRED_GEOM_AZIMUTH_SORT_PRECISION_OPT, _params->predGeom.azimuthSortRecipBinWidth)                  //
      (PRED_GEOM_TREE_PTS_MAX_OPT, _params->predGeom.maxPtsPerTree)                                       //
      (POINT_COUNT_METADATA_OPT, _params->gps.octree_point_count_list_present_flag)                       //
      (ATTR_SPHERICAL_MAX_LOG2_OPT, _params->attrSphericalMaxLog2)                                        //
      (RECOLOUR_SEARCH_RANGE_OPT, _params->recolour.searchRange)                                          //
      (RECOLOUR_NUM_NEIGHBOURS_FWD_OPT, _params->recolour.numNeighboursFwd)                               //
      (RECOLOUR_NUM_NEIGHBOURS_BWD_OPT, _params->recolour.numNeighboursBwd)                               //
      (RECOLOUR_USE_DIST_WEIGHTED_AVG_FWD_OPT, _params->recolour.useDistWeightedAvgFwd)                   //
      (RECOLOUR_USE_DIST_WEIGHTED_AVG_BWD_OPT, _params->recolour.useDistWeightedAvgBwd)                   //
      (RECOLOUR_SKIP_AVG_IF_IDENTICAL_SOURCE_POINT_PRESENT_FWD_OPT,
       _params->recolour.skipAvgIfIdenticalSourcePointPresentFwd)  //
      (RECOLOUR_SKIP_AVG_IF_IDENTICAL_SOURCE_POINT_PRESENT_BWD_OPT,
       _params->recolour.skipAvgIfIdenticalSourcePointPresentBwd)                     //
      (RECOLOUR_DIST_OFFSET_FWD_OPT, _params->recolour.distOffsetFwd)                 //
      (RECOLOUR_DIST_OFFSET_BWD_OPT, _params->recolour.distOffsetBwd)                 //
      (RECOLOUR_MAX_GEOMETRY_DIST2_FWD_OPT, _params->recolour.maxGeometryDist2Fwd)    //
      (RECOLOUR_MAX_GEOMETRY_DIST2_BWD_OPT, _params->recolour.maxGeometryDist2Bwd)    //
      (RECOLOUR_MAX_ATTRIBUTE_DIST2_FWD_OPT, _params->recolour.maxAttributeDist2Fwd)  //
      (RECOLOUR_MAX_ATTRIBUTE_DIST2_BWD_OPT, _params->recolour.maxAttributeDist2Bwd)  //
      ;
  return out;
}

}  // namespace jpcc::encoder

namespace pcc {
//////////////////////////////////////////////////////////////////////////////////////////////
// :: Command line / config parsing helpers
template <typename T>
static std::istream& readUInt(std::istream& in, T& val) {
  unsigned int tmp;
  in >> tmp;
  val = T(tmp);
  return in;
}
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
static std::istream& operator>>(std::istream& in, AxisOrder& val) { return readUInt(in, val); }
static std::istream& operator>>(std::istream& in, PartitionMethod& val) { return readUInt(in, val); }
static std::istream& operator>>(std::istream& in, PredGeomEncOpts::SortMode& val) { return readUInt(in, val); }
static std::istream& operator>>(std::istream& in, OctreeEncOpts::QpMethod& val) { return readUInt(in, val); }
static std::ostream& operator<<(std::ostream& out, const ScaleUnit& val) {
  switch (val) {
    case ScaleUnit::kDimensionless: out << "0 (Dimensionless)"; break;
    case ScaleUnit::kMetre: out << "1 (Metre)"; break;
  }
  return out;
}
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