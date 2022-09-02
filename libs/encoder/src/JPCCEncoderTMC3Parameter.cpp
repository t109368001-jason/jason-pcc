#include <jpcc/encoder/JPCCEncoderTMC3Parameter.h>
namespace jpcc::encoder {
using namespace std;
using namespace po;
using namespace pcc;
#define BACKEND_TYPE_OPT ".backendType"
JPCCEncoderTMC3Parameter::JPCCEncoderTMC3Parameter() :
    JPCCEncoderTMC3Parameter(JPCC_ENCODER_TMC3_OPT_PREFIX, __FUNCTION__) {}
JPCCEncoderTMC3Parameter::JPCCEncoderTMC3Parameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption), EncoderParams() {
  setDefault();
  opts_.add_options()                                                              //
      ("srcUnitLength", value<double>(&this->srcUnitLength),                       //
       "Length of source point cloud x,y,z unit vectors in srcUnits")              //
      ("srcUnit", value<ScaleUnit>(&this->sps.seq_geom_scale_unit_flag),           //
       " 0: dimensionless\n 1: metres")                                            //
      ("codingScale", value<double>(&this->codedGeomScale),                        //
       "Scale used to represent coded geometry. Relative to inputScale")           //
      ("sequenceScale", value<double>(&this->seqGeomScale),                        //
       "Scale used to obtain sequence coordinate system. Relative to inputScale")  //
      // Alias for compatibility with old name.
      ("positionQuantizationScale", value<double>(&this->seqGeomScale),          //
       "(deprecated)")                                                           //
      ("externalScale", value<double>(&this->extGeomScale),                      //
       "Scale used to define external coordinate system.\n"                      //
       "Meaningless when srcUnit = metres\n"                                     //
       "  0: Use srcUnitLength\n"                                                //
       " >0: Relative to inputScale")                                            //
      ("geometry_axis_order", value<AxisOrder>(&this->sps.geometry_axis_order),  //
       "Sets the geometry axis coding order:\n"                                  //
       "  0: (zyx)\n  1: (xyz)\n  2: (xzy)\n"                                    //
       "  3: (yzx)\n  4: (zyx)\n  5: (zxy)\n"                                    //
       "  6: (yxz)\n  7: (xyz)")                                                 //
      ("autoSeqBbox", value<bool>(&this->autoSeqBbox),                           //
       "Calculate seqOrigin and seqSizeWhd automatically.")                      //
      // NB: the underlying variable is in STV order.
      //     Conversion happens during argument sanitization.
      ("seqOrigin", value<Vec3<int>>(&this->sps.seqBoundingBoxOrigin),                                      //
       "Origin (x,y,z) of the sequence bounding box (in input coordinate system). Requires autoSeqBbox=0")  //
      // NB: the underlying variable is in STV order.
      //     Conversion happens during argument sanitization.
      ("seqSizeWhd", value<Vec3<int>>(&this->sps.seqBoundingBoxSize),                             //
       "Size of the sequence bounding box (in input coordinate system). Requires autoSeqBbox=0")  //
      ("mergeDuplicatedPoints", value<bool>(&this->gps.geom_unique_points_flag),                  //
       "Enables removal of duplicated points")                                                    //
      ("partitionMethod", value<PartitionMethod>(&this->partition.method),                        //
       "Method used to partition input point cloud into slices/tiles:\n"
       "  0: none\n"
       "  2: n Uniform-geometry partition bins along the longest edge\n"
       "  3: Uniform geometry partition at n octree depth\n"
       "  4: Uniform square partition\n"
       "  5: n-point spans of input")                                     //
      ("partitionOctreeDepth", value<int>(&this->partition.octreeDepth),  //
       "Depth of octree partition for partitionMethod=4")                 //
      ("sliceMaxPoints", value<int>(&this->partition.sliceMaxPoints),     //
       "Maximum number of points per slice")                              //
      ("sliceMinPoints", value<int>(&this->partition.sliceMinPoints),     //
       "Minimum number of points per slice (soft limit)")                 //
      ("tileSize", value<int>(&this->partition.tileSize),
       "Partition input into cubic tiles of given size")                                              //
      ("cabac_bypass_stream_enabled_flag", value<bool>(&this->sps.cabac_bypass_stream_enabled_flag),  //
       "Controls coding method for ep(bypass) bins")                                                  //
      ("entropyContinuationEnabled", value<bool>(&this->sps.entropy_continuation_enabled_flag),       //
       "Propagate context state between slices")                                                      //
      ("enforceLevelLimits", value<bool>(&this->enforceLevelLimits),                                  //
       "Abort if level limits exceeded")                                                              //
      ("geomTreeType", value<bool>(&this->gps.predgeom_enabled_flag),                                 //
       "Selects the tree coding method:\n"
       "  0: octree\n"
       "  1: predictive")                                                      //
      ("qtbtEnabled", value<bool>(&this->gps.qtbt_enabled_flag),               //
       "Enables non-cubic geometry bounding box")                              //
      ("maxNumQtBtBeforeOt", value<int>(&this->geom.qtbt.maxNumQtBtBeforeOt),  //
       "Max number of qtbt partitions before ot")                              //
      ("minQtbtSizeLog2", value<int>(&this->geom.qtbt.minQtbtSizeLog2),        //
       "Minimum size of qtbt partitions")                                      //
      ("numOctreeEntropyStreams",
       // NB: this is adjusted by minus 1 after the arguments
       // are parsed
       value<int>(&this->gbh.geom_stream_cnt_minus1),                                   //
       "Number of entropy streams for octree coding")                                   //
      ("bitwiseOccupancyCoding", value<int>(&this->gps.bitwise_occupancy_coding_flag),  //
       "Selects between bitwise and bytewise occupancy coding:\n"
       "  0: bytewise\n"
       "  1: bitwise")  //
      ("neighbourAvailBoundaryLog2",
       // NB: this is adjusted by minus 1 after the
       // arguments are parsed
       value<int>(&this->gps.neighbour_avail_boundary_log2_minus1),  //
       "Defines the avaliability volume for neighbour occupancy lookups:\n"
       "<2: Limited to sibling nodes only")                                             //
      ("inferredDirectCodingMode", value<int>(&this->gps.inferred_direct_coding_mode),  //
       "Early termination of the geometry octree for isolated points:"
       " 0: disabled\n"
       " 1: fully constrained\n"
       " 2: partially constrained\n"
       " 3: unconstrained (fastest)")                                                                              //
      ("jointTwoPointIdcm", value<int>(&this->gps.joint_2pt_idcm_enabled_flag),                                    //
       "Jointly code common prefix of two IDCM points")                                                            //
      ("adjacentChildContextualization", value<int>(&this->gps.adjacent_child_contextualization_enabled_flag),     //
       "Occupancy contextualization using neighbouring adjacent children")                                         //
      ("intra_pred_max_node_size_log2", value<int>(&this->gps.intra_pred_max_node_size_log2),                      //
       "octree nodesizes eligible for occupancy intra prediction")                                                 //
      ("planarEnabled", value<int>(&this->gps.geom_planar_mode_enabled_flag),                                      //
       "Use planar mode for geometry coding")                                                                      //
      ("planarModeThreshold0", value<int>(&this->gps.geom_planar_threshold0),                                      //
       "Activation threshold (0-127) of first planar mode. Lower values imply more use of the first planar mode")  //
      ("planarModeThreshold"
       "1",
       value<int>(&this->gps.geom_planar_threshold1),                                                               //
       "Activation threshold (0-127) of second planar mode. Lower values imply more use of the first planar mode")  //
      ("planarModeThres"
       "hold2",
       value<int>(&this->gps.geom_planar_threshold2),                                                              //
       "Activation threshold (0-127) of third planar mode. Lower values imply more use of the third planar mode")  //
      ("planarModeIdcmUse",
       // NB: this is adjusted by minus1 after the arguments are parsed
       value<int>(&this->gps.geom_idcm_rate_minus1),  //
       "Degree (1/32%) of IDCM activation when planar mode is enabled\n"
       "  0 => never, 32 => always")                                                                   //
      ("trisoupNodeSizeLog2", value<int>(&this->trisoupNodeSizesLog2),                                 //
       "Node size for surface triangulation\n"                                                         //
       " <2: disabled")                                                                                //
      ("trisoup_sampling_value", value<int>(&this->gps.trisoup_sampling_value),                        //
       "Trisoup voxelisation sampling rate\n"                                                          //
       "  0: automatic")                                                                               //
      ("positionQuantisationEnabled", value<int>(&this->gps.geom_scaling_enabled_flag),                //
       "Enable in-loop quantisation of positions")                                                     //
      ("positionQuantisationMethod", value<int>(&this->geom.qpMethod),                                 //
       "Method used to determine per-node QP:\n"                                                       //
       "  0: uniform\n"                                                                                //
       "  1: random\n"                                                                                 //
       "  2: by node point density")                                                                   //
      ("positionQpMultiplierLog2", value<int>(&this->gps.geom_qp_multiplier_log2),                     //
       "Granularity of QP to step size mapping:\n"                                                     //
       "  n: 2^n QPs per doubling interval, n in 0..3")                                                //
      ("positionBaseQp", value<int>(&this->gps.geom_base_qp),                                          //
       "Base QP used in position quantisation (0 = lossless)")                                         //
      ("positionIdcmQp", value<int>(&this->idcmQp), "QP used in position quantisation of IDCM nodes")  //
      ("positionSliceQpOffset", value<int>(&this->gbh.geom_slice_qp_offset),                           //
       "Per-slice QP offset used in position quantisation")                                            //
      ("positionQuantisationOctreeSizeLog2", value<int>(&this->geom.qpOffsetNodeSizeLog2),             //
       "Octree node size used for signalling position QP offsets (-1 => disabled)")                    //
      ("positionQuantisationOctreeDepth", value<int>(&this->geom.qpOffsetDepth),                       //
       "Octree depth used for signalling position QP offsets (-1 => disabled)")                        //
      ("positionBaseQpFreqLog2", value<int>(&this->gps.geom_qp_offset_intvl_log2),                     //
       "Frequency of sending QP offsets in predictive geometry coding")                                //
      // NB: this will be corrected to be relative to base value later
      ("positionSliceQpFreqLog2", value<double>(&this->gbh.geom_qp_offset_intvl_log2_delta),  //
       "Frequency of sending QP offsets in predictive geometry coding")                       //
      ("angularEnabled", value<double>(&this->gps.geom_angular_mode_enabled_flag),            //
       "Controls angular contextualisation of occupancy")                                     //
      // NB: the underlying variable is in STV order.
      //     Conversion happens during argument sanitization.
      ("lidarHeadPosition", value<double>(&this->gps.gpsAngularOrigin),                                //
       "laser head position (x,y,z) in angular mode")                                                  //
      ("numLasers", value<double>(&this->numLasers),                                                   //
       "Number of lasers in angular mode")                                                             //
      ("lasersTheta", value<double>(&this->lasersTheta),                                               //
       "Vertical laser angle in angular mode")                                                         //
      ("lasersZ", value<double>(&this->lasersZ), "Vertical laser offset in angular mode")              //
      ("lasersNumPhiPerTurn", value<double>(&this->gps.angularNumPhiPerTurn),                          //
       "Number of sampling poisitions in a complete laser turn in angular mode")                       //
      ("planarBufferDisabled", value<double>(&this->gps.planar_buffer_disabled_flag),                  //
       "Disable planar buffer (when angular mode is enabled)")                                         //
      ("predGeomAzimuthQuantization", value<double>(&this->gps.azimuth_scaling_enabled_flag),          //
       "Quantize azimuth according to radius in predictive geometry coding")                           //
      ("positionAzimuthScaleLog2", value<double>(&this->gps.geom_angular_azimuth_scale_log2_minus11),  //
       "Additional bits to represent azimuth angle in predictive geometry coding")                     //
      // NB: this will be corrected to be minus 1 later
      ("positionAzimuthSpeed", value<double>(&this->gps.geom_angular_azimuth_speed_minus1),         //
       "Scale factor applied to azimuth angle in predictive geometry coding")                       //
      ("positionRadiusInvScaleLog2", value<double>(&this->gps.geom_angular_radius_inv_scale_log2),  //
       "Inverse scale factor applied to radius in predictive geometry coding")                      //
      ("predGeomSort", value<double>(&this->predGeom.sortMode),                                     //
       "Predictive geometry tree construction order")                                               //
      ("predGeomAzimuthSortPrecision", value<double>(&this->predGeom.azimuthSortRecipBinWidth),     //
       "Reciprocal precision used in azimuthal sorting for tree construction")                      //
      ("predGeomTreePtsMax", value<double>(&this->predGeom.maxPtsPerTree),                          //
       "Maximum number of points per predictive geometry tree")                                     //
      ("pointCountMetadata", value<double>(&this->gps.octree_point_count_list_present_flag),        //
       "Add octree layer point count metadata")                                                     //
      ("attrSphericalMaxLog2", value<double>(&this->attrSphericalMaxLog2),                          //
       "Override spherical coordinate normalisation factor")                                        //
      ("recolourSearchRange", value<double>(&this->recolour.searchRange),                           //
       "")                                                                                          //
      ("recolourNumNeighboursFwd", value<double>(&this->recolour.numNeighboursFwd),                 //
       "")                                                                                          //
      ("recolourNumNeighboursBwd", value<double>(&this->recolour.numNeighboursBwd),                 //
       "")                                                                                          //
      ("recolourUseDistWeightedAvgFwd", value<double>(&this->recolour.useDistWeightedAvgFwd),       //
       "")                                                                                          //
      ("recolourUseDistWeightedAvgBwd", value<double>(&this->recolour.useDistWeightedAvgBwd),       //
       "")                                                                                          //
      ("recolourSkipAvgIfIdenticalSourcePointPresentFwd",                                           //
       value<double>(&this->recolour.skipAvgIfIdenticalSourcePointPresentFwd),                      //
       "")                                                                                          //
      ("recolourSkipAvgIfIdenticalSourcePointPresentBwd",                                           //
       value<double>(&this->recolour.skipAvgIfIdenticalSourcePointPresentBwd),                      //
       "")                                                                                          //
      ("recolourDistOffsetFwd", value<double>(&this->recolour.distOffsetFwd), "")                   //
      ("recolourDistOffsetBwd", value<double>(&this->recolour.distOffsetBwd), "")                   //
      ("recolourMaxGeometryDist2Fwd", value<double>(&this->recolour.maxGeometryDist2Fwd),           //
       "")                                                                                          //
      ("recolourMaxGeometryDist2Bwd", value<double>(&this->recolour.maxGeometryDist2Bwd),           //
       "")                                                                                          //
      ("recolourMaxAttributeDist2Fwd", value<double>(&this->recolour.maxAttributeDist2Fwd),         //
       "")                                                                                          //
      ("recolourMaxAttributeDist2Bwd", value<double>(&this->recolour.maxAttributeDist2Bwd),         //
       "")                                                                                          //
      ;
}
void JPCCEncoderTMC3Parameter::setDefault() {
  srcUnitLength                                     = 1.;
  sps.seq_geom_scale_unit_flag                      = ScaleUnit::kDimensionless;
  codedGeomScale                                    = 1.;
  seqGeomScale                                      = 1.;
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
void     JPCCEncoderTMC3Parameter::notify() {}
ostream& operator<<(ostream& out, const JPCCEncoderTMC3Parameter& obj) {
  obj.coutParameters(out)  //
      ;
  return out;
}
}  // namespace jpcc::encoder
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
      throw std::runtime_error("Cannot parse unit");
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
