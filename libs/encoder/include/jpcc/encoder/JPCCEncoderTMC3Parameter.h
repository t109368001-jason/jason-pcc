#pragma once

#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>

namespace jpcc::encoder {

#define JPCC_ENCODER_TMC3_OPT_PREFIX "jpccEncoderTMC3Parameter"

enum class ScaleUnit : bool {
  kDimensionless = 0,
  kMetre         = 1,
};
enum class AxisOrder {
  kZYX   = 0,
  kXYZ   = 1,
  kXZY   = 2,
  kYZX   = 3,
  kZYX_4 = 4,
  kZXY   = 5,
  kYXZ   = 6,
  kXYZ_7 = 7,
};
struct SequenceParameterSet {
  std::vector<int> seqBoundingBoxOrigin;
  std::vector<int> seqBoundingBoxSize;
  ScaleUnit        seq_geom_scale_unit_flag;
  AxisOrder        geometry_axis_order;
  bool             cabac_bypass_stream_enabled_flag;
  bool             entropy_continuation_enabled_flag;
};
struct GeometryParameterSet {
  bool             predgeom_enabled_flag;
  bool             geom_unique_points_flag;
  int              neighbour_avail_boundary_log2_minus1;
  int              inferred_direct_coding_mode;
  bool             joint_2pt_idcm_enabled_flag;
  bool             bitwise_occupancy_coding_flag;
  bool             adjacent_child_contextualization_enabled_flag;
  int              intra_pred_max_node_size_log2;
  int              trisoup_sampling_value;
  bool             geom_scaling_enabled_flag;
  int              geom_qp_multiplier_log2;
  int              geom_base_qp;
  bool             qtbt_enabled_flag;
  bool             geom_planar_mode_enabled_flag;
  int              geom_planar_threshold0;
  int              geom_planar_threshold1;
  int              geom_planar_threshold2;
  int              geom_idcm_rate_minus1;
  bool             geom_angular_mode_enabled_flag;
  std::vector<int> gpsAngularOrigin;
  std::vector<int> angularNumPhiPerTurn;
  bool             planar_buffer_disabled_flag;
  int              geom_qp_offset_intvl_log2;
  int              geom_angular_azimuth_scale_log2_minus11;
  int              geom_angular_azimuth_speed_minus1;
  int              geom_angular_radius_inv_scale_log2;
  bool             octree_point_count_list_present_flag;
  bool             azimuth_scaling_enabled_flag;
};
struct GeometryBrickHeader {
  int geom_slice_qp_offset;
  int geom_qp_offset_intvl_log2_delta;
  int geom_stream_cnt_minus1;
};
struct QtBtParameters {
  int maxNumQtBtBeforeOt;
  int minQtbtSizeLog2;
};
struct OctreeEncOpts {
  QtBtParameters qtbt;
  enum class QpMethod {
    kUniform   = 0,
    kRandom    = 1,
    kByDensity = 2,
  } qpMethod;
  int qpOffsetDepth;
  int qpOffsetNodeSizeLog2;
};
struct PredGeomEncOpts {
  enum SortMode { kNoSort, kSortMorton, kSortAzimuth, kSortRadius, kSortLaserAngle } sortMode;
  int   maxPtsPerTree;
  float azimuthSortRecipBinWidth;
};
enum class PartitionMethod {
  kNone          = 0,
  kUniformGeom   = 2,
  kOctreeUniform = 3,
  kUniformSquare = 4,
  kNpoints       = 5,
};
struct PartitionParams {
  PartitionMethod method;
  int             octreeDepth;
  int             sliceMaxPoints;
  int             sliceMinPoints;
  int             tileSize;
};
struct RecolourParams {
  double distOffsetFwd;
  double distOffsetBwd;
  double maxGeometryDist2Fwd;
  double maxGeometryDist2Bwd;
  double maxAttributeDist2Fwd;
  double maxAttributeDist2Bwd;
  int    searchRange;
  int    numNeighboursFwd;
  int    numNeighboursBwd;
  bool   useDistWeightedAvgFwd;
  bool   useDistWeightedAvgBwd;
  bool   skipAvgIfIdenticalSourcePointPresentFwd;
  bool   skipAvgIfIdenticalSourcePointPresentBwd;
};

class JPCCEncoderTMC3Parameter : public virtual Parameter {
 public:
  SequenceParameterSet sps;
  GeometryParameterSet gps;
  GeometryBrickHeader  gbh;
  bool                 autoSeqBbox;
  double               srcUnitLength;
  double               codedGeomScale;
  double               seqGeomScale;
  double               extGeomScale;
  OctreeEncOpts        geom;
  PredGeomEncOpts      predGeom;
  PartitionParams      partition;
  RecolourParams       recolour;
  int                  numLasers;
  std::vector<double>  lasersTheta;
  std::vector<double>  lasersZ;
  std::vector<int>     trisoupNodeSizesLog2;
  bool                 enforceLevelLimits;
  int                  idcmQp;
  int                  attrSphericalMaxLog2;

 public:
  JPCCEncoderTMC3Parameter();

  JPCCEncoderTMC3Parameter(const std::string& prefix, const std::string& caption);

  void setDefault();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCEncoderTMC3Parameter& obj);
};

//---------------------------------------------------------------------------
// :: Command line / config parsing helpers

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

//---------------------------------------------------------------------------

}  // namespace jpcc::encoder
