#include <jpcc/encoder/JPCCEncoderTMC3.h>

#include <sstream>

#include <io_tlv.h>
#include <PCCTMC3Encoder.h>

using namespace pcc;

namespace jpcc::encoder {

class PCCTMC3Encoder3LambdaCallbacks : public PCCTMC3Encoder3::Callbacks {
 protected:
  const std::function<void(const PayloadBuffer& buffer)>& onOutputBuffer_;
  const std::function<void(const PCCPointSet3& set3)>&    onPostRecolour_;

 public:
  PCCTMC3Encoder3LambdaCallbacks(const std::function<void(const PayloadBuffer& buffer)>& onOutputBuffer,
                                 const std::function<void(const PCCPointSet3& set3)>&    onPostRecolour);

 protected:
  void onOutputBuffer(const PayloadBuffer& buffer) override;
  void onPostRecolour(const PCCPointSet3& set3) override;
};

pcc::EncoderParams toPCC(const JPCCEncoderTMC3Parameter& parameter);

//////////////////////////////////////////////////////////////////////////////////////////////
PCCTMC3Encoder3LambdaCallbacks::PCCTMC3Encoder3LambdaCallbacks(
    const std::function<void(const PayloadBuffer& buffer)>& onOutputBuffer,
    const std::function<void(const PCCPointSet3& set3)>&    onPostRecolour) :
    onOutputBuffer_(onOutputBuffer), onPostRecolour_(onPostRecolour) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void PCCTMC3Encoder3LambdaCallbacks::onOutputBuffer(const PayloadBuffer& buffer) { onOutputBuffer_(buffer); }

//////////////////////////////////////////////////////////////////////////////////////////////
void PCCTMC3Encoder3LambdaCallbacks::onPostRecolour(const PCCPointSet3& set3) { onPostRecolour_(set3); }

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC3::JPCCEncoderTMC3(const JPCCEncoderParameter& parameter) : JPCCEncoder(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderTMC3::isConvertToCoderTypeThreadSafe() { return true; }

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderTMC3::isEncodeThreadSafe() { return true; }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC3::convertToCoderType(const FramePtr& frame, CoderFramePtr& coderFrame) {
  coderFrame       = std::make_shared<pcc::PCCPointSet3>();
  auto _coderFrame = std::static_pointer_cast<pcc::PCCPointSet3>(coderFrame);
  _coderFrame->resize(frame->getPointCount());
  for (size_t i = 0; i < _coderFrame->getPointCount(); i++) {
    (*_coderFrame)[i].x() = (*frame)[i][0];
    (*_coderFrame)[i].y() = (*frame)[i][1];
    (*_coderFrame)[i].z() = (*frame)[i][2];
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC3::encode(const CoderFramePtr& coderFrame, std::vector<char>& encodedBytes) {
  std::function<void(const PayloadBuffer& buffer)> onOutputBuffer = [&encodedBytes](const PayloadBuffer& buffer) {
    std::stringstream os;
    writeTlv(buffer, os);
#if !defined(NDEBUG)
    size_t oldSize = encodedBytes.size();
#endif
    std::string tmpString = os.str();
    for (char& i : tmpString) { encodedBytes.push_back(i); }
    assert((buffer.size() + 5) == (encodedBytes.size() - oldSize));
  };
  std::function<void(const PCCPointSet3& set3)> onPostRecolour = [](const PCCPointSet3& set3) {};

  auto _coderFrame = std::static_pointer_cast<pcc::PCCPointSet3>(coderFrame);
  if (_coderFrame->getPointCount() == 0) {
    PayloadBuffer buffer;
    buffer.type = PayloadType::kSequenceParameterSet;

    onOutputBuffer(buffer);
    return;
  }

  pcc::EncoderParams             param = toPCC(this->parameter_.tmc3);
  PCCTMC3Encoder3LambdaCallbacks callback(onOutputBuffer, onPostRecolour);
  PCCTMC3Encoder3                encoder;

  encoder.compress(*_coderFrame, &param, &callback, nullptr);
  std::cout << __FUNCTION__ << "() "
            << "bytes=" << encodedBytes.size() << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcc::EncoderParams toPCC(const JPCCEncoderTMC3Parameter& parameter) {
  pcc::EncoderParams result           = {0};
  result.sps.seqBoundingBoxOrigin     = {parameter.sps.seqBoundingBoxOrigin[0], parameter.sps.seqBoundingBoxOrigin[1],
                                         parameter.sps.seqBoundingBoxOrigin[2]};
  result.sps.seqBoundingBoxSize       = {parameter.sps.seqBoundingBoxSize[0], parameter.sps.seqBoundingBoxSize[1],
                                         parameter.sps.seqBoundingBoxSize[2]};
  result.sps.seq_geom_scale_unit_flag = static_cast<pcc::ScaleUnit>(parameter.sps.seq_geom_scale_unit_flag);
  result.sps.geometry_axis_order      = static_cast<pcc::AxisOrder>(parameter.sps.geometry_axis_order);
  result.sps.cabac_bypass_stream_enabled_flag     = parameter.sps.cabac_bypass_stream_enabled_flag;
  result.sps.entropy_continuation_enabled_flag    = parameter.sps.entropy_continuation_enabled_flag;
  result.gps.predgeom_enabled_flag                = parameter.gps.predgeom_enabled_flag;
  result.gps.geom_unique_points_flag              = parameter.gps.geom_unique_points_flag;
  result.gps.neighbour_avail_boundary_log2_minus1 = parameter.gps.neighbour_avail_boundary_log2_minus1;
  result.gps.inferred_direct_coding_mode          = parameter.gps.inferred_direct_coding_mode;
  result.gps.joint_2pt_idcm_enabled_flag          = parameter.gps.joint_2pt_idcm_enabled_flag;
  result.gps.bitwise_occupancy_coding_flag        = parameter.gps.bitwise_occupancy_coding_flag;
  result.gps.adjacent_child_contextualization_enabled_flag =
      parameter.gps.adjacent_child_contextualization_enabled_flag;
  result.gps.intra_pred_max_node_size_log2  = parameter.gps.intra_pred_max_node_size_log2;
  result.gps.trisoup_sampling_value         = parameter.gps.trisoup_sampling_value;
  result.gps.geom_scaling_enabled_flag      = parameter.gps.geom_scaling_enabled_flag;
  result.gps.geom_qp_multiplier_log2        = parameter.gps.geom_qp_multiplier_log2;
  result.gps.geom_base_qp                   = parameter.gps.geom_base_qp;
  result.gps.qtbt_enabled_flag              = parameter.gps.qtbt_enabled_flag;
  result.gps.geom_planar_mode_enabled_flag  = parameter.gps.geom_planar_mode_enabled_flag;
  result.gps.geom_planar_threshold0         = parameter.gps.geom_planar_threshold0;
  result.gps.geom_planar_threshold1         = parameter.gps.geom_planar_threshold1;
  result.gps.geom_planar_threshold2         = parameter.gps.geom_planar_threshold2;
  result.gps.geom_idcm_rate_minus1          = parameter.gps.geom_idcm_rate_minus1;
  result.gps.geom_angular_mode_enabled_flag = parameter.gps.geom_angular_mode_enabled_flag;
  result.gps.gpsAngularOrigin               = {parameter.gps.gpsAngularOrigin[0], parameter.gps.gpsAngularOrigin[1],
                                               parameter.gps.gpsAngularOrigin[2]};
  result.gps.angularNumPhiPerTurn           = parameter.gps.angularNumPhiPerTurn;
  result.gps.planar_buffer_disabled_flag    = parameter.gps.planar_buffer_disabled_flag;
  result.gps.geom_qp_offset_intvl_log2      = parameter.gps.geom_qp_offset_intvl_log2;
  result.gps.geom_angular_azimuth_scale_log2_minus11 = parameter.gps.geom_angular_azimuth_scale_log2_minus11;
  result.gps.geom_angular_azimuth_speed_minus1       = parameter.gps.geom_angular_azimuth_speed_minus1;
  result.gps.geom_angular_radius_inv_scale_log2      = parameter.gps.geom_angular_radius_inv_scale_log2;
  result.gps.octree_point_count_list_present_flag    = parameter.gps.octree_point_count_list_present_flag;
  result.gps.azimuth_scaling_enabled_flag            = parameter.gps.azimuth_scaling_enabled_flag;
  result.gbh.geom_slice_qp_offset                    = parameter.gbh.geom_slice_qp_offset;
  result.gbh.geom_qp_offset_intvl_log2_delta         = parameter.gbh.geom_qp_offset_intvl_log2_delta;
  result.gbh.geom_stream_cnt_minus1                  = parameter.gbh.geom_stream_cnt_minus1;
  result.autoSeqBbox                                 = parameter.autoSeqBbox;
  result.srcUnitLength                               = parameter.srcUnitLength;
  result.codedGeomScale                              = parameter.codedGeomScale;
  result.seqGeomScale                                = parameter.seqGeomScale;
  result.extGeomScale                                = parameter.extGeomScale;
  result.geom.qtbt.maxNumQtBtBeforeOt                = parameter.geom.qtbt.maxNumQtBtBeforeOt;
  result.geom.qtbt.minQtbtSizeLog2                   = parameter.geom.qtbt.minQtbtSizeLog2;
  switch (parameter.geom.qpMethod) {
    case OctreeEncOpts::QpMethod::kUniform: result.geom.qpMethod = pcc::OctreeEncOpts::QpMethod::kUniform; break;
    case OctreeEncOpts::QpMethod::kRandom: result.geom.qpMethod = pcc::OctreeEncOpts::QpMethod::kRandom; break;
    case OctreeEncOpts::QpMethod::kByDensity: result.geom.qpMethod = pcc::OctreeEncOpts::QpMethod::kByDensity; break;
  }
  result.geom.qpOffsetDepth        = parameter.geom.qpOffsetDepth;
  result.geom.qpOffsetNodeSizeLog2 = parameter.geom.qpOffsetNodeSizeLog2;
  switch (parameter.predGeom.sortMode) {
    case PredGeomEncOpts::SortMode::kNoSort: result.predGeom.sortMode = pcc::PredGeomEncOpts::SortMode::kNoSort; break;
    case PredGeomEncOpts::SortMode::kSortMorton:
      result.predGeom.sortMode = pcc::PredGeomEncOpts::SortMode::kSortMorton;
      break;
    case PredGeomEncOpts::SortMode::kSortAzimuth:
      result.predGeom.sortMode = pcc::PredGeomEncOpts::SortMode::kSortAzimuth;
      break;
    case PredGeomEncOpts::SortMode::kSortRadius:
      result.predGeom.sortMode = pcc::PredGeomEncOpts::SortMode::kSortRadius;
      break;
    case PredGeomEncOpts::SortMode::kSortLaserAngle:
      result.predGeom.sortMode = pcc::PredGeomEncOpts::SortMode::kSortLaserAngle;
      break;
  }
  result.predGeom.maxPtsPerTree            = parameter.predGeom.maxPtsPerTree;
  result.predGeom.azimuthSortRecipBinWidth = parameter.predGeom.azimuthSortRecipBinWidth;
  switch (parameter.partition.method) {
    case PartitionMethod::kNone: result.partition.method = pcc::PartitionMethod::kNone; break;
    case PartitionMethod::kUniformGeom: result.partition.method = pcc::PartitionMethod::kUniformGeom; break;
    case PartitionMethod::kOctreeUniform: result.partition.method = pcc::PartitionMethod::kOctreeUniform; break;
    case PartitionMethod::kUniformSquare: result.partition.method = pcc::PartitionMethod::kUniformSquare; break;
    case PartitionMethod::kNpoints: result.partition.method = pcc::PartitionMethod::kNpoints; break;
  }
  result.partition.octreeDepth                            = parameter.partition.octreeDepth;
  result.partition.sliceMaxPoints                         = parameter.partition.sliceMaxPoints;
  result.partition.sliceMinPoints                         = parameter.partition.sliceMinPoints;
  result.partition.tileSize                               = parameter.partition.tileSize;
  result.recolour.distOffsetFwd                           = parameter.recolour.distOffsetFwd;
  result.recolour.distOffsetBwd                           = parameter.recolour.distOffsetBwd;
  result.recolour.maxGeometryDist2Fwd                     = parameter.recolour.maxGeometryDist2Fwd;
  result.recolour.maxGeometryDist2Bwd                     = parameter.recolour.maxGeometryDist2Bwd;
  result.recolour.maxAttributeDist2Fwd                    = parameter.recolour.maxAttributeDist2Fwd;
  result.recolour.maxAttributeDist2Bwd                    = parameter.recolour.maxAttributeDist2Bwd;
  result.recolour.searchRange                             = parameter.recolour.searchRange;
  result.recolour.numNeighboursFwd                        = parameter.recolour.numNeighboursFwd;
  result.recolour.numNeighboursBwd                        = parameter.recolour.numNeighboursBwd;
  result.recolour.useDistWeightedAvgFwd                   = parameter.recolour.useDistWeightedAvgFwd;
  result.recolour.useDistWeightedAvgBwd                   = parameter.recolour.useDistWeightedAvgBwd;
  result.recolour.skipAvgIfIdenticalSourcePointPresentFwd = parameter.recolour.skipAvgIfIdenticalSourcePointPresentFwd;
  result.recolour.skipAvgIfIdenticalSourcePointPresentBwd = parameter.recolour.skipAvgIfIdenticalSourcePointPresentBwd;
  result.numLasers                                        = parameter.numLasers;
  result.lasersTheta                                      = parameter.lasersTheta;
  result.lasersZ                                          = parameter.lasersZ;
  result.trisoupNodeSizesLog2                             = parameter.trisoupNodeSizesLog2;
  result.enforceLevelLimits                               = parameter.enforceLevelLimits;
  result.idcmQp                                           = parameter.idcmQp;
  result.attrSphericalMaxLog2                             = parameter.attrSphericalMaxLog2;
  return result;
}

}  // namespace jpcc::encoder