#pragma once

#include <iostream>
#include <string>

#include <PCCTMC3Encoder.h>

#include <jpcc/common/Parameter.h>

namespace jpcc::encoder {

#define JPCC_ENCODER_TMC3_OPT_PREFIX "jpccEncoderTMC3Parameter"

class JPCCEncoderTMC3Parameter : public virtual Parameter, public pcc::EncoderParams {
 public:
  JPCCEncoderTMC3Parameter();

  JPCCEncoderTMC3Parameter(const std::string& prefix, const std::string& caption);

  void setDefault();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCEncoderTMC3Parameter& obj);
};

}  // namespace jpcc::encoder

//---------------------------------------------------------------------------
// :: Command line / config parsing helpers

template <typename T>
static std::istream& readUInt(std::istream& in, T& val);

namespace pcc {
static std::istream& operator>>(std::istream& in, pcc::ScaleUnit& val);
}  // namespace pcc

// static std::istream& operator>>(std::istream& in, pcc::OutputSystem& val);

namespace pcc {
static std::istream& operator>>(std::istream& in, pcc::ColourMatrix& val);
}  // namespace pcc

namespace pcc {
static std::istream& operator>>(std::istream& in, pcc::AxisOrder& val);
}  // namespace pcc

namespace pcc {
static std::istream& operator>>(std::istream& in, pcc::AttributeEncoding& val);
}  // namespace pcc

namespace pcc {
static std::istream& operator>>(std::istream& in, pcc::LodDecimationMethod& val);
}  // namespace pcc

namespace pcc {
static std::istream& operator>>(std::istream& in, pcc::PartitionMethod& val);
}  // namespace pcc

namespace pcc {
static std::istream& operator>>(std::istream& in, pcc::PredGeomEncOpts::SortMode& val);
}  // namespace pcc

namespace pcc {
static std::istream& operator>>(std::istream& in, pcc::OctreeEncOpts::QpMethod& val);
}  // namespace pcc

// static std::ostream& operator<<(std::ostream& out, const pcc::OutputSystem& val);

namespace pcc {
static std::ostream& operator<<(std::ostream& out, const pcc::ScaleUnit& val);
}  // namespace pcc

namespace pcc {
static std::ostream& operator<<(std::ostream& out, const pcc::ColourMatrix& val);
}  // namespace pcc

namespace pcc {
static std::ostream& operator<<(std::ostream& out, const pcc::AxisOrder& val);
}  // namespace pcc

namespace pcc {
static std::ostream& operator<<(std::ostream& out, const pcc::AttributeEncoding& val);
}  // namespace pcc

namespace pcc {
static std::ostream& operator<<(std::ostream& out, const pcc::LodDecimationMethod& val);
}  // namespace pcc

namespace pcc {
static std::ostream& operator<<(std::ostream& out, const pcc::PartitionMethod& val);
}  // namespace pcc

namespace pcc {
static std::ostream& operator<<(std::ostream& out, const pcc::PredGeomEncOpts::SortMode& val);
}  // namespace pcc

namespace pcc {
static std::ostream& operator<<(std::ostream& out, const pcc::OctreeEncOpts::QpMethod& val);
}  // namespace pcc

//---------------------------------------------------------------------------
