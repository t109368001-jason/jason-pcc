#include <jpcc/encoder/JPCCEncoderTMC2.h>

#include <sstream>

#include <PCCEncoder.h>
#include <PCCContext.h>
#include <PCCFrameContext.h>
#include <PCCBitstream.h>
#include <PCCGroupOfFrames.h>
#include <PCCEncoderParameters.h>
#include <PCCBitstreamWriter.h>

using namespace pcc;

namespace jpcc::encoder {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC2::JPCCEncoderTMC2(const JPCCEncoderTMC2Parameter parameter) :
    JPCCEncoder(), parameter_(parameter), encoder_(std::make_shared<PCCEncoder>()), contextIndex(0) {
  auto _encoder = std::static_pointer_cast<PCCEncoder>(encoder_);
  _encoder->setParameters(*std::static_pointer_cast<PCCEncoderParameters>(parameter_.params_));
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderTMC2::isConvertToCoderTypeThreadSafe() {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderTMC2::isEncodeThreadSafe() {
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC2::convertToCoderType(const FramePtr& frame, CoderFramePtr& coderFrame) {
  auto _coderFrame = std::make_shared<PCCPointSet3>();
  coderFrame       = _coderFrame;
  _coderFrame->resize(frame->getPointCount());
  for (size_t i = 0; i < _coderFrame->getPointCount(); i++) {
    auto point          = (*frame)[i];
    using TMC2ValueType = std::remove_reference_t<std::remove_const_t<decltype(((PCCPoint3D*)nullptr)->x())>>;
    _coderFrame->setPosition(i, PCCPoint3D((TMC2ValueType)point[0], (TMC2ValueType)point[1], (TMC2ValueType)point[2]));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC2::encode(const JPCCEncoder::CoderFramePtr& coderFrame, std::ostream& os) {
  BOOST_THROW_EXCEPTION(std::logic_error("no implementation"));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC2::encode(const CoderGroupOfFrame& coderFrames, std::ostream& os, const bool parallel) {
  auto _encoder = std::static_pointer_cast<PCCEncoder>(encoder_);
  auto _params  = std::static_pointer_cast<PCCEncoderParameters>(parameter_.params_);

  PCCBitstreamStat    bitstreamStat;
  SampleStreamV3CUnit ssvu;

  PCCGroupOfFrames sources(coderFrames.size());
  for (size_t i = 0; i < coderFrames.size(); i++) {
    auto _coderFrame = std::static_pointer_cast<PCCPointSet3>(coderFrames[i]);
    sources[i]       = *_coderFrame;
  }

  PCCGroupOfFrames reconstructs;
  PCCContext       context;
  context.setBitstreamStat(bitstreamStat);
  context.addV3CParameterSet(contextIndex);
  context.setActiveVpsId(contextIndex);
  int ret = _encoder->encode(sources, context, reconstructs);
  THROW_IF_NOT(ret == 0);

  PCCBitstreamWriter bitstreamWriter;
  ret = bitstreamWriter.encode(context, ssvu);
  THROW_IF_NOT(ret == 0);

  PCCBitstream bitstream;
  bitstreamStat.setHeader(bitstream.size());
  size_t headerSize = bitstreamWriter.write(ssvu, bitstream, _params->forcedSsvhUnitSizePrecisionBytes_);
  bitstreamStat.incrHeader(headerSize);

  for (auto& data : bitstream.vector()) {
    os << data;
  }
}

}  // namespace jpcc::encoder