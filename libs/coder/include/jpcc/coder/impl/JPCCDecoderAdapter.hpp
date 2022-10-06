#include <jpcc/coder/JPCCDecoderNone.h>
#include <jpcc/coder/JPCCDecoderTMC3.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCDecoderAdapter<PointT>::JPCCDecoderAdapter(const JPCCDecoderParameter& dynamicParameter,
                                               const JPCCDecoderParameter& staticParameter) {
  if (dynamicParameter.backendType == CoderBackendType::NONE) {
    dynamicDecoder_ = make_shared<JPCCDecoderNone<PointT>>(dynamicParameter);
  } else if (dynamicParameter.backendType == CoderBackendType::TMC3) {
    dynamicDecoder_ = make_shared<JPCCDecoderTMC3<PointT>>(dynamicParameter);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
  if (staticParameter.backendType == CoderBackendType::NONE) {
    staticDecoder_        = make_shared<JPCCDecoderNone<PointT>>(staticParameter);
    staticAddedDecoder_   = make_shared<JPCCDecoderNone<PointT>>(staticParameter);
    staticRemovedDecoder_ = make_shared<JPCCDecoderNone<PointT>>(staticParameter);
  } else if (staticParameter.backendType == CoderBackendType::TMC3) {
    staticDecoder_        = make_shared<JPCCDecoderTMC3<PointT>>(staticParameter);
    staticAddedDecoder_   = make_shared<JPCCDecoderTMC3<PointT>>(staticParameter);
    staticRemovedDecoder_ = make_shared<JPCCDecoderTMC3<PointT>>(staticParameter);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderAdapter<PointT>::decode(IJPCCDecoderContext<PointT>& context,
                                        const size_t                 frameCount,
                                        const bool                   parallel) {
  std::istringstream dynamicIs(
      std::string(context.getDynamicEncodedBytes().begin(), context.getDynamicEncodedBytes().end()));
  std::istringstream staticIs(
      std::string(context.getStaticEncodedBytes().begin(), context.getStaticEncodedBytes().end()));
  std::istringstream staticAddedReconIs(
      std::string(context.getStaticAddedEncodedBytes().begin(), context.getStaticAddedEncodedBytes().end()));
  std::istringstream staticRemovedIs(
      std::string(context.getStaticRemovedEncodedBytes().begin(), context.getStaticRemovedEncodedBytes().end()));
  context.getDynamicReconstructFrames().resize(frameCount);
  context.getStaticReconstructFrames().resize(frameCount);
  context.getStaticAddedReconstructFrames().resize(frameCount);
  context.getStaticRemovedReconstructFrames().resize(frameCount);
  dynamicDecoder_->decode(dynamicIs, context.getDynamicReconstructFrames(), parallel);
  staticDecoder_->decode(staticIs, context.getStaticReconstructFrames(), parallel);
  staticAddedDecoder_->decode(staticAddedReconIs, context.getStaticAddedReconstructFrames(), parallel);
  staticRemovedDecoder_->decode(staticRemovedIs, context.getStaticRemovedReconstructFrames(), parallel);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderAdapter<PointT>::convertToPCL(IJPCCDecoderContext<PointT>& context, const bool parallel) {
  dynamicDecoder_->convertToPCL(context.getDynamicReconstructFrames(), context.getDynamicReconstructPclFrames(),
                                parallel);
  staticDecoder_->convertToPCL(context.getStaticReconstructFrames(), context.getStaticReconstructPclFrames(), parallel);
  staticAddedDecoder_->convertToPCL(context.getStaticAddedReconstructFrames(),
                                    context.getStaticAddedReconstructPclFrames(), parallel);
  staticRemovedDecoder_->convertToPCL(context.getStaticRemovedReconstructFrames(),
                                      context.getStaticRemovedReconstructPclFrames(), parallel);
}

}  // namespace jpcc::coder