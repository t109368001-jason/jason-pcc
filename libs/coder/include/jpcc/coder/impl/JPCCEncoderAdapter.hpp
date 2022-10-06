#include <jpcc/coder/JPCCEncoderNone.h>
#include <jpcc/coder/JPCCEncoderTMC3.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCEncoderAdapter<PointT>::JPCCEncoderAdapter(const JPCCEncoderParameter& dynamicParameter,
                                               const JPCCEncoderParameter& staticParameter) {
  if (dynamicParameter.backendType == CoderBackendType::NONE) {
    dynamicEncoder_ = make_shared<JPCCEncoderNone<PointT>>(dynamicParameter);
  } else if (dynamicParameter.backendType == CoderBackendType::TMC3) {
    dynamicEncoder_ = make_shared<JPCCEncoderTMC3<PointT>>(dynamicParameter);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
  if (staticParameter.backendType == CoderBackendType::NONE) {
    staticEncoder_        = make_shared<JPCCEncoderNone<PointT>>(staticParameter);
    staticAddedEncoder_   = make_shared<JPCCEncoderNone<PointT>>(staticParameter);
    staticRemovedEncoder_ = make_shared<JPCCEncoderNone<PointT>>(staticParameter);
  } else if (staticParameter.backendType == CoderBackendType::TMC3) {
    staticEncoder_        = make_shared<JPCCEncoderTMC3<PointT>>(staticParameter);
    staticAddedEncoder_   = make_shared<JPCCEncoderTMC3<PointT>>(staticParameter);
    staticRemovedEncoder_ = make_shared<JPCCEncoderTMC3<PointT>>(staticParameter);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderAdapter<PointT>::convertFromPCL(IJPCCEncoderContext<PointT>& context, const bool parallel) {
  dynamicEncoder_->convertFromPCL(context.getDynamicPclFrames(), context.getDynamicFrames(), parallel);
  staticEncoder_->convertFromPCL(context.getStaticPclFrames(), context.getStaticFrames(), parallel);
  staticAddedEncoder_->convertFromPCL(context.getStaticAddedPclFrames(), context.getStaticAddedFrames(), parallel);
  staticRemovedEncoder_->convertFromPCL(context.getStaticRemovedPclFrames(), context.getStaticRemovedFrames(),
                                        parallel);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderAdapter<PointT>::encode(IJPCCEncoderContext<PointT>& context, const bool parallel) {
  dynamicEncoder_->encode(context.getDynamicFrames(), context.getDynamicEncodedBytes(), parallel);
  staticEncoder_->encode(context.getStaticFrames(), context.getStaticEncodedBytes(), parallel);
  staticAddedEncoder_->encode(context.getStaticAddedFrames(), context.getStaticAddedEncodedBytes(), parallel);
  staticRemovedEncoder_->encode(context.getStaticRemovedFrames(), context.getStaticRemovedEncodedBytes(), parallel);
}
}  // namespace jpcc::coder