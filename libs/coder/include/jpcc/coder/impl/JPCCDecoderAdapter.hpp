#include <jpcc/coder/JPCCDecoderNone.h>
#include <jpcc/coder/JPCCDecoderTMC3.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename JPCCDecoder<PointT>::Ptr JPCCDecoderAdapter::build(const JPCCDecoderParameter& parameter) {
  if (parameter.backendType == CoderBackendType::NONE) {
    return make_shared<JPCCDecoderNone<PointT>>(parameter);
  } else if (parameter.backendType == CoderBackendType::TMC3) {
    return make_shared<JPCCDecoderTMC3<PointT>>(parameter);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
}

}  // namespace jpcc::coder