#include <jpcc/encoder/JPCCEncoderTMC3.h>

namespace jpcc::encoder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename JPCCEncoder<PointT>::Ptr JPCCEncoderAdapter::build(const JPCCEncoderParameter& parameter) {
  if (parameter.backendType == EncoderBackendType::TMC3) {
    return make_shared<JPCCEncoderTMC3<PointT>>(parameter);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
}

}  // namespace jpcc::encoder