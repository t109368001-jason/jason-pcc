#include <jpcc/coder/JPCCEncoderTMC3.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename JPCCEncoder<PointT>::Ptr JPCCEncoderAdapter::build(const JPCCEncoderParameter& parameter) {
  if (parameter.backendType == CoderBackendType::TMC3) {
    return make_shared<JPCCEncoderTMC3<PointT>>(parameter);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
}

}  // namespace jpcc::coder