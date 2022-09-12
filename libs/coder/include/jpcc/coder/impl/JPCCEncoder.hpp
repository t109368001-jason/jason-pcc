#include <execution>

#include <boost/range/counting_range.hpp>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCEncoder<PointT>::JPCCEncoder(const JPCCEncoderParameter& parameter) : parameter_(parameter) {}

}  // namespace jpcc::coder