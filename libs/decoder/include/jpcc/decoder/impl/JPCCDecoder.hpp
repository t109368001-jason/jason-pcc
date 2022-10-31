#include <execution>

#include <boost/range/counting_range.hpp>

namespace jpcc::decoder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCDecoder<PointT>::isConvertToPCLThreadSafe() {
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoder<PointT>::convertToPCL(std::vector<shared_ptr<void>>& reconstructFrames,
                                       GroupOfFrame<PointT>&          reconstructPclFrames,
                                       bool                           parallel) {
  reconstructPclFrames.clear();
  reconstructPclFrames.resize(reconstructFrames.size());
  if (!parallel || !isConvertToPCLThreadSafe()) {
    for (size_t i = 0; i < reconstructFrames.size(); i++) {
      this->convertToPCL(reconstructFrames[i], reconstructPclFrames[i]);
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, reconstructFrames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->convertToPCL(reconstructFrames[i], reconstructPclFrames[i]);
                  });
  }
}

}  // namespace jpcc::decoder