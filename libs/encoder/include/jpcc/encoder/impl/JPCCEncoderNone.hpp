#include <sstream>

#include <io_tlv.h>

namespace jpcc::encoder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCEncoderNone<PointT>::JPCCEncoderNone(const JPCCEncoderParameter& parameter) : JPCCEncoder<PointT>(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCEncoderNone<PointT>::isConvertFromPCLThreadSafe() {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderNone<PointT>::convertFromPCL(const FramePtr<PointT>& pclFrame, shared_ptr<void>& frame) {
  frame = pclFrame;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderNone<PointT>::encode(const shared_ptr<void>& frame, std::vector<char>& encodedBytes) {
  const auto _frame = std::static_pointer_cast<Frame<PointT>>(frame);

  std::stringstream os;

  uint32_t length = uint32_t(_frame->size());

  os.put(char(length >> 24));
  os.put(char(length >> 16));
  os.put(char(length >> 8));
  os.put(char(length >> 0));

  for (auto& point : _frame->points) {
    os.write(reinterpret_cast<char*>(&point.x), sizeof(((PointT*)0)->x));
    os.write(reinterpret_cast<char*>(&point.y), sizeof(((PointT*)0)->y));
    os.write(reinterpret_cast<char*>(&point.z), sizeof(((PointT*)0)->z));
  }

#if !defined(NDEBUG)
  size_t oldSize = encodedBytes.size();
#endif
  std::string tmpString = os.str();
  for (char& i : tmpString) { encodedBytes.push_back(i); }
  assert((_frame->size() * (sizeof(((PointT*)0)->x) + sizeof(((PointT*)0)->y) + sizeof(((PointT*)0)->z)) + 4) ==
         (encodedBytes.size() - oldSize));
}

}  // namespace jpcc::encoder