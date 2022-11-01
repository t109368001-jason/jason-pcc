#include <jpcc/common/JPCCPointSet3.h>

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCPointSet3::subset(JPCCPointSet3& frame, const Indices& indices) {
  frame.resize(indices.size());
  Index indexFrame = 0;
  for (Index index : indices) {
    frame[indexFrame] = (*this)[index];
    if (hasColors()) { frame.getColor(indexFrame) = this->getColor(index); }
    if (hasReflectances()) { frame.getReflectance(indexFrame) = this->getReflectance(index); }
    if (hasFrameIndex()) { frame.getFrameIndex(indexFrame) = this->getFrameIndex(index); }
    if (hasLaserAngles()) { frame.getLaserAngle(indexFrame) = this->getLaserAngle(index); }
    if (hasNormal()) { frame.getNormal(indexFrame) = this->getNormal(index); }
  }
}

}  // namespace jpcc