namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCPointSet3::toPcl(const PclFramePtr<PointT>& pclFrame) const {
  pclFrame->header.seq = static_cast<decltype(((pcl::PCLHeader*)nullptr)->seq)>(this->getFrameNumber());
  pclFrame->resize(this->getPointCount());

  for (int i = 0; i < pclFrame->size(); i++) {
    (*pclFrame)[i] = PointT((*this)[i][0], (*this)[i][1], (*this)[i][2]);
    if constexpr (pcl::traits::has_intensity_v<PointT>) {
      if (this->hasReflectances()) {
        pclFrame->points[i].intensity = this->getReflectance(i);
      }
    }
    if constexpr (pcl::traits::has_normal_v<PointT>) {
      if (this->hasNormals()) {
        pclFrame->points[i].normal_x = this->getNormal(i)[0];
        pclFrame->points[i].normal_y = this->getNormal(i)[1];
        pclFrame->points[i].normal_z = this->getNormal(i)[2];
      }
    }
  }
  assert(this->getPointCount() == pclFrame->size());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
PclFramePtr<PointT> JPCCPointSet3::toPcl() const {
  auto pclFrame = std::make_shared<PclFrame<PointT>>();
  this->toPcl<PointT>(pclFrame);
  return pclFrame;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCPointSet3::fromPcl(const PclFramePtr<PointT>& pclFrame) {
  this->setFrameNumber((Index)pclFrame->header.seq);
  this->resize(pclFrame->size());

  if constexpr (pcl::traits::has_intensity_v<PointT>) {
    this->addReflectances();
  }
  if constexpr (pcl::traits::has_normal_v<PointT>) {
    this->addNormals();
  }
  for (int i = 0; i < this->getPointCount(); i++) {
    (*this)[i] =
        PointType{(PointValueType)(*pclFrame)[i].x, (PointValueType)(*pclFrame)[i].y, (PointValueType)(*pclFrame)[i].z};
    if constexpr (pcl::traits::has_intensity_v<PointT>) {  //
      this->getReflectance(i) = pclFrame->points[i].intensity;
    }
    if constexpr (pcl::traits::has_normal_v<PointT>) {
      this->getNormal(i)[0] = pclFrame->points[i].normal_x;
      this->getNormal(i)[1] = pclFrame->points[i].normal_y;
      this->getNormal(i)[2] = pclFrame->points[i].normal_z;
    }
  }
  assert(this->getPointCount() == pclFrame->size());
}

}  // namespace jpcc
