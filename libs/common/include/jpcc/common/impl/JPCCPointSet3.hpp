namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCPointSet3::toPcl(const PclFramePtr<PointT>& pclFrame) const {
  pclFrame->header.seq = static_cast<decltype(((pcl::PCLHeader*)nullptr)->seq)>(this->getFrameNumber());
  pclFrame->resize(this->getPointCount());

  for (int i = 0; i < pclFrame->size(); i++) {
    (*pclFrame)[i] = PointT((*this)[i].x(), (*this)[i].y(), (*this)[i].z());
    if constexpr (pcl::traits::has_color_v<PointT>) {
      if (this->hasColors()) {
        pclFrame->points[i].r = this->getColor(i).x();
        pclFrame->points[i].g = this->getColor(i).y();
        pclFrame->points[i].b = this->getColor(i).z();
      }
    }
    if constexpr (pcl::traits::has_intensity_v<PointT>) {
      if (this->hasReflectances()) { pclFrame->points[i].intensity = this->getReflectance(i); }
    }
    if constexpr (pcl::traits::has_normal_v<PointT>) {
      if (this->hasNormal()) {
        pclFrame->points[i].normal_x = this->getNormal(i).x();
        pclFrame->points[i].normal_y = this->getNormal(i).y();
        pclFrame->points[i].normal_z = this->getNormal(i).z();
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

  if constexpr (pcl::traits::has_color_v<PointT>) { this->addColors(); }
  if constexpr (pcl::traits::has_intensity_v<PointT>) { this->addReflectances(); }
  if constexpr (pcl::traits::has_normal_v<PointT>) { this->addNormal(); }
  for (int i = 0; i < this->getPointCount(); i++) {
    (*this)[i] = PointType((*pclFrame)[i].x, (*pclFrame)[i].y, (*pclFrame)[i].z);
    if constexpr (pcl::traits::has_color_v<PointT>) {
      this->getColor(i).x() = pclFrame->points[i].r;
      this->getColor(i).y() = pclFrame->points[i].g;
      this->getColor(i).z() = pclFrame->points[i].b;
    }
    if constexpr (pcl::traits::has_intensity_v<PointT>) {  //
      this->getReflectance(i) = pclFrame->points[i].intensity;
    }
    if constexpr (pcl::traits::has_normal_v<PointT>) {
      this->getNormal(i).x() = pclFrame->points[i].normal_x;
      this->getNormal(i).y() = pclFrame->points[i].normal_y;
      this->getNormal(i).z() = pclFrame->points[i].normal_z;
    }
  }
  assert(this->getPointCount() == pclFrame->size());
}

}  // namespace jpcc
